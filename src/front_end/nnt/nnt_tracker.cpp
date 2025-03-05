#include <nnt/nnt_tracker.h>
#include <nnt/base/defs.h>
// #include <nnt/base/stl_helpers.h>
#include <nnt/ros/params.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace nnt
{
NNTracker::NNTracker(ros::NodeHandle& nodeHandle)
  : m_nodeHandle(nodeHandle), m_params(nodeHandle), m_geometryUtils(), m_tracker(NULL), m_timingInitialized(false)
{
  unsigned int queue_size = (unsigned)nnt::Params::get<int>("queue_size", 5);

  // Set up circular buffer for benchmarking cycle times
  m_lastCycleTimes.set_capacity(Params::get<int>("cycle_time_buffer_length", 50));  // = size of window for averaging

  // Create ROS publishers
  // m_trackedPersonsPublisher =
  // m_nodeHandle.advertise<spencer_tracking_msgs::TrackedPersons>("/spencer/perception/tracked_persons", queue_size);
  // m_trackedPersonsPublisher.setExpectedFrequency(20.0, 40.0);
  // m_trackedPersonsPublisher.setMaximumTimestampOffset(0.3, 0.1);
  // m_trackedPersonsPublisher.finalizeSetup();

  // m_trackedPersonsPublisher = m_nodeHandle.advertise<smat::TrackedObjects>("/spencer/perception/tracked_persons",
  // queue_size);

  // Forward prediction time for track center to take latencies into account
  m_forwardPredictTime =
      nnt::Params::get<double>("published_track_forward_predict_time",
                               0.1);  // in seconds (e.g. at 1.5m/s, shift centroid forward by 0.1*1.5=0.15m)

  m_tracker.reset(new nnt::NearestNeighborTracker(m_nodeHandle));
}

smat::TrackedObjects NNTracker::track(jsk_recognition_msgs::BoundingBoxArray clusters_msg)
{
  if (!m_timingInitialized)
  {
    m_startWallTime = m_wallTimeBefore = ros::WallTime::now();
    m_startClock = m_clockBefore = clock();
    m_timingInitialized = true;
  }

  // jsk_recognition_msgs::BoundingBoxArray::ConstPtr clusters_msg_ptr(new
  // jsk_recognition_msgs::BoundingBoxArray(clusters_msg));

  double currentTime = clusters_msg.header.stamp.toSec();
  ros::Time currentRosTime = clusters_msg.header.stamp;  // to make sure that timestamps remain exactly the same up to
                                                         // nanosecond precision (for ExactTime sync policy)

  // Convert DetectedPersons into Observation instances
  Observations observations;
  m_geometryUtils.convertClustersToObservations(clusters_msg, observations);

  // Save start time
  ros::WallTime startTime = ros::WallTime::now();
  clock_t startClock = clock();

  // Initiate a new tracking cycle (this is where the fun begins!)
  assert(m_tracker != NULL);
  const Tracks& newTracks = m_tracker->processCycle(currentTime, observations);

  // Save end time
  ros::WallTime endTime = ros::WallTime::now();
  double cycleTime = (endTime - startTime).toSec();
  m_lastCycleTimes.push_back(cycleTime);

  // Publish new tracks and statistics (cycle times)
  return publishTracks(currentRosTime, newTracks);
}

smat::TrackedObjects NNTracker::publishTracks(ros::Time currentRosTime, const Tracks& tracks)
{
  smat::TrackedObjects trackedPersons;
  trackedPersons.header.stamp = currentRosTime;
  trackedPersons.header.seq = m_tracker->getCurrentCycleNo();
  trackedPersons.header.frame_id = m_geometryUtils.getWorldFrame();

  foreach (Track::Ptr track, tracks)
  {
    smat::TrackedObject trackedPerson;

    trackedPerson.track_id = track->id;
    trackedPerson.age = ros::Duration(currentRosTime.toSec() - track->createdAt);

    switch (track->trackStatus)
    {
      case Track::MATCHED:
      case Track::NEW:
        trackedPerson.is_matched = true;
        trackedPerson.is_occluded = false;
        trackedPerson.detection_id = track->observation->id;
        break;
      case Track::MISSED:
        trackedPerson.is_matched = false;
        trackedPerson.is_occluded = false;
        break;
      case Track::OCCLUDED:
        trackedPerson.is_matched = false;
        trackedPerson.is_occluded = true;
    }

    StateVector xp = track->state->x().head(STATE_DIM);
    xp.head(OBS_DIM) =
        xp.head(OBS_DIM) -
        m_forwardPredictTime * track->state->x().head(2 * OBS_DIM).tail(OBS_DIM);  // not sure why minus sign is needed,
                                                                                   // but empirically shown to work
    m_geometryUtils.meanAndCovarianceToPoseAndTwist(xp, track->state->C(), trackedPerson.pose,
                                                    trackedPerson.twist);  // state estimate (update)

    double abs_vel = sqrt(trackedPerson.twist.twist.linear.x * trackedPerson.twist.twist.linear.x +
                          trackedPerson.twist.twist.linear.y * trackedPerson.twist.twist.linear.y);
    // Heuristic sanity check for tracked person poses (if anything in the tracker goes wrong)
    if (!m_geometryUtils.posePassesSanityCheck(trackedPerson.pose, true))
    {
      // Output track state history to console, best viewed using rqt Logger Console plugin
      if (ROSCONSOLE_MIN_SEVERITY <= ROSCONSOLE_SEVERITY_WARN)
      {
        std::stringstream ss;
        size_t historyIndex = track->stateHistory.size();
        foreach (FilterState::Ptr historicState, track->stateHistory)
        {
          ss << "\n\n=== Historic motion filter state #" << --historyIndex << ": ===\n\n" << historicState << "\n";
        }

        ROS_WARN_STREAM("Pose of TrackedPerson "
                        << trackedPerson.track_id << " first tracked " << trackedPerson.age.toSec()
                        << " sec ago does not pass sanity check, will not publish this track:\n\n"
                        << trackedPerson << "\n\n-----------------\n\n"
                        << "Offending track's motion filter state history:" << ss.str());
      }

      // Skip publishing this track.
      continue;
    }

    // if (abs_vel < 0.2 || trackedPerson.age.toSec() < 2.0)
    // {
    //     continue;
    // }

    if (track->observation)
    {
      trackedPerson.dimensions.x = track->observation->dim_x;
      trackedPerson.dimensions.y = track->observation->dim_y;
      trackedPerson.dimensions.z = track->observation->dim_z;

      trackedPerson.pose.pose.position.z = track->observation->pos_z;
    }

    trackedPersons.tracks.push_back(trackedPerson);
  }

  // Publish tracked persons
  // ROS_INFO("Publishing %zi tracked persons!", tracks.size());
  // m_trackedPersonsPublisher.publish(trackedPersons);
  return trackedPersons;
}

}  // end of namespace nnt
