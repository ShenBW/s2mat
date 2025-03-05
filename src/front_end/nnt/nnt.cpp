#include <nnt/nnt.h>
#include <nnt/base/defs.h>
// #include <nnt/base/stl_helpers.h>
#include <nnt/ros/params.h>

#include <nnt/occlusion_handling/basic_occlusion_manager.h>

#include <nnt/data_association/greedy_nearest_neighbor_data_association.h>

#include <ros/ros.h>
#include <Eigen/LU>
#include <limits>
#include <map>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

template <typename L, typename R>
void appendTo(L& lhs, R const& rhs)
{
  lhs.insert(lhs.end(), rhs.begin(), rhs.end());
}

namespace nnt
{
NearestNeighborTracker::NearestNeighborTracker(ros::NodeHandle& nodeHandle)
  : m_nodeHandle(nodeHandle)
  , m_cycleCounter(0)
  , m_cycleTime(0.0)
  , m_deltaTime(0)
  , m_trackIdCounter(0)
  , m_dataAssociation()
{
  m_frameID = Params::get<std::string>("world_frame", "odom");

  // Get settings for IMM or simple Kalman Filter
  if (Params::get<bool>("use_imm", false))
  {
    ROS_INFO_STREAM("Using IMM filter for NNT");
    m_filter.reset(new IMMFilter(m_nodeHandle));
    // m_filter.reset(new EKF);
  }
  else
  {
    ROS_INFO_STREAM("Using Extendend Kalman filter for NNT");
    m_filter.reset(new EKF);
  }

  ROS_INFO("Using basic occlusion manager for NNT");
  m_occlusionManager.reset(new BasicOcclusionManager);

  m_occlusionManager->initializeOcclusionManager(m_nodeHandle);
  m_occlusionManager->setFrameIDofTracker(m_frameID);

  // Get setting for occlusion manager
  std::string dataAssociationStr = Params::get<std::string>("data_association_type", "greedy_nearest_neighbor");

  if (dataAssociationStr == "greedy_nearest_neighbor")
  {
    m_dataAssociation.reset(new GreedyNearestNeighborDataAssociation);
  }
  else
  {
    ROS_FATAL_STREAM("Data association method is invalid: " << dataAssociationStr.c_str());
  }
  ROS_INFO_STREAM("Selected data association method: \t" << dataAssociationStr);
  m_dataAssociation->initializeDataAssociation(m_nodeHandle);
}

NearestNeighborTracker::~NearestNeighborTracker()
{
}

const Tracks& NearestNeighborTracker::processCycle(double currentTime, const Observations& newObservations)
{
  // ROS_INFO_STREAM("Received " << newObservations.size() << " observations.");

  beginCycle(currentTime);

  predictTrackStates();

  predictMeasurements();

  Tracks occludedTracks =
      m_occlusionManager->manageOcclusionsBeforeDataAssociation(m_tracks, ros::Time(currentTime), m_frameID);
  // ROS_INFO_STREAM("Occlusion manager returned " << occludedTracks.size() << " tracks");
  Pairings pairings = m_dataAssociation->performDataAssociation(m_tracks, newObservations);

  Pairings reappearedParings =
      m_occlusionManager->occludedTrackAssociation(occludedTracks, newObservations, ros::Time(currentTime));

  Tracks mergedTracks;
  mergedTracks.reserve(m_tracks.size() + occludedTracks.size());  // preallocate memory
  mergedTracks.insert(mergedTracks.end(), m_tracks.begin(), m_tracks.end());
  mergedTracks.insert(mergedTracks.end(), occludedTracks.begin(), occludedTracks.end());
  m_tracks = mergedTracks;
  ROS_DEBUG_STREAM("After merge " << m_tracks.size() << " tracks");

  Pairings mergedPairings;
  mergedPairings.reserve(pairings.size() + reappearedParings.size());  // preallocate memory
  mergedPairings.insert(mergedPairings.end(), pairings.begin(), pairings.end());
  mergedPairings.insert(mergedPairings.end(), reappearedParings.begin(), reappearedParings.end());

  updateKalmanFilter(mergedPairings);

  // Any unmatched observations creates a new track, unless it is too close to an existing track
  // (less than a person radius).
  initNewTracksFromObservations(newObservations);

  deleteObsoleteTracks();

  deleteDuplicateTracks();

  endCycle();

  return m_tracks;
}

void NearestNeighborTracker::beginCycle(double currentTime)
{
  if (currentTime < m_cycleTime)
  {
    // This usually happens when playing back a bagfile and looping
    ROS_WARN(
        "Jump back in time detected. Deleting all existing tracks. Old time was %.3f, new time is %.3f (dt=%.3f sec)!",
        m_cycleTime, currentTime, currentTime - m_cycleTime);

    // Reset tracker state
    m_cycleTime = 0.0;
    m_cycleCounter = 0;
    m_trackIdCounter = 0;
    m_tracks.clear();
  }
  m_deltaTime = currentTime - m_cycleTime;
  m_cycleTime = currentTime;

  ROS_DEBUG("Beginning tracking cycle no. %lu. Time since last cycle: %.3f sec", m_cycleCounter, m_deltaTime);
}

void NearestNeighborTracker::endCycle()
{
  // Increase cycle counter
  m_cycleCounter++;

  ROS_DEBUG("End of tracking cycle %lu", m_cycleCounter - 1);
}

void NearestNeighborTracker::predictTrackStates()
{
  ROS_DEBUG("Predicting track states");

  foreach (Track::Ptr track, m_tracks)
  {
    if (track->trackStatus != Track::DELETED)
    {
      m_filter->setTransitionMatrix(track->state->x(), m_deltaTime);
      track->stateHistory.push_back(track->state->deepCopy());  // copy current state into history for later Debugging &
                                                                // duplicate track elimination
      m_filter->predictTrackState(track->state, m_deltaTime);
    }
  }
}

void NearestNeighborTracker::predictMeasurements()
{
  ROS_DEBUG("Predicting measurements");

  // For the moment, our "track-to-measurement model" is very simple: Just copy x and y position, and forget about the
  // vx, vy (measurements don't have velocities)
  ObsStateMatrix fixed_H = ObsStateMatrix::Zero();
  fixed_H(0, 0) = 1.0;
  fixed_H(1, 1) = 1.0;

  foreach (Track::Ptr track, m_tracks)
  {
    track->state->updateMeasurementPrediction(fixed_H);
  }
}

void NearestNeighborTracker::updateKalmanFilter(const Pairings& pairings)
{
  ROS_DEBUG("Updating Extended Kalman filters of all tracks");

  foreach (Pairing::Ptr pairing, pairings)
  {
    // Only update tracks in validated pairings
    if (pairing->validated)
    {
      assert(pairing->track->observation);
      m_filter->updateMatchedTrack(pairing->track->state, pairing);
    }
  }
}

bool NearestNeighborTracker::checkForClosebyExistingTrack(const Observation::Ptr& observation)
{
  // HACK: Avoid creating new tracks if there is a close-by track that has recently been matched
  bool closebyExistingTrack = false;
  foreach (Track::Ptr track, m_tracks)
  {
    if (std::max(track->numberOfConsecutiveOcclusions, track->numberOfConsecutiveMisses) < 5)
    {
      ObsVector diff = track->state->x().head(2) - observation->z;
      if (diff.norm() < 0.25)
      {
        closebyExistingTrack = true;
        break;
      }
    }
  }
  return closebyExistingTrack;
}

Track::Ptr NearestNeighborTracker::createTrack(const Observation::Ptr& observation)
{
  Track::Ptr newTrack(new Track);
  newTrack->id = m_trackIdCounter++;
  newTrack->trackStatus = Track::NEW;
  newTrack->observation = observation;
  newTrack->createdAt = m_cycleTime;
  newTrack->numberOfTotalMatches = newTrack->numberOfConsecutiveOcclusions = newTrack->numberOfConsecutiveMisses =
      newTrack->numberOfConsecutiveWeakMatches = 0;
  newTrack->model_idx = 0;
  newTrack->detectionProbability = 1.0;
  newTrack->stateHistory.set_capacity(Params::get<int>("state_history_length", 30));  // for DEBUGging & elimination of
                                                                                      // duplicate tracks
  return newTrack;
}

void NearestNeighborTracker::initNewTracksFromObservations(const Observations& newObservations)
{
  ROS_DEBUG("Initializing new tracks");

  Tracks newTracks;
  foreach (Observation::Ptr observation, newObservations)
  {
    if (!observation->matched)
    {
      // Make sure no track exists closeby, in case we have duplicate detections for some reason
      if (checkForClosebyExistingTrack(observation))
        continue;

      Track::Ptr newTrack = createTrack(observation);
      newTrack->state = m_filter->initializeTrackState(observation);
      newTracks.push_back(newTrack);
    }
  }

  if (!newTracks.empty())
    ROS_DEBUG("%zu new track(s) have been initialized!", newTracks.size());
  appendTo(m_tracks, newTracks);
}

void NearestNeighborTracker::deleteDuplicateTracks()
{
  ROS_DEBUG("Deleting duplicate tracks");

  const size_t NUM_ENTRIES_TO_COMPARE = Params::get<int>("duplicate_track_num_history_entries_to_compare", 10);
  const size_t NUM_ENTRIES_MUST_MATCH = Params::get<int>("duplicate_track_num_history_entries_must_match", 7);
  const double MAX_DISTANCE = Params::get<double>("duplicate_track_max_dist", 0.15);
  const double MAX_DISTANCE_SQUARED = MAX_DISTANCE * MAX_DISTANCE;

  // Extension: Remove duplicate tracks
  // Step 1: Find duplicate tracks and mark younger duplicate for deletion
  std::vector<size_t> tracksToDelete;
  for (size_t t1 = 0; t1 < m_tracks.size(); t1++)
  {
    Track::Ptr& t1_ptr = m_tracks[t1];
    int requiredStateHistoryLength = std::min(t1_ptr->stateHistory.capacity(), NUM_ENTRIES_TO_COMPARE);
    if (t1_ptr->stateHistory.size() < requiredStateHistoryLength)
      continue;  // not enough history available yet to compare tracks
    for (size_t t2 = t1 + 1; t2 < m_tracks.size(); t2++)
    {
      Track::Ptr& t2_ptr = m_tracks[t2];
      if (t2_ptr->stateHistory.size() < requiredStateHistoryLength)
        continue;  // not enough history available yet to compare tracks

      size_t numMatches = 0;
      for (size_t historyIndex = 0; historyIndex < requiredStateHistoryLength; historyIndex++)
      {
        // if (historyIndex - numMatches > requiredStateHistoryLength - NUM_ENTRIES_MUST_MATCH) break;
        FilterState::Ptr& s1 =
            t1_ptr->stateHistory[t1_ptr->stateHistory.size() - historyIndex - 1];  // get n-latest element
        FilterState::Ptr& s2 = t2_ptr->stateHistory[t2_ptr->stateHistory.size() - historyIndex - 1];

        ObsVector diff = s1->x().head(2) - s2->x().head(2);
        if (diff.squaredNorm() < MAX_DISTANCE_SQUARED)
          numMatches++;
      }

      if (numMatches >= NUM_ENTRIES_MUST_MATCH)
      {
        // Delete the younger / lower-quality track
        if (t1_ptr->numberOfTotalMatches < t2_ptr->numberOfTotalMatches)
        {
          tracksToDelete.push_back(t1);
        }
        else
        {
          tracksToDelete.push_back(t2);
        }
      }
    }
  }

  // ROS_INFO_STREAM("Deleting " << tracksToDelete.size() << " duplicated tracks.");

  if (tracksToDelete.size() > 0)
  {
    std::sort(tracksToDelete.begin(), tracksToDelete.end());

    // Delete marked tracks
    int trackIdx = -1;
    for (int i = tracksToDelete.size() - 1; i >= 0; i--)
    {
      if (tracksToDelete.at(i) == trackIdx)
        continue;  // Could happen that we want to delete same track more often
      trackIdx = tracksToDelete.at(i);
      m_tracks.erase(m_tracks.begin() + trackIdx);
    }
  }
}

void NearestNeighborTracker::deleteObsoleteTracks()
{
  m_occlusionManager->deleteOccludedTracks(m_tracks, ros::Time(m_cycleTime));
}

}  // end of namespace nnt
