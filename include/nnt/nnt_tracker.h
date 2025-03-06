#ifndef _NNT_ROS_H
#define _NNT_ROS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/circular_buffer.hpp>

#include <s2mat/TrackedObjects.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <memory>

// #include <spencer_diagnostics/publisher.h>
#include <nnt/nnt.h>
#include <nnt/base/tracker.h>
#include <nnt/ros/params.h>
#include <nnt/ros/geometry_utils.h>
#include <nnt/data/observation.h>
#include <nnt/data/track.h>

namespace nnt
{
/// A wrapper around the tracker which receives spencer_tracking_msgs/DetectedPersons as input, converts them to
/// Observations, invokes the tracker's processCycle() method, and outputs the resulting Tracks as
/// spencer_tracking_msgs/TrackedPersons.
class NNTracker
{
public:
  /// Creates a ROS interface that subscribes to a DetectedPersons topic using the provided nodeHandle, and
  /// reads parameters using the provided privateNodeHandle.
  NNTracker(ros::NodeHandle& nodeHandle);

  s2mat::TrackedObjects track(jsk_recognition_msgs::BoundingBoxArray clusters_msg);

private:
  /// Publishes tracks on ROS after completion of a tracking cycle.
  s2mat::TrackedObjects publishTracks(ros::Time currentRosTime, const Tracks& tracks);

  // Publishes statistics, such as average processing cycle duration and processing rate.
  // void publishStatistics(ros::Time currentRosTime, const unsigned int numberTracks);

  /// ROS handles for publisher and subscriber management
  ros::NodeHandle m_nodeHandle;
  ros::Subscriber m_detectedPersonsSubscriber;

  // spencer_diagnostics::MonitoredPublisher m_trackedPersonsPublisher;
  // ros::Publisher m_trackedPersonsPublisher;
  // ros::Publisher m_averageProcessingRatePublisher, m_averageCycleTimePublisher, m_trackCountPublisher,
  // m_averageLoadPublisher, m_timingMetricsPublisher;

  /// Tracker instance
  std::shared_ptr<nnt::NearestNeighborTracker> m_tracker;

  /// Tracker parameters
  Params m_params;

  /// Utility classes
  GeometryUtils m_geometryUtils;
  boost::circular_buffer<double> m_lastCycleTimes;
  clock_t m_startClock;
  clock_t m_clockBefore;
  ros::WallTime m_startWallTime;
  ros::WallTime m_wallTimeBefore;
  bool m_timingInitialized;
  bool m_overwriteMeasurementNoise;
  float m_forwardPredictTime;
};

}  // end of namespace nnt

#endif
