#ifndef _GEOMETRY_UTILS_H
#define _GEOMETRY_UTILS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <string>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <nnt/base/defs.h>
#include <nnt/data/observation.h>
#include <Eigen/Geometry>

namespace nnt
{
class GeometryUtils
{
public:
  /// Constructor.
  GeometryUtils();

  /// Returns the singleton instance, if any
  static GeometryUtils& getInstance()
  {
    ROS_ASSERT(s_instance != NULL);
    return *s_instance;
  }

  /// Returns the ID of the fixed world frame used for tracking.
  std::string getWorldFrame();

  /// Lookup the transform from the given source frame into the fixed world frame, for the given timestamp. Returns an
  /// Eigen::Affine3d transform.
  bool lookupTransformIntoWorldFrame(ros::Time stamp, const std::string& sourceFrame,
                                     Eigen::Affine3d& resultingTransform);

  /// Initialize a PoseWithCovariance and TwistWithCovariance message from the given mean vector and covariance matrix.
  void meanAndCovarianceToPoseAndTwist(const StateVector& x, const StateMatrix& C,
                                       geometry_msgs::PoseWithCovariance& pose,
                                       geometry_msgs::TwistWithCovariance& twist);

  void meanAndCovarianceToPoseAndTwist(const StateVector& x, const StateMatrix& C, double heading,
                                       geometry_msgs::PoseWithCovariance& pose,
                                       geometry_msgs::TwistWithCovariance& twist);

  // Extracts mean and covariance from a given pose, also taking the transform from the given source frame into the
  // fixed world frame into account.
  void poseToMeanAndCovariance(const geometry_msgs::PoseWithCovariance& pose, ObsVector& x, ObsMatrix& C,
                               const Eigen::Affine3d& transformToApply);

  /// Checks if a pose has reasonable values (no NaN values, coordinates not extremely large, covariance matrix X/Y
  /// positive semi-definite, etc.)
  bool posePassesSanityCheck(const geometry_msgs::PoseWithCovariance& pose, bool checkOrientation = true);

  /// Converts the ROS BoundingBoxArray messages into Observation instances, and transforms them into our world frame.
  void convertClustersToObservations(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& clusters_msg,
                                     Observations& observations);

  void convertClustersToObservations(jsk_recognition_msgs::BoundingBoxArray clusters_msg, Observations& observations);

  /// Converts Observations into ROS BoundingBoxArray messages (for debugging usually).
  void convertObservationsToClusters(const double currentTime, const Observations& observations,
                                     jsk_recognition_msgs::BoundingBoxArray& clusters_msg);

private:
  tf::TransformListener m_transformListener;
  static GeometryUtils* s_instance;
};

}  // namespace nnt

#endif
