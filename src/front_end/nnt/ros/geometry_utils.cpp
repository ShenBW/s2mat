#include <nnt/ros/geometry_utils.h>
#include <nnt/ros/params.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

// #include <nnt/base/stl_helpers.h>
#include <Eigen/Eigenvalues>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace nnt
{
// Static variable definition
GeometryUtils* GeometryUtils::s_instance = NULL;

GeometryUtils::GeometryUtils() : m_transformListener()
{
  ROS_ASSERT_MSG(s_instance == NULL, "Cannot create multiple instances of GeometryUtils singleton!");
  s_instance = this;
}

std::string GeometryUtils::getWorldFrame()
{
  return nnt::Params::get<std::string>("world_frame", "odom");
}

bool GeometryUtils::lookupTransformIntoWorldFrame(ros::Time stamp, const std::string& sourceFrame,
                                                  Eigen::Affine3d& resultingTransform)
{
  tf::StampedTransform tfTransform;
  const std::string targetFrame = getWorldFrame();

  try
  {
    m_transformListener.waitForTransform(targetFrame, sourceFrame, stamp,
                                         ros::Duration(nnt::Params::get<double>("transform_timeout", 0.2)));
    m_transformListener.lookupTransform(targetFrame, sourceFrame, stamp, tfTransform);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR_STREAM_THROTTLE(2.0, "Could not determine transform from observation frame \""
                                       << sourceFrame << "\" into fixed world frame \"" << targetFrame << "\", "
                                       << "which may lead to observations being dropped. This message will re-appear "
                                          "every 2 seconds. Reason: "
                                       << ex.what());
    return false;
  }

  tf::transformTFToEigen(tfTransform, resultingTransform);
  return true;
}

void GeometryUtils::convertClustersToObservations(jsk_recognition_msgs::BoundingBoxArray clusters_msg,
                                                  Observations& observations)
{
  // We need to convert all coordinates into our fixed world reference frame
  double currentTime = clusters_msg.header.stamp.toSec();
  Eigen::Affine3d transformIntoWorldFrame;
  if (!lookupTransformIntoWorldFrame(clusters_msg.header.stamp, clusters_msg.header.frame_id, transformIntoWorldFrame))
    return;

  // In case we are overriding the detector's measurement noise (see below)
  bool overrideMeasurementNoise = nnt::Params::get<bool>("overwrite_measurement_noise", false);
  const double measurementNoise = nnt::Params::get<double>("measurement_noise", 0.01);
  ObsMatrix newMeasurementNoiseMatrix = ObsMatrix::Identity() * measurementNoise;

  // Eigen::Affine3d transformIndentiy;
  // transformIndentiy.setIdentity();
  // Convert DetectedPerson instances (ROS messages) into Observation instances (our own format).
  foreach (const jsk_recognition_msgs::BoundingBox& cluster, clusters_msg.boxes)
  {
    geometry_msgs::PoseWithCovariance pose_with_cov;
    pose_with_cov.pose = cluster.pose;
    pose_with_cov.covariance = {
      0.1, 0., 0., 0.,  0., 0., 0., 0.1, 0., 0., 0.,  0., 0., 0., 0.1, 0., 0., 0.,
      0.,  0., 0., 0.1, 0., 0., 0., 0.,  0., 0., 0.1, 0., 0., 0., 0.,  0., 0., 0.1,
    };
    // Heuristic sanity check for detected person poses (if anything in the detector goes wrong or groundtruth
    // annotations are invalid)
    if (!posePassesSanityCheck(pose_with_cov, false))
    {
      ROS_WARN_STREAM("Pose of DetectedPerson "
                      << cluster.label
                      << " does not pass sanity check, will ignore this detection: " << pose_with_cov.pose);
      continue;
    }

    Observation::Ptr observation(new Observation);
    observation->id = cluster.label;
    observation->createdAt = currentTime;
    observation->confidence = 0.0;

    observation->dim_x = cluster.dimensions.x;
    observation->dim_y = cluster.dimensions.y;
    observation->dim_z = cluster.dimensions.z;

    observation->pos_x = cluster.pose.position.x;
    observation->pos_y = cluster.pose.position.y;
    observation->pos_z = cluster.pose.position.z;

    // vector<std::string> modalities;
    // boost::split(modalities, detectedPerson.modality, boost::is_any_of(","));

    // foreach(std::string modality, modalities) {
    //     boost::trim(modality, std::locale(""));
    //     observation->modalities.insert(modality);
    // }

    poseToMeanAndCovariance(pose_with_cov, observation->z, observation->R, transformIntoWorldFrame);
    // ROS_INFO_STREAM(transformIntoWorldFrame[]);

    // Overwrite measurement noise if activated
    if (overrideMeasurementNoise)
      observation->R = newMeasurementNoiseMatrix;

    observations.push_back(observation);

    // ROS_DEBUG("Received observation from ROS (ID=%d) at x=%.2f, y=%.2f", (unsigned int) detectedPerson.detection_id,
    // observation->z(0), observation->z(1));
  }

  // ROS_INFO_STREAM("Received " << observations.size() << " observation(s) from ROS!");
}

void GeometryUtils::convertClustersToObservations(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& clusters_msg,
                                                  Observations& observations)
{
  // Ignore if pointer is not set
  if (!clusters_msg)
    return;

  // We need to convert all coordinates into our fixed world reference frame
  double currentTime = clusters_msg->header.stamp.toSec();
  Eigen::Affine3d transformIntoWorldFrame;
  if (!lookupTransformIntoWorldFrame(clusters_msg->header.stamp, clusters_msg->header.frame_id,
                                     transformIntoWorldFrame))
    return;

  // In case we are overriding the detector's measurement noise (see below)
  bool overrideMeasurementNoise = nnt::Params::get<bool>("overwrite_measurement_noise", false);
  const double measurementNoise = nnt::Params::get<double>("measurement_noise", 0.01);
  ObsMatrix newMeasurementNoiseMatrix = ObsMatrix::Identity() * measurementNoise;

  // Eigen::Affine3d transformIndentiy;
  // transformIndentiy.setIdentity();
  // Convert DetectedPerson instances (ROS messages) into Observation instances (our own format).
  foreach (const jsk_recognition_msgs::BoundingBox& cluster, clusters_msg->boxes)
  {
    geometry_msgs::PoseWithCovariance pose_with_cov;
    pose_with_cov.pose = cluster.pose;
    pose_with_cov.covariance = {
      0.1, 0., 0., 0.,  0., 0., 0., 0.1, 0., 0., 0.,  0., 0., 0., 0.1, 0., 0., 0.,
      0.,  0., 0., 0.1, 0., 0., 0., 0.,  0., 0., 0.1, 0., 0., 0., 0.,  0., 0., 0.1,
    };
    // Heuristic sanity check for detected person poses (if anything in the detector goes wrong or groundtruth
    // annotations are invalid)
    if (!posePassesSanityCheck(pose_with_cov, false))
    {
      ROS_WARN_STREAM("Pose of DetectedPerson "
                      << cluster.label
                      << " does not pass sanity check, will ignore this detection: " << pose_with_cov.pose);
      continue;
    }

    Observation::Ptr observation(new Observation);
    observation->id = cluster.label;
    observation->createdAt = currentTime;
    observation->confidence = 0.0;

    observation->dim_x = cluster.dimensions.x;
    observation->dim_y = cluster.dimensions.y;
    observation->dim_z = cluster.dimensions.z;

    observation->pos_x = cluster.pose.position.x;
    observation->pos_y = cluster.pose.position.y;
    observation->pos_z = cluster.pose.position.z;

    // vector<std::string> modalities;
    // boost::split(modalities, detectedPerson.modality, boost::is_any_of(","));

    // foreach(std::string modality, modalities) {
    //     boost::trim(modality, std::locale(""));
    //     observation->modalities.insert(modality);
    // }

    poseToMeanAndCovariance(pose_with_cov, observation->z, observation->R, transformIntoWorldFrame);
    // ROS_INFO_STREAM(transformIntoWorldFrame[]);

    // Overwrite measurement noise if activated
    if (overrideMeasurementNoise)
      observation->R = newMeasurementNoiseMatrix;

    observations.push_back(observation);

    // ROS_DEBUG("Received observation from ROS (ID=%d) at x=%.2f, y=%.2f", (unsigned int) detectedPerson.detection_id,
    // observation->z(0), observation->z(1));
  }

  // ROS_INFO_STREAM("Received " << observations.size() << " observation(s) from ROS!");
}

void GeometryUtils::convertObservationsToClusters(const double currentTime, const Observations& observations,
                                                  jsk_recognition_msgs::BoundingBoxArray& clusters_msg)
{
  clusters_msg.header.stamp = ros::Time(currentTime);
  clusters_msg.header.frame_id = getWorldFrame();

  foreach (Observation::Ptr observation, observations)
  {
    jsk_recognition_msgs::BoundingBox cluster;
    cluster.label = observation->id;
    // detectedPerson.confidence = observation->confidence;

    StateVector z;
    z(0) = observation->z(0);
    z(1) = observation->z(1);
    for (int i = 2; i < STATE_VISUALIZATION_DIM; i++)
      z(i) = 0;

    StateMatrix R;
    for (int r = 0; r < STATE_VISUALIZATION_DIM; r++)
    {
      for (int c = 0; c < STATE_VISUALIZATION_DIM; c++)
      {
        if (r < 2 && c < 2)
          R(r, c) = observation->R(r, c);
        else
          R(r, c) = 0;
      }
    }

    geometry_msgs::TwistWithCovariance unusedTwist;
    geometry_msgs::PoseWithCovariance pose_with_cov;
    meanAndCovarianceToPoseAndTwist(z, R, pose_with_cov, unusedTwist);
    cluster.pose = pose_with_cov.pose;
    clusters_msg.boxes.push_back(cluster);
  }
}

void GeometryUtils::meanAndCovarianceToPoseAndTwist(const StateVector& x, const StateMatrix& C,
                                                    geometry_msgs::PoseWithCovariance& pose,
                                                    geometry_msgs::TwistWithCovariance& twist)
{
  // Some constants for determining the pose
  const double AVERAGE_ROTATION_VARIANCE = pow(10.0 / 180 * M_PI, 2);  // FIXME: determine from vx, vy?
  const double INFINITE_VARIANCE = 9999999;  // should not really use infinity here because then the covariance matrix
                                             // cannot be rotated (singularities!)

  StateVisVector xVis = x.head(STATE_VISUALIZATION_DIM);
  StateVisMatrix CVis = C.block<STATE_VISUALIZATION_DIM, STATE_VISUALIZATION_DIM>(0, 0);

  assert(xVis.size() % 2 == 0);
  const int numAxes = xVis.size() / 2;  // either 2 or 3
  const double vx = xVis(numAxes + 0), vy = xVis(numAxes + 1);

  // Set pose (=position + orientation)
  pose.pose.position.x = xVis(0);
  pose.pose.position.y = xVis(1);
  pose.pose.position.z = numAxes > 2 ? xVis(2) : 0.0;
  pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(atan2(vy, vx));  // determine orientation from current velocity estimate

  pose.covariance.fill(0.0);
  pose.covariance[2 * ROS_COV_DIM + 2] =
      INFINITE_VARIANCE;  // default variance of z position, might get overwritten below if numAxes > 2

  for (int row = 0; row < numAxes; row++)
  {
    for (int col = 0; col < numAxes; col++)
    {
      pose.covariance[row * ROS_COV_DIM + col] = CVis(row, col);  // copy position covariances
    }
  }

  pose.covariance[3 * ROS_COV_DIM + 3] = INFINITE_VARIANCE;          // variance of x rotation
  pose.covariance[4 * ROS_COV_DIM + 4] = INFINITE_VARIANCE;          // variance of y rotation
  pose.covariance[5 * ROS_COV_DIM + 5] = AVERAGE_ROTATION_VARIANCE;  // variance of z rotation

  // Set twist (=velocities)
  twist.twist.linear.x = xVis(numAxes + 0);
  twist.twist.linear.y = xVis(numAxes + 1);
  twist.twist.linear.z = numAxes > 2 ? xVis(numAxes + 2) : 0.0;

  twist.covariance.fill(0.0);
  twist.covariance[2 * ROS_COV_DIM + 2] =
      INFINITE_VARIANCE;  // default variance of z linear velocity, might get overwritten below if numAxes > 2

  for (int row = 0; row < numAxes; row++)
  {
    for (int col = 0; col < numAxes; col++)
    {
      twist.covariance[row * ROS_COV_DIM + col] = CVis(numAxes + row, numAxes + col);  // copy velocity covariances
    }
  }

  twist.covariance[3 * ROS_COV_DIM + 3] = INFINITE_VARIANCE;  // variance of x angular velocity
  twist.covariance[4 * ROS_COV_DIM + 4] = INFINITE_VARIANCE;  // variance of y angular velocity
  twist.covariance[5 * ROS_COV_DIM + 5] = INFINITE_VARIANCE;  // variance of z angular velocity
}

void GeometryUtils::meanAndCovarianceToPoseAndTwist(const StateVector& x, const StateMatrix& C, double heading,
                                                    geometry_msgs::PoseWithCovariance& pose,
                                                    geometry_msgs::TwistWithCovariance& twist)
{
  // Some constants for determining the pose
  const double AVERAGE_ROTATION_VARIANCE = pow(10.0 / 180 * M_PI, 2);  // FIXME: determine from vx, vy?
  const double INFINITE_VARIANCE = 9999999;  // should not really use infinity here because then the covariance matrix
                                             // cannot be rotated (singularities!)

  StateVisVector xVis = x.head(STATE_VISUALIZATION_DIM);
  StateVisMatrix CVis = C.block<STATE_VISUALIZATION_DIM, STATE_VISUALIZATION_DIM>(0, 0);

  assert(xVis.size() % 2 == 0);
  const int numAxes = xVis.size() / 2;  // either 2 or 3
  const double vx = xVis(numAxes + 0), vy = xVis(numAxes + 1);

  // Set pose (=position + orientation)
  pose.pose.position.x = xVis(0);
  pose.pose.position.y = xVis(1);
  pose.pose.position.z = numAxes > 2 ? xVis(2) : 0.0;
  pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(heading);  // determine orientation from current velocity estimate

  pose.covariance.fill(0.0);
  pose.covariance[2 * ROS_COV_DIM + 2] =
      INFINITE_VARIANCE;  // default variance of z position, might get overwritten below if numAxes > 2

  for (int row = 0; row < numAxes; row++)
  {
    for (int col = 0; col < numAxes; col++)
    {
      pose.covariance[row * ROS_COV_DIM + col] = CVis(row, col);  // copy position covariances
    }
  }

  pose.covariance[3 * ROS_COV_DIM + 3] = INFINITE_VARIANCE;          // variance of x rotation
  pose.covariance[4 * ROS_COV_DIM + 4] = INFINITE_VARIANCE;          // variance of y rotation
  pose.covariance[5 * ROS_COV_DIM + 5] = AVERAGE_ROTATION_VARIANCE;  // variance of z rotation

  // Set twist (=velocities)
  twist.twist.linear.x = xVis(numAxes + 0);
  twist.twist.linear.y = xVis(numAxes + 1);
  twist.twist.linear.z = numAxes > 2 ? xVis(numAxes + 2) : 0.0;

  twist.covariance.fill(0.0);
  twist.covariance[2 * ROS_COV_DIM + 2] =
      INFINITE_VARIANCE;  // default variance of z linear velocity, might get overwritten below if numAxes > 2

  for (int row = 0; row < numAxes; row++)
  {
    for (int col = 0; col < numAxes; col++)
    {
      twist.covariance[row * ROS_COV_DIM + col] = CVis(numAxes + row, numAxes + col);  // copy velocity covariances
    }
  }

  twist.covariance[3 * ROS_COV_DIM + 3] = INFINITE_VARIANCE;  // variance of x angular velocity
  twist.covariance[4 * ROS_COV_DIM + 4] = INFINITE_VARIANCE;  // variance of y angular velocity
  twist.covariance[5 * ROS_COV_DIM + 5] = INFINITE_VARIANCE;  // variance of z angular velocity
}

void GeometryUtils::poseToMeanAndCovariance(const geometry_msgs::PoseWithCovariance& pose, ObsVector& x, ObsMatrix& C,
                                            const Eigen::Affine3d& transformToApply)
{
  // ROS Pose is a 3D position + rotation relative to the message's coordinate frame
  Eigen::Affine3d poseInDetectionFrame;
  tf::poseMsgToEigen(pose.pose, poseInDetectionFrame);

  // ROS Covariance is a 6x6 matrix (xyz + xyz rotation) relative to the message's coordinate frame
  // We are not interested in pose rotation, so only take first 3 rows and columns
  Eigen::Matrix3d covInDetectionFrame;
  for (int row = 0; row < ROS_COV_DIM / 2; row++)
  {
    for (int col = 0; col < ROS_COV_DIM / 2; col++)
    {
      covInDetectionFrame(row, col) = pose.covariance[row * ROS_COV_DIM + col];
    }
  }

  // Transform pose and covariance into our reference frame, still in 3D
  Eigen::Affine3d poseInOurFrame = transformToApply * poseInDetectionFrame;
  Eigen::Vector3d positionInOurFrame = poseInOurFrame.translation();

  // For covariance, only the coordinate frame rotation is relevant (invariant w.r.t. translation)
  Eigen::Matrix3d detectionFrameToOurFrameRotation = transformToApply.linear().matrix();
  Eigen::Matrix3d covInOurFrame =
      detectionFrameToOurFrameRotation * covInDetectionFrame * detectionFrameToOurFrameRotation.transpose();

  // Convert from 3D to 2D by simply dropping the Z coordinate
  assert(OBS_DIM == 2);
  x = positionInOurFrame.head(OBS_DIM);
  C = covInOurFrame.topLeftCorner(OBS_DIM, OBS_DIM);
}

bool GeometryUtils::posePassesSanityCheck(const geometry_msgs::PoseWithCovariance& poseWithCovariance,
                                          bool checkOrientation)
{
  const geometry_msgs::Pose& pose = poseWithCovariance.pose;

  // Position
  // const float MAX_REASONABLE_DISTANCE_FROM_ORIGIN = 1000.0f; // in meters
  // double positionValues[] = { pose.position.x, pose.position.y, pose.position.z };
  // for(size_t i = 0; i < sizeof(positionValues) / sizeof(positionValues[0]); i++) {
  //     if(!isfinite(positionValues[i]) || abs(positionValues[i]) > MAX_REASONABLE_DISTANCE_FROM_ORIGIN) {
  //         ROS_WARN_STREAM("Suspicious coordinate value(s) in pose position encountered!");
  //         return false;
  //     }
  // }

  // Orientation
  if (checkOrientation)
  {
    double squaredSum = 0;
    double orientationValues[] = { pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w };
    for (size_t i = 0; i < sizeof(orientationValues) / sizeof(orientationValues[0]); i++)
    {
      if (!isfinite(orientationValues[i]))
      {
        ROS_WARN_STREAM("Non-finite pose orientation value(s) encountered!");
        return false;
      }
      squaredSum += orientationValues[i] * orientationValues[i];
    }

    if (abs(squaredSum - 1.0) > 0.05)
    {
      ROS_WARN_STREAM("Pose orientation quaternion is not a unit quaternion!");
      return false;
    }
  }

  // Positive semi-definiteness of covariance matrix (x, y coordinates only)
  Eigen::Matrix2d cov;
  for (int row = 0; row < 2; row++)
    for (int col = 0; col < 2; col++)
      cov(row, col) = poseWithCovariance.covariance[row * ROS_COV_DIM + col];

  Eigen::Vector2cd eigenvals = cov.eigenvalues();
  bool positiveSemiDefinite = eigenvals(0).real() > 0 && eigenvals(1).real() > 0;
  if (!positiveSemiDefinite)
  {
    ROS_WARN_STREAM("Pose covariance matrix X,Y part is not positive semi-definite: " << std::endl << cov);
    return false;
  }

  return true;
}

}  // namespace nnt
