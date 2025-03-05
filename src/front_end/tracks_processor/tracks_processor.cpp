#include "front_end/tracks_processor/tracks_processor.h"

namespace smat
{
TracksProcessor::TracksProcessor()
{
}

TracksProcessor::~TracksProcessor()
{
}

void TracksProcessor::calculateTrackDimension(pcl::ExtractIndices<PointType>& extractor,
                                              const std::vector<int>& cluster_idx, TrackedObject track,
                                              Eigen::Vector3f& extent)
{
  PointCloudPtr cluster_global(new PointCloud());
  PointCloudPtr cluster_local(new PointCloud());

  boost::shared_ptr<std::vector<int>> idx_ptr = boost::make_shared<std::vector<int>>(cluster_idx);
  extractor.setIndices(idx_ptr);
  extractor.setNegative(false);
  cluster_global->clear();
  extractor.filter(*cluster_global);

  cluster_local->clear();
  pcl::transformPointCloud(*cluster_global, *cluster_local, poseToMatrix(track.pose.pose).inverse());

  Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest());
  Eigen::Vector3f min_point(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());
  for (auto point : cluster_local->points)
  {
    Eigen::Vector3f point_eigen(point.x, point.y, point.z);
    min_point << std::min(min_point.x(), point_eigen.x()), std::min(min_point.y(), point_eigen.y()),
        std::min(min_point.z(), point_eigen.z());
    max_point << std::max(max_point.x(), point_eigen.x()), std::max(max_point.y(), point_eigen.y()),
        std::max(max_point.z(), point_eigen.z());
  }

  extent = max_point - min_point;
}

void TracksProcessor::predictBboxFromTracking(TrackedObject track, BoundingBox& pred_box, double delta_time)
{
  double vx = track.twist.twist.linear.x;
  double vy = track.twist.twist.linear.y;

  double x = track.pose.pose.position.x;
  double y = track.pose.pose.position.y;

  tf::Quaternion quat;
  quat.setX(0.0);
  quat.setY(0.0);
  quat.setZ(track.pose.pose.orientation.z);
  quat.setW(track.pose.pose.orientation.w);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  double new_x = x + vx * delta_time;
  double new_y = y + vy * delta_time;

  pred_box.pose.position.x = new_x;
  pred_box.pose.position.y = new_y;
  pred_box.pose.position.z = track.pose.pose.position.z;

  pred_box.pose.orientation.x = track.pose.pose.orientation.x;
  pred_box.pose.orientation.y = track.pose.pose.orientation.y;
  pred_box.pose.orientation.z = track.pose.pose.orientation.z;
  pred_box.pose.orientation.w = track.pose.pose.orientation.w;
}

void TracksProcessor::measureTracks(const std::vector<PointCloudPtr>& submaps,
                                    const std::vector<geometry_msgs::Pose>& submaps_pose,
                                    const std::vector<TrackedObjects>& submaps_object,
                                    std::unordered_set<int>& success_tracking_traj)
{
  std::unordered_map<int, std::vector<TrackedObject>> tracking_traj;
  for (auto tracks : submaps_object)
  {
    for (auto track : tracks.tracks)
    {
      tracking_traj[track.track_id + 1.0].push_back(track);
    }
  }

  for (auto it = tracking_traj.begin(); it != tracking_traj.end(); ++it)
  {
    Eigen::Vector3f avg_twist = Eigen::Vector3f::Zero();
    Eigen::Vector3f extent = Eigen::Vector3f::Zero();

    Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest());
    Eigen::Vector3f min_point(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max());

    float max_tracking_age = std::numeric_limits<float>::lowest();
    float min_tracking_age = std::numeric_limits<float>::max();

    float max_vol = std::numeric_limits<float>::lowest();
    float min_vol = std::numeric_limits<float>::max();

    std::stringstream ss;
    float num_missed = 0.0;
    float num_matched = 0.0;
    for (auto track : it->second)
    {
      Eigen::Vector3f point_eigen(track.pose.pose.position.x, track.pose.pose.position.y, track.pose.pose.position.z);
      ss << " --- \n";
      ss << "age: " << track.age.toSec() << "\n";
      ss << "matched: " << (bool)track.is_matched << "\n";
      ss << "pose: \n " << point_eigen << "\n";
      if (!(bool)track.is_matched)
      {
        num_missed += 1.0;
        continue;
      }

      float vol = track.dimensions.x * track.dimensions.y * track.dimensions.z;

      Eigen::Vector3f twist(track.twist.twist.linear.x, track.twist.twist.linear.y, track.twist.twist.linear.z);

      avg_twist = avg_twist + twist;

      num_matched += 1.0;
      min_point << std::min(min_point.x(), point_eigen.x()), std::min(min_point.y(), point_eigen.y()),
          std::min(min_point.z(), point_eigen.z());
      max_point << std::max(max_point.x(), point_eigen.x()), std::max(max_point.y(), point_eigen.y()),
          std::max(max_point.z(), point_eigen.z());
      max_tracking_age = std::max(max_tracking_age, (float)track.age.toSec());
      min_tracking_age = std::min(min_tracking_age, (float)track.age.toSec());

      max_vol = std::max(max_vol, vol);
      min_vol = std::min(min_vol, vol);
    }

    if (max_vol < min_vol)
    {
      continue;
    }

    avg_twist /= (float)it->second.size();

    ss << " --- \n";
    ss << "max_point: \n" << max_point << "\n";
    ss << "min_point: \n" << min_point << "\n";

    extent = max_point - min_point;
    float duration = max_tracking_age - min_tracking_age;
    float matched_rate = (num_matched) / (num_missed + num_matched);

    // float vol_diff = 2 * (max_vol - min_vol) / (max_vol + min_vol);
    float vol_diff = max_vol - min_vol;

    ss << " --- \n";
    ss << "duration: " << duration << "\n";
    ss << "matched_rate: " << matched_rate << "\n";
    if (duration > 1.0 && extent.norm() / duration > 0.5 && extent.norm() > 1.0 && avg_twist.norm() > 0.5 &&
        matched_rate > 0.5 && vol_diff < 1.0)
    {
      success_tracking_traj.insert(it->first);
      // ROS_INFO_STREAM("\n === tracked track_id: " << it->first << " === \n" << ss.str());
    }
  }
}
}  // namespace smat