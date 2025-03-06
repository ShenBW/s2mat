#include "back_end/back_end.h"

namespace s2mat
{
ScanToMapBackEnd::ScanToMapBackEnd(ros::NodeHandle nh)
  : nh_(nh), query_flag_(false), downsampling_flag_(true), max_nearest_size_(60)
{
  scans_points_.reset(new PointCloud());
  kdtree_scans_points_.reset(new pcl::KdTreeFLANN<PointType>());

  scans_points_->clear();

  readParameters();

  submap_pub_ = nh_.advertise<s2mat::Submap>("/static_submap", 1);
  static_submap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("static_submap", 1);
  dynamic_submap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dynamic_submap", 1);

  preprocess_scan_sub_ = nh_.subscribe<s2mat::Submap>("/preprocess_scan", 1, &ScanToMapBackEnd::scanCallback, this);
  query_sub_ =
      nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &ScanToMapBackEnd::queryScanCallback, this);
}

ScanToMapBackEnd::~ScanToMapBackEnd()
{
}

bool ScanToMapBackEnd::readParameters()
{
  nh_.param<int>("max_nearest_size", max_nearest_size_, max_nearest_size_);

  nh_.param<float>("scan_radius", scan_radius_, 15.0);
  nh_.param<float>("voxel_size", voxel_size_, 0.1);

  nh_.param<std::string>("local_frame", local_frame_, "base_link");
  nh_.param<std::string>("global_frame", global_frame_, "odom");

  nh_.param<bool>("downsampling_flag", downsampling_flag_, downsampling_flag_);
  return true;
}

void ScanToMapBackEnd::scanCallback(const s2mat::Submap::ConstPtr& msg)
{
  double start_time = ros::Time::now().toSec();

  std::unique_lock<std::mutex> lock(scan_lock_);
  geometry_msgs::Pose pose;
  pose.position.x = msg->pose.position.x;
  pose.position.y = msg->pose.position.y;
  pose.position.z = msg->pose.position.z;

  pose.orientation.x = msg->pose.orientation.x;
  pose.orientation.y = msg->pose.orientation.y;
  pose.orientation.z = msg->pose.orientation.z;
  pose.orientation.w = msg->pose.orientation.w;

  scans_pose_.push_back(pose);

  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
      .getRPY(roll, pitch, yaw);
  scans_yaw_.push_back(yaw);

  PointType point;
  point.x = msg->pose.position.x;
  point.y = msg->pose.position.y;
  point.z = msg->pose.position.z;
  scans_points_->points.push_back(point);

  PointCloudPtr scan(new PointCloud());
  PointCloudPtr scan_dst(new PointCloud());

  pcl::fromROSMsg(msg->pointcloud, *scan);

  if (downsampling_flag_)
  {
    octreeDownsamplingWithIntensity(scan, scan_dst, voxel_size_);
  }
  else
  {
    *scan_dst = *scan;
  }

  scans_pointcloud_.push_back(scan_dst);

  ROS_INFO_STREAM("[Back End]: scan process time: " << ros::Time::now().toSec() - start_time);
}

void ScanToMapBackEnd::getScans(const PointType& curr_point, std::vector<PointCloudPtr>& nearest_scans,
                                 std::vector<geometry_msgs::Pose>& nearest_scans_pose)
{
  std::unique_lock<std::mutex> lock(scan_lock_);

  kdtree_scans_points_->setInputCloud(scans_points_);

  std::vector<int> scan_knn_points_idx;
  std::vector<float> scan_knn_points_dist;
  kdtree_scans_points_->radiusSearch(curr_point, scan_radius_, scan_knn_points_idx, scan_knn_points_dist, 0);

  int nearest_size = 0;

  double curr_yaw = scans_yaw_[scan_knn_points_idx[0]];

  for (auto i : scan_knn_points_idx)
  {
    if (std::abs(scans_yaw_[i] - curr_yaw) < 0.79)
    {
      nearest_scans_pose.push_back(scans_pose_[i]);
      nearest_scans.push_back(scans_pointcloud_[i]);

      nearest_size++;
      if (nearest_size >= max_nearest_size_)
      {
        break;
      }
    }
  }
}

void ScanToMapBackEnd::pubSubmap(const PointCloudPtr& static_map, const PointType& point)
{
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*static_map, pointcloud_msg);

  s2mat::Submap map_msg;
  map_msg.pointcloud = pointcloud_msg;
  map_msg.pose.position.x = point.x;
  map_msg.pose.position.y = point.y;
  map_msg.pose.position.z = point.z;
  submap_pub_.publish(map_msg);
}

void ScanToMapBackEnd::queryScanCallback(const geometry_msgs::PointStampedConstPtr& point_msg)
{
  std::unique_lock<std::mutex> lock(query_lock_);
  query_point_.x = point_msg->point.x;
  query_point_.y = point_msg->point.y;
  query_point_.z = point_msg->point.z;

  query_flag_ = true;
}

bool ScanToMapBackEnd::isRemove(PointType& point)
{
  std::unique_lock<std::mutex> lock(query_lock_);
  if (query_flag_)
  {
    point.x = query_point_.x;
    point.y = query_point_.y;
    point.z = query_point_.z;
    query_flag_ = false;
    return true;
  }
  return false;
}

void ScanToMapBackEnd::removeThread()
{
  ros::Rate loop(10);
  while (ros::ok())
  {
    PointType point;
    if (isRemove(point))
    {
      PointCloudPtr static_global_map(new PointCloud());
      PointCloudPtr dynamic_global_map(new PointCloud());
      PointCloudPtr global_map(new PointCloud());

      occupancyEstimation(point, static_global_map, dynamic_global_map);

      if (static_global_map->points.size() > 0)
      {
        pubSubmap(static_global_map, query_point_);
        publishMap(static_submap_pub_, static_global_map, ros::Time::now(), "odom");
        publishMap(dynamic_submap_pub_, dynamic_global_map, ros::Time::now(), "odom");
      }
    }
    loop.sleep();
  }
}

void ScanToMapBackEnd::run()
{
  std::thread remove_thread = std::thread(&ScanToMapBackEnd::removeThread, this);
  ros::Rate loop(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }
}
}  // namespace s2mat