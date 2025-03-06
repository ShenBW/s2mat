#include "map_merger/map_merger.h"

namespace s2mat
{
MapMerger::MapMerger(ros::NodeHandle nh)
  : nh_(nh), scans_count_(0), curr_scans_num_(0), curr_submap_id_(0), local_radius_(1000.0), output_local_(false)
{
  submap_poses_.reset(new PointCloud());
  final_static_global_map_.reset(new PointCloud());

  submap_poses_->clear();
  final_static_global_map_->clear();

  readParameters();

  req_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/clicked_point", 1);
  static_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("static_map", 1);

  preprocess_scan_sub_ = nh_.subscribe<s2mat::Submap>("/preprocess_scan", 1, &MapMerger::staticScanCallback, this);
  static_submap_sub_ = nh_.subscribe<s2mat::Submap>("/static_submap", 1, &MapMerger::staticSubmapCallback, this);
  save_sub_ = nh_.subscribe<std_msgs::Empty>("/save_static_map", 1, &MapMerger::saveCallback, this);
}

MapMerger::~MapMerger()
{
}

void MapMerger::readParameters()
{
  nh_.param<std::string>("global_frame", global_frame_, "odom");
  nh_.param<std::string>("output_static_map_path", output_static_map_path_, "/home");

  nh_.param<int>("scan_frequency", scan_frequency_, 10);
  nh_.param<int>("lidar_lines", image_size_.first, 32);
  nh_.param<int>("image_width", image_size_.second, 360);

  nh_.param<float>("merge_dist", merge_dist_, 10.0);
  nh_.param<float>("lidar_vfov_upper", vfov_upper_, 22.5);
  nh_.param<float>("lidar_vfov_lower", vfov_lower_, -22.5);
  nh_.param<float>("lidar_hfov", hfov_, 360.0);
  nh_.param<float>("voxel_size", voxel_size_, 0.1);

  nh_.param<float>("local_radius", local_radius_, local_radius_);
  nh_.param<bool>("output_local", output_local_, output_local_);
}

void MapMerger::getGlobalScanPos(const PointType& scan_pos)
{
  std::unique_lock<std::mutex> scan_pos_lock(scan_pos_mutex_);
  scan_pos_ = scan_pos;
}

void MapMerger::staticScanCallback(const s2mat::Submap::ConstPtr& scan_msg)
{
  double start_time = ros::Time::now().toSec();
  scans_count_ += 1;

  // if (scans_count_ < scan_frequency_)
  if (scans_count_ < 2 * scan_frequency_)
  {
    return;
  }

  PointType scan_pos;
  scan_pos.x = scan_msg->pose.position.x;
  scan_pos.y = scan_msg->pose.position.y;
  scan_pos.z = scan_msg->pose.position.z;

  getGlobalScanPos(scan_pos);

  std::unique_lock<std::mutex> static_lock(submap_poses_mutex_);
  if (submap_poses_->points.size() < 1)
  {
    geometry_msgs::PointStamped point_msg;
    point_msg.point.x = scan_pos.x;
    point_msg.point.y = scan_pos.y;
    point_msg.point.z = scan_pos.z;
    req_point_pub_.publish(point_msg);
    curr_scans_num_ = 0;
  }
  else
  {
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_submap_poses(new pcl::KdTreeFLANN<PointType>());
    kdtree_submap_poses->setInputCloud(submap_poses_);

    std::vector<int> knn_submap_poses_idx;
    std::vector<float> knn_submap_poses_dist;
    kdtree_submap_poses->nearestKSearch(scan_pos, 1, knn_submap_poses_idx, knn_submap_poses_dist);

    if (sqrt(knn_submap_poses_dist[0]) > merge_dist_)
    {
      geometry_msgs::PointStamped point_msg;
      point_msg.point.x = scan_pos.x;
      point_msg.point.y = scan_pos.y;
      point_msg.point.z = scan_pos.z;
      req_point_pub_.publish(point_msg);
      curr_scans_num_ = 0;
    }
    else
    {
      curr_scans_num_++;
      // Handling special cases of loopback, such as a mobile robot swinging back and forth between two points
      if (curr_scans_num_ < scan_frequency_)
      {
        return;
      }

      if (curr_submap_id_ != knn_submap_poses_idx[0])
      {
        geometry_msgs::PointStamped point_msg;
        auto update_point = submap_poses_->points[knn_submap_poses_idx[0]];
        point_msg.point.x = update_point.x;
        point_msg.point.y = update_point.y;
        point_msg.point.z = update_point.z;
        req_point_pub_.publish(point_msg);
        curr_scans_num_ = 0;
      }
      else
      {
        if (curr_scans_num_ > (3 * scan_frequency_ - 1))
        {
          geometry_msgs::PointStamped point_msg;
          auto update_pos = submap_poses_->points[knn_submap_poses_idx[0]];
          point_msg.point.x = update_pos.x;
          point_msg.point.y = update_pos.y;
          point_msg.point.z = update_pos.z;
          req_point_pub_.publish(point_msg);
          curr_scans_num_ = 0;
        }
      }
    }
  }

  ROS_INFO_STREAM("[preprocess scan subscribe]: process time: " << ros::Time::now().toSec() - start_time);
}

void MapMerger::staticSubmapCallback(const s2mat::Submap::ConstPtr& submap_msg)
{
  double start_time = ros::Time::now().toSec();

  PointCloudPtr submap(new PointCloud());
  pcl::fromROSMsg(submap_msg->pointcloud, *submap);

  PointCloudPtr submap_local(new PointCloud());
  pcl::transformPointCloud(*submap, *submap_local, poseToMatrix(submap_msg->pose).inverse());

  const std::pair<int, int> image_size = { image_size_.first, image_size_.second };
  const float vfov_upper = vfov_upper_;
  const float vfov_lower = vfov_lower_;
  const float hfov = hfov_;
  cv::Mat submap_rimg = pointcloudToRimg(submap_local, image_size, vfov_upper, vfov_lower, hfov);

  PointType submap_pos;
  submap_pos.x = submap_msg->pose.position.x;
  submap_pos.y = submap_msg->pose.position.y;
  submap_pos.z = submap_msg->pose.position.z;

  std::unique_lock<std::mutex> static_lock(submap_poses_mutex_);
  if (submap_poses_->points.size() > 0)
  {
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_submap_poses(new pcl::KdTreeFLANN<PointType>());
    kdtree_submap_poses->setInputCloud(submap_poses_);

    std::vector<int> knn_submap_poses_idx;
    std::vector<float> knn_submap_poses_dist;
    kdtree_submap_poses->nearestKSearch(submap_pos, 1, knn_submap_poses_idx, knn_submap_poses_dist);

    if (sqrt(knn_submap_poses_dist[0]) > merge_dist_)
    {
      std::unique_lock<std::mutex> lock(submap_vectors_mutex_);
      submap_poses_->push_back(submap_pos);
      submap_pose_vector_.push_back(submap_msg->pose);
      submap_rimg_vector_.push_back(submap_rimg);
      curr_submap_id_ = submap_pose_vector_.size() - 1;
      submap_pose_pair_map_[curr_submap_id_] = { submap, submap_msg->pose };

      ROS_INFO_STREAM("[Merger]: Receive new static submap. The id of the new submap: " << curr_submap_id_);
    }
    else
    {
      std::unique_lock<std::mutex> lock(submap_vectors_mutex_);
      int update_id = knn_submap_poses_idx[0];
      submap_rimg_vector_[update_id] = submap_rimg;
      curr_submap_id_ = update_id;
      submap_pose_pair_map_[curr_submap_id_] = { submap, submap_pose_vector_[update_id] };

      ROS_INFO_STREAM("[Merger]: Update static submap. The id of the updated submap: " << curr_submap_id_);
    }
  }
  else
  {
    std::unique_lock<std::mutex> lock(submap_vectors_mutex_);
    submap_poses_->push_back(submap_pos);
    submap_pose_vector_.push_back(submap_msg->pose);
    submap_rimg_vector_.push_back(submap_rimg);
    curr_submap_id_ = 0;
    submap_pose_pair_map_[curr_submap_id_] = { submap, submap_msg->pose };

    ROS_INFO_STREAM("[Merger]: Receive new static submap. The id of the new submap: " << curr_submap_id_);
  }

  ROS_INFO_STREAM("[static submap subscribe]: process time: " << ros::Time::now().toSec() - start_time);
}

bool MapMerger::isMerge(std::vector<int>& submaps_id,
                        std::vector<std::unordered_set<struct Cell, hash_cell>>& occ_cells_vector,
                        std::vector<geometry_msgs::Pose>& poses, std::vector<cv::Mat>& ref_rimgs, PointType& scan_pos)
{
  std::unique_lock<std::mutex> lock(submap_vectors_mutex_);
  if (submap_pose_pair_map_.size() > 0)
  {
    double start_time = ros::Time::now().toSec();

    for (auto it = submap_pose_pair_map_.begin(); it != submap_pose_pair_map_.end(); ++it)
    {
      int submap_id = it->first;
      auto submap = it->second.first;
      auto pose = it->second.second;

      submaps_id.push_back(submap_id);

      std::unordered_set<struct Cell, hash_cell> occ_cells;
      for (int i = 0; i < submap->points.size(); ++i)
      {
        auto point = submap->points[i];
        auto cell = point_to_cell(point, voxel_size_);
        occ_cells.insert(cell);

        Eigen::Vector3f curr_pos(point.x, point.y, point.z);
        if (avg_pos_in_cells_.find(cell) == avg_pos_in_cells_.end())
        {
          RunningMean avg_pos = { curr_pos, 1 };
          avg_pos_in_cells_[cell] = avg_pos;
        }
        else
        {
          auto avg_pos = avg_pos_in_cells_[cell];
          avg_pos.first = (avg_pos.first * avg_pos.second + curr_pos) / (avg_pos.second + 1);
          avg_pos.second = avg_pos.second + 1;
          avg_pos_in_cells_[cell] = avg_pos;
        }
      }

      occ_cells_vector.push_back(occ_cells);
    }

    poses.insert(poses.end(), submap_pose_vector_.begin(), submap_pose_vector_.end());
    ref_rimgs.insert(ref_rimgs.end(), submap_rimg_vector_.begin(), submap_rimg_vector_.end());
    submap_pose_pair_map_.clear();

    std::unique_lock<std::mutex> scan_pos_lock(scan_pos_mutex_);
    scan_pos = scan_pos_;

    ROS_INFO_STREAM("[merge judgement]: process time: " << ros::Time::now().toSec() - start_time);

    return true;
  }
  else
  {
    return false;
  }
}

bool MapMerger::isObservedBySubmap(const PointType& point, const geometry_msgs::Pose& pose, const cv::Mat& ref_rimg)
{
  auto local_point = pcl::transformPoint(point, poseToMatrix(pose).inverse());

  const int row_size = image_size_.first;
  const int col_size = image_size_.second;
  const float vfov_upper = vfov_upper_;
  const float vfov_lower = vfov_lower_;
  const float hfov = hfov_;

  SphericalPoint sph_point = cartToSph(local_point);

  if (!isPointInVFOV(sph_point, vfov_upper, vfov_lower))
  {
    return false;
  }

  int lower_bound_row_idx = 0;
  int lower_bound_col_idx = 0;
  int upper_bound_row_idx = row_size - 1;
  int upper_bound_col_idx = col_size - 1;
  int pixel_idx_row = int(std::min(
      std::max(std::round((row_size - 1) * (1 - (radToDeg(sph_point.el) - vfov_lower) / (vfov_upper - vfov_lower))),
               float(lower_bound_row_idx)),
      float(upper_bound_row_idx)));
  int pixel_idx_col = int(std::min(
      std::max(std::round((col_size - 1) * ((radToDeg(sph_point.az) + (hfov / float(2.0))) / (hfov - float(0.0)))),
               float(lower_bound_col_idx)),
      float(upper_bound_col_idx)));

  float ref_range = ref_rimg.at<float>(pixel_idx_row, pixel_idx_col);

  if (ref_range == 10000.0 || sph_point.r > ref_range)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void MapMerger::incrementalMerge(std::vector<int>& submaps_id,
                                 std::vector<std::unordered_set<struct Cell, hash_cell>>& occ_cells_vector,
                                 std::vector<geometry_msgs::Pose>& poses, std::vector<cv::Mat>& ref_rimgs)
{
  double start_time = ros::Time::now().toSec();

  PointCloudPtr submap_poses(new PointCloud());
  for (auto pose : poses)
  {
    PointType point;
    point.x = pose.position.x;
    point.y = pose.position.y;
    point.z = pose.position.z;
    submap_poses->push_back(point);
  }

  ROS_INFO_STREAM("[Merger]: The size of merged submpas: " << submaps_id.size());

  pcl::KdTreeFLANN<PointType>::Ptr kdtree_submap_poses(new pcl::KdTreeFLANN<PointType>());
  kdtree_submap_poses->setInputCloud(submap_poses);

  std::unordered_set<int> submap_id_dict;
  for (int i = 0; i < submaps_id.size(); ++i)
  {
    submap_id_dict.insert(submaps_id[i]);
  }

  // remove dynamic points in the occupied map
  std::vector<struct Cell> erase_list;
  for (auto it = occupied_map_.begin(); it != occupied_map_.end(); ++it)
  {
    auto cell = it->first;
    auto submap_id = it->second;

    if (submap_id_dict.find(submap_id) != submap_id_dict.end())
    {
      erase_list.push_back(cell);
    }
    else
    {
      auto point = cell_to_point(cell, voxel_size_);
      for (auto submap_id : submaps_id)
      {
        auto pose = poses[submap_id];
        auto rimg = ref_rimgs[submap_id];
        if (isObservedBySubmap(point, pose, rimg))
        {
          float d = point_to_pose_distance(point, pose);
          if (distance_map_[cell] > d)
          {
            distance_map_[cell] = d;
            erase_list.push_back(cell);
          }
        }
      }
    }
  }

  for (int i = 0; i < erase_list.size(); ++i)
  {
    occupied_map_.erase(erase_list[i]);
  }

  ROS_INFO_STREAM("[remove dynamic points]: process time: " << ros::Time::now().toSec() - start_time);

  for (int i = 0; i < submaps_id.size(); ++i)
  {
    auto submap_id = submaps_id[i];
    auto occ_cells = occ_cells_vector[i];

    // update occupied map by new observations
    for (auto it = occ_cells.begin(); it != occ_cells.end(); ++it)
    {
      auto cell = *it;
      auto point = cell_to_point(cell, voxel_size_);
      float distance = point_to_pose_distance(point, poses[submap_id]);

      auto occ_it = distance_map_.find(cell);
      if (occ_it == distance_map_.end())
      {
        std::vector<int> knn_submap_poses_idx;
        std::vector<float> knn_submap_poses_dist;
        kdtree_submap_poses->nearestKSearch(point, 1, knn_submap_poses_idx, knn_submap_poses_dist);
        if (sqrt(knn_submap_poses_dist[0]) >= (distance - 1e-2))
        {
          occupied_map_[cell] = submap_id;
          distance_map_[cell] = distance;
        }
      }
      else if (distance - 1e-2 <= distance_map_[cell])
      {
        occupied_map_[cell] = submap_id;
        distance_map_[cell] = distance;
      }
    }
  }

  ROS_INFO_STREAM("[Merger]: merge process time: " << ros::Time::now().toSec() - start_time);
}

void MapMerger::publishStaticMap(const PointType& scan_pos)
{
  PointCloudPtr static_map(new PointCloud());
  std::vector<struct Cell> erase_list;
  
  for (auto it = occupied_map_.begin(); it != occupied_map_.end(); ++it)
  {
    auto cell = it->first;
    auto submap_id = it->second;

    PointType point;
    auto avg_pos = avg_pos_in_cells_[cell].first;
    point.x = avg_pos.x();
    point.y = avg_pos.y();
    point.z = avg_pos.z();

    if (output_local_ && pcl::euclideanDistance(point, scan_pos) > local_radius_)
    {
      erase_list.push_back(cell);
    }

    point.intensity = static_cast<float>(submap_id);
    static_map->push_back(point);
  }

  for (int i = 0; i < erase_list.size(); ++i)
  {
    struct Cell cell = erase_list[i];
    occupied_map_.erase(cell);
  }

  if (static_map->points.size() > 0)
  {
    publishMap(static_map_pub_, static_map, ros::Time::now(), global_frame_);

    final_static_global_map_->clear();
    *final_static_global_map_ = *static_map;
  }
}

void MapMerger::mergeThread()
{
  ros::Rate loop(5);
  while (ros::ok())
  {
    std::vector<int> submaps_id;
    std::vector<std::unordered_set<struct Cell, hash_cell>> occ_cells;
    std::vector<geometry_msgs::Pose> poses;
    std::vector<cv::Mat> ref_rimgs;
    PointType scan_pos;
    if (isMerge(submaps_id, occ_cells, poses, ref_rimgs, scan_pos))
    {
      incrementalMerge(submaps_id, occ_cells, poses, ref_rimgs);
      publishStaticMap(scan_pos);
    }
    loop.sleep();
  }
}

void MapMerger::run()
{
  std::thread merge_thread = std::thread(&MapMerger::mergeThread, this);
  ros::Rate loop(5);
  while (ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }
}

void MapMerger::saveCallback(const std_msgs::EmptyConstPtr& msg)
{
  ROS_INFO_STREAM("Saving static map...");
  if (final_static_global_map_->points.size() > 0)
  {
    std::string occ_map_file_name = output_static_map_path_ + "/static_map.pcd";
    ROS_INFO_STREAM("Saving static map to " << occ_map_file_name);
    pcl::io::savePCDFileBinary(occ_map_file_name, *final_static_global_map_);
  }
}

}  // namespace s2mat