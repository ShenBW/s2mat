#include "front_end/rimg_front_end.h"

namespace smat
{
RimgFrontEnd::RimgFrontEnd(ros::NodeHandle nh)
  : nh_(nh)
  , img_transport_(nh_)
  , last_track_id_(0)
  , init_map_(false)
  , update_flag_(false)
{
  map_global_.reset(new PointCloud());
  map_global_->clear();

  preprocessor_.reset(new ScanPreprocessor());
  object_detector_.reset(new RimgObjectDetector(nh_));
  tracker_.reset(new nnt::NNTracker(nh_));
  tracks_processor_.reset(new TracksProcessor());

  readParameters();

  compare_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("compare_map", 1);
  static_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("static_scan", 1);
  preprocess_scan_pub_ = nh_.advertise<smat::Submap>("/preprocess_scan", 1);
  tracking_pub_ = nh_.advertise<BoundingBoxArray>("tracking", 1);
  traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectories", 1);

  scan_rimg_pub_ = img_transport_.advertise("scan_rimg", 10);

  init_map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/init_map", 1, &RimgFrontEnd::initMapCallback, this);
  map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/static_map", 1, &RimgFrontEnd::mapCallback, this);
}

RimgFrontEnd::~RimgFrontEnd()
{
}

bool RimgFrontEnd::readParameters()
{
  nh_.param<std::string>("local_frame", local_frame_, "base_link");
  nh_.param<std::string>("global_frame", global_frame_, "odom");
  nh_.param<int>("omp_cores", omp_cores_, 4);
  nh_.param<int>("scan_frequency", scan_frequency_, 10);
  max_num_ = std::round(2 * scan_frequency_);

  nh_.param<float>("max_depth", max_depth_, 25.0);
  nh_.param<float>("voxel_size", voxel_size_, 0.1);

  preprocessor_->omp_cores_ = omp_cores_;
  nh_.param<int>("lidar_type", preprocessor_->lidar_type_, 1);
  nh_.param<int>("lidar_lines", preprocessor_->lidar_lines_, 32);
  nh_.param<int>("lidar_hresolution", preprocessor_->lidar_hresolution_, 1024);
  nh_.param<float>("blind", preprocessor_->blind_, 0.01);
  nh_.param<float>("lidar_vfov_upper", preprocessor_->vfov_upper_, 22.5);
  nh_.param<float>("lidar_vfov_lower", preprocessor_->vfov_lower_, -22.5);
  nh_.param<float>("lidar_hfov", preprocessor_->hfov_, 360.0);
  nh_.param<bool>("is_ring_available", preprocessor_->is_ring_available_, false);

  object_detector_->voxel_size_ = voxel_size_;

  object_detector_->cluster_->omp_cores_ = omp_cores_;
  object_detector_->cluster_->lidar_lines_ = preprocessor_->lidar_lines_;
  object_detector_->cluster_->lidar_hresolution_ = preprocessor_->lidar_hresolution_;
  object_detector_->cluster_->hfov_ = preprocessor_->hfov_;
  object_detector_->cluster_->max_depth_ = max_depth_;
  nh_.param<int>("cluster_size", object_detector_->cluster_->cluster_size_, 50);
  nh_.param<float>("sensor_height", object_detector_->cluster_->sensor_height_, 0.5);
  nh_.param<float>("lidar_vfov", object_detector_->cluster_->vfov_, 45.0);
  nh_.param<bool>("consider_max_depth", object_detector_->cluster_->consider_max_depth_,
                  object_detector_->cluster_->consider_max_depth_);

  return true;
}

bool RimgFrontEnd::getTransformPose(const ros::Time& stamp, const std::string& target_frame,
                                    const std::string& source_frame, geometry_msgs::Pose& pose)
{
  tf::StampedTransform transform_msg;
  try
  {
    tf_listener_.lookupTransform(target_frame, source_frame, stamp, transform_msg);
  }

  catch (tf::TransformException ex)
  {
    ROS_WARN_STREAM(ex.what());
    return false;
  }

  pose.position.x = transform_msg.getOrigin().x();
  pose.position.y = transform_msg.getOrigin().y();
  pose.position.z = transform_msg.getOrigin().z();

  pose.orientation.x = transform_msg.getRotation().x();
  pose.orientation.y = transform_msg.getRotation().y();
  pose.orientation.z = transform_msg.getRotation().z();
  pose.orientation.w = transform_msg.getRotation().w();

  return true;
}

void RimgFrontEnd::cropMapGlobal(const PointCloudPtr& map_crop, const geometry_msgs::Pose& frame_pose)
{
  std::unique_lock<std::mutex> lock(map_global_lock_);
  Eigen::Affine3f trans_to_global = poseToMatrix(frame_pose);
  Eigen::Affine3f trans_to_frame = trans_to_global.inverse();

  PointCloudPtr map_local(new PointCloud());
  PointCloudPtr map_local_crop(new PointCloud());
  map_local->clear();
  pcl::transformPointCloud(*map_global_, *map_local, trans_to_frame);

  int point_size = map_local->points.size();

#pragma omp parallel for num_threads(omp_cores_)
  for (int i = 0; i < point_size; i++)
  {
    auto point = map_local->points[i];

    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
    {
      continue;
    }

    float depth = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (depth < max_depth_)
    {
      map_local_crop->push_back(point);
    }
  }

  pcl::transformPointCloud(*map_local_crop, *map_crop, trans_to_global);
}

void RimgFrontEnd::scansideRemove(const PointCloudPtr& all_scan, const PointCloudPtr& static_scan,
                                  const TrackedObjects& tracked_objects,
                                  std::unordered_map<int, std::vector<int>>& clusters_idx)
{
  pcl::ExtractIndices<PointType> extractor;
  extractor.setInputCloud(all_scan);

  PointCloudPtr dynamic_scan(new PointCloud());
  dynamic_scan->clear();
  std::vector<int> dynamic_idx;

  std::unique_lock<std::mutex> lock(track_id_lock_);
  for (auto track : tracked_objects.tracks)
  {
    if (track.is_matched)
    {
      int label = static_cast<int>(track.detection_id);
      auto it = clusters_idx.find(label);
      if (it != clusters_idx.end())
      {
        PointCloudPtr dynamic_points(new PointCloud());

        auto idx = it->second;

        boost::shared_ptr<std::vector<int>> idx_ptr = boost::make_shared<std::vector<int>>(idx);
        extractor.setIndices(idx_ptr);
        extractor.setNegative(false);
        dynamic_points->clear();
        extractor.filter(*dynamic_points);
        setIntensity(dynamic_points, static_cast<float>(track.track_id) + 1.0);
        *dynamic_scan += *dynamic_points;

        dynamic_idx.insert(dynamic_idx.end(), idx.begin(), idx.end());
      }
      else
      {
        ROS_WARN_STREAM("Can not find label " << label);
      }
    }
  }

  if (dynamic_idx.size() > 0)
  {
    boost::shared_ptr<std::vector<int>> idx_ptr = boost::make_shared<std::vector<int>>(dynamic_idx);
    extractor.setIndices(idx_ptr);
    // If set to true, you can extract point clouds outside the specified index
    extractor.setNegative(true);

    static_scan->clear();
    extractor.filter(*static_scan);
    PointCloudPtr static_scan_track_id(new PointCloud());
    *static_scan_track_id = *static_scan;

    setIntensity(static_scan_track_id, 0.0);
    all_scan->clear();
    *all_scan += *static_scan_track_id;
    *all_scan += *dynamic_scan;
  }
  else
  {
    *static_scan = *all_scan;
    setIntensity(all_scan, 0.0);
  }
}

void RimgFrontEnd::scansideRemoveByCrop(const PointCloudPtr& all_scan, const PointCloudPtr& static_scan,
                                        const TrackedObjects& tracked_objects,
                                        std::unordered_map<int, std::vector<int>>& clusters_idx)
{
  pcl::ExtractIndices<PointType> extractor;
  extractor.setInputCloud(all_scan);

  PointCloudPtr dynamic_scan(new PointCloud());

  std::unique_lock<std::mutex> lock(track_id_lock_);
  BoundingBoxArray dynamic_bboxes;
  for (auto track : tracked_objects.tracks)
  {
    if (track.is_matched)
    {
      int label = static_cast<int>(track.detection_id);
      auto it = clusters_idx.find(label);
      if (it != clusters_idx.end())
      {
        PointCloudPtr dynamic_points(new PointCloud());

        auto idx = it->second;

        boost::shared_ptr<std::vector<int>> idx_ptr = boost::make_shared<std::vector<int>>(idx);
        extractor.setIndices(idx_ptr);
        extractor.setNegative(false);
        dynamic_points->clear();
        extractor.filter(*dynamic_points);
        setIntensity(dynamic_points, static_cast<float>(track.track_id) + 1.0);
        *dynamic_scan += *dynamic_points;

        BoundingBox dynamic_bbox;
        computePointcloudBbox(dynamic_points, dynamic_bbox);

        dynamic_bboxes.boxes.push_back(dynamic_bbox);
      }
      else
      {
        ROS_WARN_STREAM("Can not find label " << label);
      }
    }
  }

  if (dynamic_bboxes.boxes.size() > 0)
  {
    pcl::CropBox<PointType> crop_filter;
    *static_scan = *all_scan;

    for (auto dynamic_bbox : dynamic_bboxes.boxes)
    {
      crop_filter.setInputCloud(static_scan);
      crop_filter.setMin(Eigen::Vector4f(dynamic_bbox.pose.position.x - 0.5 * dynamic_bbox.dimensions.x,
                                         dynamic_bbox.pose.position.y - 0.5 * dynamic_bbox.dimensions.y,
                                         dynamic_bbox.pose.position.z - 0.5 * dynamic_bbox.dimensions.z, 1.0));
      crop_filter.setMax(Eigen::Vector4f(dynamic_bbox.pose.position.x + 0.5 * dynamic_bbox.dimensions.x,
                                         dynamic_bbox.pose.position.y + 0.5 * dynamic_bbox.dimensions.y,
                                         dynamic_bbox.pose.position.z + 0.5 * dynamic_bbox.dimensions.z, 1.0));
      crop_filter.setNegative(true);
      crop_filter.filter(*static_scan);
    }

    PointCloudPtr static_scan_track_id(new PointCloud());
    *static_scan_track_id = *static_scan;

    setIntensity(static_scan_track_id, 0.0);
    all_scan->clear();
    *all_scan += *static_scan_track_id;
    *all_scan += *dynamic_scan;
  }
  else
  {
    *static_scan = *all_scan;
    setIntensity(all_scan, 0.0);
  }
}

void RimgFrontEnd::publishPreprocessScan(const PointCloudPtr& preprocess_scan, const geometry_msgs::Pose pose)
{
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*preprocess_scan, pointcloud_msg);

  smat::Submap scan_msg;
  scan_msg.pointcloud = pointcloud_msg;
  scan_msg.pose = pose;
  preprocess_scan_pub_.publish(scan_msg);
}

double RimgFrontEnd::calculateDeltaTime()
{
  return 1.0 / double(scan_frequency_);
}

void RimgFrontEnd::publishTrajectories(const BoundingBoxArray& tracking_msg)
{
  visualization_msgs::MarkerArray markers;
  for (auto it = traj_dict_.begin(); it != traj_dict_.end(); ++it)
  {
    int track_id = it->first;
    auto poses = it->second;

    visualization_msgs::Marker marker;
    marker.header = tracking_msg.header;

    marker.action = marker.ADD;
    marker.type = marker.LINE_STRIP;
    marker.ns = "traj";

    marker.scale.x = 0.1;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;

    marker.pose.orientation.w = 1.0;

    int color_id = track_id % color_map_.size();
    marker.color.r = color_map_[color_id][0];
    marker.color.g = color_map_[color_id][1];
    marker.color.b = color_map_[color_id][2];
    marker.color.a = 0.5;

    if (poses.size() < 3)
    {
      continue;
    }

    for (auto pose : poses)
    {
      geometry_msgs::Point point;
      point.x = pose.position.x;
      point.y = pose.position.y;
      point.z = pose.position.z;

      marker.points.push_back(point);
    }

    marker.id = track_id;
    markers.markers.push_back(marker);
  }

  traj_pub_.publish(markers);
}

void RimgFrontEnd::publishTracking(const PointCloudPtr& scan, std::unordered_map<int, std::vector<int>>& clusters_idx,
                                   const TrackedObjects& tracked_msg)
{
  pcl::ExtractIndices<PointType> extractor;
  extractor.setInputCloud(scan);

  std::unique_lock<std::mutex> lock(track_id_lock_);
  BoundingBoxArray tracking_msg;

  tracking_msg.header = tracked_msg.header;

  prediction_msg_.boxes.clear();

  double delta_time = calculateDeltaTime();
  for (auto track : tracked_msg.tracks)
  {
    if (track.is_matched)
    {
      if (success_track_id_.find(track.track_id + 1) != success_track_id_.end())
      {
        int detection_id = static_cast<int>(track.detection_id);
        auto it = clusters_idx.find(detection_id);
        if (it != clusters_idx.end())
        {
          int track_id;
          if (track_id_map_.find(track.track_id) != track_id_map_.end())
          {
            track_id = track_id_map_[track.track_id];
          }
          else
          {
            track_id = last_track_id_;
            track_id_map_[track.track_id] = track_id;
            traj_dict_[track_id] = traj_candidate_dict_[track.track_id];
            last_track_id_++;
          }

          auto idx = it->second;
          Eigen::Vector3f extent = Eigen::Vector3f::Zero();
          tracks_processor_->calculateTrackDimension(extractor, idx, track, extent);

          BoundingBox tracking_box;
          BoundingBox pred_box;

          tracking_box.header.stamp = tracked_msg.header.stamp;

          tracking_box.header.frame_id = global_frame_;
          tracking_box.pose = track.pose.pose;

          tracking_box.dimensions.x = extent.x();
          tracking_box.dimensions.y = extent.y();
          tracking_box.dimensions.z = extent.z();
          tracking_box.label = static_cast<unsigned int>(track_id);

          tracks_processor_->predictBboxFromTracking(track, pred_box, delta_time);
          pred_box.header.frame_id = global_frame_;
          pred_box.dimensions.x = extent.x();
          pred_box.dimensions.y = extent.y();
          pred_box.dimensions.z = extent.z();
          pred_box.label = static_cast<unsigned int>(track_id);

          tracking_msg.boxes.push_back(tracking_box);
          prediction_msg_.boxes.push_back(pred_box);

          traj_dict_[track_id].push_back(track.pose.pose);
        }
      }
      else
      {
        traj_candidate_dict_[track.track_id].push_back(track.pose.pose);
      }
    }
  }

  tracking_pub_.publish(tracking_msg);

  publishTrajectories(tracking_msg);
}

void RimgFrontEnd::scanCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg)
{
  ros::Time stamp = pointcloud_msg->header.stamp;

  geometry_msgs::Pose frame_pose;
  if (!getTransformPose(stamp, global_frame_, local_frame_, frame_pose))
  {
    return;
  }

  double start_time = ros::Time::now().toSec();

  PointCloudPtr scan_local(new PointCloud());
  PointCloudPtr scan_global(new PointCloud());
  std::pair<cv::Mat, cv::Mat> scan_rimgs_pair;

  preprocessor_->process(pointcloud_msg, scan_rimgs_pair, scan_local);

  publishRangeImg(scan_rimg_pub_, scan_rimgs_pair.first, stamp, { 0, 255 });

  pcl::transformPointCloud(*scan_local, *scan_global, poseToMatrix(frame_pose));

  ROS_INFO_STREAM("[preprocess]: process time: " << ros::Time::now().toSec() - start_time);

  PointCloudPtr map_crop(new PointCloud());
  // map_crop->clear();
  cropMapGlobal(map_crop, frame_pose);

  publishMap(compare_map_pub_, map_crop, stamp, global_frame_);

  ROS_INFO_STREAM("[crop map]: process time: " << ros::Time::now().toSec() - start_time);

  BoundingBoxArray bbox_msg;
  bbox_msg.header.frame_id = global_frame_;
  bbox_msg.header.stamp = stamp;

  std::unordered_map<int, std::vector<int>> clusters_idx;
  std::unordered_map<int, std::vector<struct Pixel>> clusters_pixels;

  object_detector_->detect(map_crop, scan_local, scan_rimgs_pair, frame_pose, prediction_msg_, bbox_msg, clusters_idx,
                           clusters_pixels);

  ROS_INFO_STREAM("[object detector]: process time: " << ros::Time::now().toSec() - start_time);

  TrackedObjects tracked_objects = tracker_->track(bbox_msg);

  PointCloudPtr static_scan(new PointCloud());
  PointCloudPtr all_scan(new PointCloud());
  *all_scan = *scan_global;

  scansideRemove(all_scan, static_scan, tracked_objects, clusters_idx);
  // scansideRemoveByCrop(all_scan, static_scan, tracked_objects, clusters_idx, clusters_pixels);

  publishScan(static_scan_pub_, static_scan, stamp, global_frame_);

  publishPreprocessScan(static_scan, frame_pose);

  publishTracking(scan_global, clusters_idx, tracked_objects);

  ROS_INFO_STREAM("[tracking]: process time: " << ros::Time::now().toSec() - start_time);

  std::unique_lock<std::mutex> deque_lock(deque_lock_);

  scan_deque_.push_back(all_scan);
  pose_deque_.push_back(frame_pose);
  object_deque_.push_back(tracked_objects);

  std::unique_lock<std::mutex> update_lock(update_lock_);
  update_flag_ = true;
}

void RimgFrontEnd::subscribePointcloud()
{
  scan_sub_.subscribe(nh_, "/points", 1);
  scan_filter_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(scan_sub_, tf_listener_, global_frame_, 10));

  scan_filter_->registerCallback(boost::bind(&RimgFrontEnd::scanCallback, this, _1));
}

void RimgFrontEnd::initMapCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg)
{
  if (!init_map_)
  {
    std::unique_lock<std::mutex> lock(map_global_lock_);
    PointCloudPtr raw_pointcloud(new PointCloud());
    pcl::fromROSMsg(*pointcloud_msg, *raw_pointcloud);
    map_global_->clear();
    *map_global_ = *raw_pointcloud;
  }
}

void RimgFrontEnd::mapCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg)
{
  std::unique_lock<std::mutex> lock(map_global_lock_);
  PointCloudPtr raw_pointcloud(new PointCloud());
  pcl::fromROSMsg(*pointcloud_msg, *raw_pointcloud);
  map_global_->clear();
  *map_global_ = *raw_pointcloud;
  init_map_ = true;
}

bool RimgFrontEnd::isUpdate()
{
  std::unique_lock<std::mutex> lock(update_lock_);
  if (update_flag_)
  {
    update_flag_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

void RimgFrontEnd::getTemporalData(std::vector<PointCloudPtr>& temporal_scans,
                                   std::vector<geometry_msgs::Pose>& temporal_scans_pose,
                                   std::vector<TrackedObjects>& temporal_scans_object)
{
  std::unique_lock<std::mutex> deque_lock(deque_lock_);
  int end_idx = scan_deque_.size();
  int start_idx = end_idx - max_num_ > 0 ? end_idx - max_num_ : 0;

  for (int i = start_idx; i < end_idx; ++i)
  {
    temporal_scans.push_back(scan_deque_[i]);
    temporal_scans_pose.push_back(pose_deque_[i]);
    temporal_scans_object.push_back(object_deque_[i]);
  }

  while (scan_deque_.size() > 200)
  {
    scan_deque_.pop_front();
    pose_deque_.pop_front();
    object_deque_.pop_front();
  }
}

void RimgFrontEnd::determineTracking()
{
  double start_time = ros::Time::now().toSec();

  std::vector<PointCloudPtr> scans;
  std::vector<geometry_msgs::Pose> scans_pose;
  std::vector<TrackedObjects> scans_object;

  getTemporalData(scans, scans_pose, scans_object);

  int scans_size = scans.size();
  if (scans_size == 0)
  {
    return;
  }

  std::unordered_set<int> success_tracking_traj;
  tracks_processor_->measureTracks(scans, scans_pose, scans_object, success_tracking_traj);

  std::unique_lock<std::mutex> lock(track_id_lock_);
  success_track_id_.swap(success_tracking_traj);

  ROS_INFO_STREAM("[Front End]: detection by tracking process time: " << ros::Time::now().toSec() - start_time);
}

void RimgFrontEnd::submapThread()
{
  ros::Rate loop(2);
  while (ros::ok())
  {
    if (isUpdate())
    {
      determineTracking();
    }
    loop.sleep();
  }
}

void RimgFrontEnd::run()
{
  ros::Rate r(10);
  std::thread submap_thread = std::thread(&RimgFrontEnd::submapThread, this);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}
}  // namespace smat