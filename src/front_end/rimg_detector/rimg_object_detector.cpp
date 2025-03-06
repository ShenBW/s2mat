#include "front_end/rimg_detector/rimg_object_detector.h"

namespace s2mat
{
RimgObjectDetector::RimgObjectDetector(ros::NodeHandle nh) : nh_(nh)
{
  // cluster_.reset(new SLRCluster());
  cluster_.reset(new RimgCluster());

  detection_bbox_pub_ = nh_.advertise<BoundingBoxArray>("detection_bbox", 1);
  missing_bbox_pub_ = nh_.advertise<BoundingBoxArray>("missing_bbox", 1);
  proposal_bbox_pub_ = nh_.advertise<BoundingBoxArray>("proposal_bbox", 1);
}

RimgObjectDetector::~RimgObjectDetector()
{
}

bool RimgObjectDetector::isClusterStatic(const PointCloudPtr& cluster, pcl::CropBox<PointType>& static_map_crop_filter,
                                         BoundingBox box)
{
  PointCloudPtr box_pointcloud(new PointCloud());

  static_map_crop_filter.setMin(Eigen::Vector4f(box.pose.position.x - 0.5 * box.dimensions.x,
                                                box.pose.position.y - 0.5 * box.dimensions.y,
                                                box.pose.position.z - 0.5 * box.dimensions.z, 1.0));

  static_map_crop_filter.setMax(Eigen::Vector4f(box.pose.position.x + 0.5 * box.dimensions.x,
                                                box.pose.position.y + 0.5 * box.dimensions.y,
                                                box.pose.position.z + 0.5 * box.dimensions.z, 1.0));

  static_map_crop_filter.filter(*box_pointcloud);
  voxelFilter(cluster, voxel_size_);

  if (static_cast<float>(box_pointcloud->points.size()) / static_cast<float>(cluster->points.size()) < 0.2)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void RimgObjectDetector::filterCluster(const std::unordered_map<int, PointCloudPtr>& clusters_local,
                                       pcl::CropBox<PointType>& static_map_crop_filter,
                                       const geometry_msgs::Pose& frame_pose, BoundingBoxArray& bbox_msg)
{
  for (auto it = clusters_local.begin(); it != clusters_local.end(); ++it)
  {
    int label = it->first;
    auto cluster_local = it->second;

    PointCloudPtr cluster_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*cluster_local, *cluster_global, poseToMatrix(frame_pose));

    BoundingBox box;
    computePointcloudBbox(cluster_global, box);

    if (box.dimensions.x < 0.20 || box.dimensions.x > 10.0 || box.dimensions.y < 0.20 || box.dimensions.y > 10.0 ||
        box.dimensions.z < 0.50 || box.dimensions.z > 5.0)
    {
      continue;
    }

    if (!isClusterStatic(cluster_global, static_map_crop_filter, box))
    {
      box.header.frame_id = bbox_msg.header.frame_id;
      box.header.stamp = bbox_msg.header.stamp;
      box.label = label;
      bbox_msg.boxes.push_back(box);
    }
  }
}

// For large objects
bool RimgObjectDetector::calculateBboxPointcloud(const PointCloudPtr& scan, const BoundingBox& prediction_box,
                                                 pcl::CropBox<PointType>& crop_filter, const PointCloudPtr& cluster,
                                                 std::vector<int>& idx_inside, int max_iterations)
{
  crop_filter.setInputCloud(scan);

  Eigen::Vector4f dimensions_min =
      Eigen::Vector4f(-0.5 * prediction_box.dimensions.x - 0.1, -0.5 * prediction_box.dimensions.y - 0.1,
                      -0.5 * prediction_box.dimensions.z, 1.0);

  Eigen::Vector4f dimensions_max =
      Eigen::Vector4f(0.5 * prediction_box.dimensions.x + 0.1, 0.5 * prediction_box.dimensions.y + 0.1,
                      0.5 * prediction_box.dimensions.z, 1.0);

  for (int i = 0; i < max_iterations; ++i)
  {
    cluster->clear();
    idx_inside.clear();

    crop_filter.setMin(dimensions_min);
    crop_filter.setMax(dimensions_max);
    crop_filter.setNegative(false);

    crop_filter.filter(*cluster);
    crop_filter.filter(idx_inside);

    if (cluster->points.size() < 5)
    {
      return false;
    }

    BoundingBox box;
    computePointcloudBbox(cluster, box);

    if ((box.dimensions.x * box.dimensions.y) / (prediction_box.dimensions.x * prediction_box.dimensions.y) > 0.50)
    {
      return true;
    }

    dimensions_min += Eigen::Vector4f(std::min(0.0, box.pose.position.x), std::min(0.0, box.pose.position.y), 0.0, 0.0);

    dimensions_max += Eigen::Vector4f(std::max(0.0, box.pose.position.x), std::max(0.0, box.pose.position.y), 0.0, 0.0);
  }

  return false;
}

void RimgObjectDetector::generateBboxFromPredictions(const PointCloudPtr& scan_local,
                                                     pcl::CropBox<PointType>& static_map_crop_filter,
                                                     const geometry_msgs::Pose& frame_pose,
                                                     const BoundingBoxArray& prediction_msg,
                                                     BoundingBoxArray& pred_bbox_msg,
                                                     std::unordered_map<int, std::vector<int>>& clusters_idx)
{
  Eigen::Affine3f trans_to_global = poseToMatrix(frame_pose);

  PointCloudPtr scan_global(new PointCloud());
  pcl::transformPointCloud(*scan_local, *scan_global, trans_to_global);

  pcl::CropBox<PointType> crop_filter;

  for (auto box : prediction_msg.boxes)
  {
    PointCloudPtr scan_box(new PointCloud());
    auto pose = box.pose;
    pcl::transformPointCloud(*scan_global, *scan_box, poseToMatrix(pose).inverse());

    PointCloudPtr cluster_box(new PointCloud());
    std::vector<int> idx_inside;

    if (calculateBboxPointcloud(scan_box, box, crop_filter, cluster_box, idx_inside, 2))
    {
      PointCloudPtr cluster_global(new PointCloud());

      pcl::transformPointCloud(*cluster_box, *cluster_global, poseToMatrix(pose));

      BoundingBox box_global;
      computePointcloudBbox(cluster_global, box_global);

      if (!isClusterStatic(cluster_global, static_map_crop_filter, box_global))
      {
        box_global.header.frame_id = pred_bbox_msg.header.frame_id;
        box_global.header.stamp = pred_bbox_msg.header.stamp;
        box_global.label = box.label + 10000;
        pred_bbox_msg.boxes.push_back(box_global);

        clusters_idx[box.label + 10000] = idx_inside;
      }
    }
  }
}

void RimgObjectDetector::bboxMatching(const BoundingBoxArray& bbox_msg_1, const BoundingBoxArray& bbox_msg_2,
                                      BoundingBoxArray& output_bbox_msg, bool output_matching)
{
  std::vector<double> temp;

  intptr_t nr = static_cast<intptr_t>(bbox_msg_1.boxes.size());
  intptr_t nc = static_cast<intptr_t>(bbox_msg_2.boxes.size());

  if (nr < 1 || nc < 1)
  {
    return;
  }

  temp.resize(nr * nc);

  // compute affinity matrix
  for (int i = 0; i < bbox_msg_1.boxes.size(); ++i)
  {
    for (int j = 0; j < bbox_msg_2.boxes.size(); ++j)
    {
      double iou = computeBboxIOU(bbox_msg_1.boxes[i], bbox_msg_2.boxes[j]);
      temp[i * nc + j] = -iou;
    }
  }

  // std::cout << "nr : " << nr << " | nc: " << nc << std::endl;

  double* cost_matrix = temp.data();

  intptr_t max_matching = nr < nc ? nr : nc;

  int64_t a[max_matching];
  int64_t b[max_matching];
  int res = solve_rectangular_linear_sum_assignment(nr, nc, cost_matrix, false, a, b);

  int effective_matching;
  effective_matching = sizeof(a) / sizeof(int64_t);
  std::unordered_set<int> matching_set;
  for (int i = 0; i < effective_matching; ++i)
  {
    if (temp[a[i] * nc + b[i]] != 0)
    {
      matching_set.insert(b[i]);
      if (output_matching)
      {
        output_bbox_msg.boxes.push_back(bbox_msg_1.boxes[a[i]]);
      }
    }
  }

  if (!output_matching)
  {
    for (int i = 0; i < bbox_msg_2.boxes.size(); ++i)
    {
      if (matching_set.find(i) == matching_set.end())
      {
        output_bbox_msg.boxes.push_back(bbox_msg_2.boxes[i]);
      }
    }
  }
}

void RimgObjectDetector::mergeBbox(BoundingBoxArray& src_bbox_msg, const BoundingBoxArray& dst_bbox_msg)
{
  for (auto box : dst_bbox_msg.boxes)
  {
    src_bbox_msg.boxes.push_back(box);
  }
}

void RimgObjectDetector::detect(const PointCloudPtr& static_map, const PointCloudPtr& scan_local,
                                const std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair,
                                const geometry_msgs::Pose& frame_pose, BoundingBoxArray prediction_msg,
                                BoundingBoxArray& bbox_msg, std::unordered_map<int, std::vector<int>>& clusters_idx,
                                std::unordered_map<int, std::vector<struct Pixel>>& clusters_pixels)
{
  // prediction_msg.header.stamp = bbox_msg.header.stamp;
  // prediction_msg.header.frame_id = bbox_msg.header.frame_id;

  cluster_->removeGround(scan_local);

  std::unordered_map<int, PointCloudPtr> clusters_local;

  cv::Mat label_image = cv::Mat::zeros(cluster_->lidar_lines_, cluster_->lidar_hresolution_, CV_32SC1);

  cluster_->clusterPointcloud(scan_local, scan_rimgs_pair, label_image, clusters_local, clusters_idx, clusters_pixels);

  pcl::CropBox<PointType> static_map_crop_filter;
  static_map_crop_filter.setInputCloud(static_map);

  filterCluster(clusters_local, static_map_crop_filter, frame_pose, bbox_msg);

  detection_bbox_pub_.publish(bbox_msg);

  BoundingBoxArray pred_bbox_msg;
  pred_bbox_msg.header.stamp = bbox_msg.header.stamp;
  pred_bbox_msg.header.frame_id = bbox_msg.header.frame_id;

  generateBboxFromPredictions(scan_local, static_map_crop_filter, frame_pose, prediction_msg, pred_bbox_msg,
                              clusters_idx);

  BoundingBoxArray miss_bbox_msg;
  miss_bbox_msg.header.stamp = bbox_msg.header.stamp;
  miss_bbox_msg.header.frame_id = bbox_msg.header.frame_id;

  if (bbox_msg.boxes.size() > 0)
  {
    bboxMatching(bbox_msg, pred_bbox_msg, miss_bbox_msg, false);
  }
  else
  {
    for (auto box : pred_bbox_msg.boxes)
    {
      miss_bbox_msg.boxes.push_back(box);
    }
  }

  missing_bbox_pub_.publish(miss_bbox_msg);

  mergeBbox(bbox_msg, miss_bbox_msg);

  proposal_bbox_pub_.publish(bbox_msg);
}

void RimgObjectDetector::detectNoPredictions(const PointCloudPtr& static_map, const PointCloudPtr& scan_local,
                                             const std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair,
                                             const geometry_msgs::Pose& frame_pose, BoundingBoxArray& bbox_msg,
                                             std::unordered_map<int, std::vector<int>>& clusters_idx,
                                             std::unordered_map<int, std::vector<struct Pixel>>& clusters_pixels)
{
  double start_time = ros::Time::now().toSec();

  cluster_->removeGround(scan_local);

  std::unordered_map<int, PointCloudPtr> clusters_local;

  cv::Mat label_image = cv::Mat::zeros(cluster_->lidar_lines_, cluster_->lidar_hresolution_, CV_32SC1);

  cluster_->clusterPointcloud(scan_local, scan_rimgs_pair, label_image, clusters_local, clusters_idx, clusters_pixels);

  pcl::CropBox<PointType> static_map_crop_filter;
  static_map_crop_filter.setInputCloud(static_map);

  filterCluster(clusters_local, static_map_crop_filter, frame_pose, bbox_msg);

  proposal_bbox_pub_.publish(bbox_msg);
}
}  // namespace s2mat