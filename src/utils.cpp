#include "utils.h"

ssize_t py_div(float a, float b)
{
  ssize_t q = a / b;
  float r = fmod(a, b);
  if ((r != 0) && ((r < 0) != (b < 0)))
  {
    q -= 1;
  }
  return q;
}

struct Cell point_to_cell(const PointType& p, float voxel_size)
{
  return Cell(py_div(p.x, voxel_size), py_div(p.y, voxel_size), py_div(p.z, voxel_size));
}

PointType cell_to_point(const struct Cell& cell, float voxel_size)
{
  PointType point;
  point.x = cell.x * voxel_size;
  point.y = cell.y * voxel_size;
  point.z = cell.z * voxel_size;
  return point;
}

void pointcloud_to_cellmap(const PointCloudPtr& pointcloud, std::unordered_set<struct Cell, hash_cell>& cell_map,
                           float voxel_size)
{
#pragma omp parallel for num_threads(4)
  for (int i = 0; i < pointcloud->points.size(); ++i)
  {
    auto point = pointcloud->points[i];
    auto cell = point_to_cell(point, voxel_size);
    cell_map.insert(cell);
  }
}

Eigen::Affine3f poseToMatrix(const geometry_msgs::Pose& pose)
{
  Eigen::Translation3f trans(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaternionf quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Affine3f tf_matrix = trans * quat.toRotationMatrix();

  return tf_matrix;
}

void publishScan(ros::Publisher pub, const PointCloudPtr& scan, const ros::Time& stamp, std::string frame_id)
{
  sensor_msgs::PointCloud2 pub_pointcloud;

  pcl::toROSMsg(*scan, pub_pointcloud);
  pub_pointcloud.header.frame_id = frame_id;
  pub_pointcloud.header.stamp = stamp;

  pub.publish(pub_pointcloud);
};

void publishMap(ros::Publisher pub, const PointCloudPtr& map, const ros::Time& stamp, std::string frame_id)
{
  sensor_msgs::PointCloud2 pub_pointcloud;

  pcl::toROSMsg(*map, pub_pointcloud);
  pub_pointcloud.header.frame_id = frame_id;
  pub_pointcloud.header.stamp = stamp;

  pub.publish(pub_pointcloud);
};

cv::Mat convertColorMappedImg(const cv::Mat& src, std::pair<float, float> color_axis)
{
  float min_color_val = color_axis.first;
  float max_color_val = color_axis.second;

  cv::Mat dst;
  dst = 255 * (src - min_color_val) / (max_color_val - min_color_val);
  dst.convertTo(dst, CV_8UC1);

  cv::applyColorMap(dst, dst, cv::COLORMAP_JET);
  return dst;
};

void publishRangeImg(const image_transport::Publisher& pub, const cv::Mat& rimg, const ros::Time& stamp,
                     std::pair<float, float> color_axis)
{
  cv::Mat rimg_viz = convertColorMappedImg(rimg, color_axis);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rimg_viz).toImageMsg();
  msg->header.stamp = stamp;
  pub.publish(msg);
}

float point_to_pose_distance(const PointType& point, const geometry_msgs::Pose& pose)
{
  float diff_x = point.x - pose.position.x;
  float diff_y = point.y - pose.position.y;
  float diff_z = point.z - pose.position.z;
  return std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
}

float radToDeg(float rad)
{
  return rad * 180.0 / M_PI;
}

SphericalPoint cartToSph(const PointType& point_cor)
{
  SphericalPoint sph_point{ std::atan2(point_cor.y, point_cor.x),
                            std::atan2(point_cor.z, std::sqrt(point_cor.x * point_cor.x + point_cor.y * point_cor.y)),
                            std::sqrt(point_cor.x * point_cor.x + point_cor.y * point_cor.y +
                                      point_cor.z * point_cor.z) };

  return sph_point;
}

double point_to_range(const PointType& point)
{
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

std::pair<int, int> resetRimgSize(std::pair<float, float> FOV, float alpha_res)
{
  // default is 1 deg x 1 deg
  float alpha_vfov = alpha_res;
  float alpha_hfov = alpha_res;

  float vfov = FOV.first;
  float hfov = FOV.second;

  int row_size = std::round(vfov * alpha_vfov);
  int col_size = std::round(hfov * alpha_hfov);

  std::pair<int, int> rimg_shape{ row_size, col_size };

  return rimg_shape;
}

void setIntensity(const PointCloudPtr& pc, float intensity)
{
#pragma omp parallel for num_threads(4)
  for (int i = 0; i < pc->points.size(); ++i)
  {
    pc->points[i].intensity = intensity;
  }
}

void voxelFilter(const PointCloudPtr& point_cloud, const float voxel_size)
{
  // filter point cloud with constant leaf size 0.2m
  PointCloud filter_rs_cloud;
  pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud(point_cloud);
  vg.setLeafSize(voxel_size, voxel_size, voxel_size);
  vg.filter(filter_rs_cloud);
  *point_cloud = filter_rs_cloud;
}

void octreeDownsampling(const PointCloudPtr& pointcloud, const PointCloudPtr& downsampled_pointcloud, float voxel_size)
{
  pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(voxel_size);
  octree.setInputCloud(pointcloud);
  octree.defineBoundingBox();
  octree.addPointsFromInputCloud();
  pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
  // Get PointT vector of centroids for all occupied voxels
  octree.getVoxelCentroids(centroids);

  // init current map with the downsampled full cloud
  downsampled_pointcloud->clear();
  downsampled_pointcloud->points.assign(centroids.begin(), centroids.end());
  downsampled_pointcloud->width = 1;
  downsampled_pointcloud->height =
      downsampled_pointcloud->points.size();  // make sure again the format of the downsampled point cloud
}

void octreeDownsamplingWithIntensity(const PointCloudPtr& pointcloud, const PointCloudPtr& downsampled_pointcloud,
                                     float voxel_size)
{
  PointCloudPtr pointcloud_ds(new PointCloud());
  pcl::PointCloud<PointType>::Ptr ptr_reassigned(new pcl::PointCloud<PointType>);

  pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(voxel_size);
  octree.setInputCloud(pointcloud);
  octree.defineBoundingBox();
  octree.addPointsFromInputCloud();
  pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
  // Get PointT vector of centroids for all occupied voxels
  octree.getVoxelCentroids(centroids);

  // init current map with the downsampled full cloud
  pointcloud_ds->clear();
  pointcloud_ds->points.assign(centroids.begin(), centroids.end());
  pointcloud_ds->width = 1;
  pointcloud_ds->height = pointcloud_ds->points.size();  // make sure again the format of the downsampled point cloud

  // static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  // voxel_filter.setInputCloud(pointcloud);
  // voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  // downsampled_pointcloud->clear();
  // voxel_filter.filter(*downsampled_pointcloud);

  if (true)
  {
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(pointcloud);

    ptr_reassigned->points.reserve(downsampled_pointcloud->points.size());

    int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    // Set dst <- output
    for (const auto& pt : pointcloud_ds->points)
    {
      if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        auto updated = pt;
        // Update meaned intensity to original intensity
        updated.intensity = (*pointcloud)[pointIdxNKNSearch[0]].intensity;
        ptr_reassigned->points.emplace_back(updated);
      }
    }
    *downsampled_pointcloud = *ptr_reassigned;
    downsampled_pointcloud->width = 1;
    downsampled_pointcloud->height = downsampled_pointcloud->points.size();
  }
  else
  {
    *downsampled_pointcloud = *pointcloud_ds;
  }
}

void calculateCoordinate(const PointType& point, float vfov_upper, float vfov_lower, float hfov,
                         std::pair<int, int> rimg_shape, int& row, int& col)
{
  float vertical_angle_range = vfov_upper - vfov_lower;
  int image_rows = rimg_shape.first;
  int image_cols = rimg_shape.second;
  // calculate angle
  float vertical_angle =
      radToDeg(std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y)));  //[-22.5,22.5]
  float horizon_angle = radToDeg(std::atan2(point.y, point.x));                         //[-180,180]

  // calculate current points row col coordinate
  int row_index = int(image_rows * (-vertical_angle + vfov_upper) / vertical_angle_range);  // 0-32

  if (row_index < 0)
  {
    row_index = 0;
  }
  else if (row_index > image_rows)
  {
    row_index = image_rows;
  }

  int col_index = -round(horizon_angle / hfov * image_cols) + image_cols / 2;

  if (col_index >= image_cols)
  {
    col_index -= image_cols;
  }
  if (col_index < 0)
  {
    col_index += image_cols;
  }

  row = row_index;
  col = col_index;
}

void computePointcloudBboxExtend(const PointCloudPtr& pointcloud, BoundingBox& box)
{
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f extent = Eigen::Vector3f::Zero();

  Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest());
  Eigen::Vector3f min_point(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());

  int cluster_size = pointcloud->points.size();

  for (auto point : pointcloud->points)
  {
    Eigen::Vector3f point_eigen(point.x, point.y, point.z);
    min_point << std::min(min_point.x(), point_eigen.x()), std::min(min_point.y(), point_eigen.y()),
        std::min(min_point.z(), point_eigen.z());
    max_point << std::max(max_point.x(), point_eigen.x()), std::max(max_point.y(), point_eigen.y()),
        std::max(max_point.z(), point_eigen.z());
  }
  center = (max_point + min_point) / 2.0;
  extent = max_point - min_point;

  box.pose.position.x = center.x();
  box.pose.position.y = center.y();
  box.pose.position.z = center.z();

  box.pose.orientation.w = 1.0;

  box.dimensions.x = extent.x() + 0.1;
  box.dimensions.y = extent.y() + 0.1;
  box.dimensions.z = extent.z() + 0.1;
}

void computePointcloudBbox(const PointCloudPtr& pointcloud, BoundingBox& box)
{
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f extent = Eigen::Vector3f::Zero();

  Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest());
  Eigen::Vector3f min_point(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());

  // int cluster_size = pointcloud->points.size();

  for (auto point : pointcloud->points)
  {
    Eigen::Vector3f point_eigen(point.x, point.y, point.z);
    min_point << std::min(min_point.x(), point_eigen.x()), std::min(min_point.y(), point_eigen.y()),
        std::min(min_point.z(), point_eigen.z());
    max_point << std::max(max_point.x(), point_eigen.x()), std::max(max_point.y(), point_eigen.y()),
        std::max(max_point.z(), point_eigen.z());
  }

  center = (max_point + min_point) / 2.0;
  extent = max_point - min_point;

  box.pose.position.x = center.x();
  box.pose.position.y = center.y();
  box.pose.position.z = center.z();

  box.pose.orientation.w = 1.0;

  box.dimensions.x = extent.x();
  box.dimensions.y = extent.y();
  box.dimensions.z = extent.z();
}

double computeBboxIOU(const BoundingBox& box1, const BoundingBox& box2)
{
  PointType box1_min_point;
  box1_min_point.x = box1.pose.position.x - box1.dimensions.x;
  box1_min_point.y = box1.pose.position.y - box1.dimensions.y;
  box1_min_point.z = box1.pose.position.z - box1.dimensions.z;

  PointType box1_max_point;
  box1_max_point.x = box1.pose.position.x + box1.dimensions.x;
  box1_max_point.y = box1.pose.position.y + box1.dimensions.y;
  box1_max_point.z = box1.pose.position.z + box1.dimensions.z;

  PointType box2_min_point;
  box2_min_point.x = box2.pose.position.x - box2.dimensions.x;
  box2_min_point.y = box2.pose.position.y - box2.dimensions.y;
  box2_min_point.z = box2.pose.position.z - box2.dimensions.z;

  PointType box2_max_point;
  box2_max_point.x = box2.pose.position.x + box2.dimensions.x;
  box2_max_point.y = box2.pose.position.y + box2.dimensions.y;
  box2_max_point.z = box2.pose.position.z + box2.dimensions.z;

  PointType inter_max_point;
  inter_max_point.x = std::min(box1_max_point.x, box2_max_point.x);
  inter_max_point.y = std::min(box1_max_point.y, box2_max_point.y);
  inter_max_point.z = std::min(box1_max_point.z, box2_max_point.z);

  PointType inter_min_point;
  inter_min_point.x = std::max(box1_min_point.x, box2_min_point.x);
  inter_min_point.y = std::max(box1_min_point.y, box2_min_point.y);
  inter_min_point.z = std::max(box1_min_point.z, box2_min_point.z);

  double area1 = (box1_max_point.x - box1_min_point.x) * (box1_max_point.y - box1_min_point.y) *
                 (box1_max_point.z - box1_min_point.z);

  double area2 = (box2_max_point.x - box2_min_point.x) * (box2_max_point.y - box2_min_point.y) *
                 (box2_max_point.z - box2_min_point.z);

  double inter_area;
  if (inter_max_point.x <= inter_min_point.x || inter_max_point.y <= inter_min_point.y ||
      inter_max_point.z <= inter_min_point.z)
  {
    inter_area = 0.0;
  }
  else
  {
    inter_area = (inter_max_point.x - inter_min_point.x) * (inter_max_point.y - inter_min_point.y) *
                 (inter_max_point.z - inter_min_point.z);
  }

  return inter_area / (area1 + area2 - inter_area);
}

bool point_cmp(PointType a, PointType b)
{
  return a.z < b.z;
}

bool isPointInVFOV(const SphericalPoint& sph_point, float vfov_upper, float vfov_lower)
{
  if (radToDeg(sph_point.el) > vfov_upper || radToDeg(sph_point.el) < vfov_lower)
  {
    return false;
  }
  else
  {
    return true;
  }
}

cv::Mat pointcloudToRimg(const PointCloudPtr& pointcloud, std::pair<int, int> image_size, float vfov_upper,
                         float vfov_lower, float hfov)
{
  const int row_size = image_size.first;
  const int col_size = image_size.second;

  cv::Mat rimg = cv::Mat(row_size, col_size, CV_32F, cv::Scalar::all(10000.0));

  int num_points = pointcloud->points.size();

// #pragma omp parallel for num_threads(omp_cores)
  for (int idx = 0; idx < num_points; ++idx)
  {
    PointType point = pointcloud->points[idx];
    SphericalPoint sph_point = cartToSph(point);

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

    float range = sph_point.r;
    if (range < rimg.at<float>(pixel_idx_row, pixel_idx_col))
    {
      rimg.at<float>(pixel_idx_row, pixel_idx_col) = range;
    }
  }

  return rimg;
}