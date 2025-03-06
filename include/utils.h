#ifndef _UTILS_H
#define _UTILS_H

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <s2mat/Submap.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.h>

#include <boost/make_shared.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/functional/hash.hpp>

#include <omp.h>
#include <math.h>

#include <limits>
#include <iostream>
#include <string>
#include <queue>
#include <deque>
#include <vector>
#include <set>
#include <mutex>
#include <thread>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

typedef jsk_recognition_msgs::BoundingBox BoundingBox;
typedef jsk_recognition_msgs::BoundingBoxArray BoundingBoxArray;
typedef jsk_recognition_msgs::BoundingBoxArrayConstPtr BoundingBoxArrayConstPtr;

struct SphericalPoint
{
  float az;  // azimuth
  float el;  // elevation
  float r;   // radius
};

struct Pixel
{
  int r;
  int c;

  Pixel(const int R, const int C) : r(R), c(C)
  {
  }

  bool operator==(const Pixel& p) const
  {
    return r == p.r && c == p.c;
  }
};

struct hash_pixel
{
  std::size_t operator()(struct Pixel const& p) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, p.r);
    boost::hash_combine(seed, p.c);
    return seed;
  }
};

struct Cell
{
  int x;
  int y;
  int z;

  Cell(const int X, const int Y, const int Z) : x(X), y(Y), z(Z)
  {
  }

  bool operator==(const Cell& c) const
  {
    return x == c.x && y == c.y && z == c.z;
  }
};

struct hash_cell
{
  std::size_t operator()(struct Cell const& t) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, t.x);
    boost::hash_combine(seed, t.y);
    boost::hash_combine(seed, t.z);
    return seed;
  }
};

extern ssize_t py_div(float a, float b);

extern struct Cell point_to_cell(const PointType& p, float voxel_size);

extern PointType cell_to_point(const struct Cell& cell, float voxel_size);

extern void pointcloud_to_cellmap(const PointCloudPtr& pointcloud, std::unordered_set<struct Cell, hash_cell>& cell_map,
                                  float voxel_size);

extern Eigen::Affine3f poseToMatrix(const geometry_msgs::Pose& pose);

extern void publishScan(ros::Publisher pub, const PointCloudPtr& scan, const ros::Time& stamp, std::string frame_id);

extern void publishMap(ros::Publisher pub, const PointCloudPtr& map, const ros::Time& stamp, std::string frame_id);

extern cv::Mat convertColorMappedImg(const cv::Mat& src, std::pair<float, float> color_axis);

extern void publishRangeImg(const image_transport::Publisher& pub, const cv::Mat& rimg, const ros::Time& stamp,
                            std::pair<float, float> color_axis);

extern float point_to_pose_distance(const PointType& point, const geometry_msgs::Pose& pose);

extern float radToDeg(float rad);

extern SphericalPoint cartToSph(const PointType& point_cor);

extern double point_to_range(const PointType& point);

extern std::pair<int, int> resetRimgSize(std::pair<float, float> FOV, float alpha_res);

extern void setIntensity(const PointCloudPtr& pc, float intensity);

extern void voxelFilter(const PointCloudPtr& point_cloud, const float voxel_size);

extern void octreeDownsampling(const PointCloudPtr& pointcloud, const PointCloudPtr& downsampled_pointcloud,
                               float voxel_size);

extern void octreeDownsamplingWithIntensity(const PointCloudPtr& pointcloud,
                                            const PointCloudPtr& downsampled_pointcloud, float voxel_size);

extern void calculateCoordinate(const PointType& point, float vfov_upper, float vfov_lower, float hfov,
                                std::pair<int, int> rimg_shape, int& row, int& col);

extern void computePointcloudBboxExtend(const PointCloudPtr& pointcloud, BoundingBox& box);

extern void computePointcloudBbox(const PointCloudPtr& pointcloud, BoundingBox& box);

extern double computeBboxIOU(const BoundingBox& box1, const BoundingBox& box2);

extern bool point_cmp(PointType a, PointType b);

extern bool isPointInVFOV(const SphericalPoint& sph_point, float vfov_upper, float vfov_lower);

extern cv::Mat pointcloudToRimg(const PointCloudPtr& pointcloud, std::pair<int, int> image_size, float vfov_upper,
                                float vfov_lower, float hfov);

#endif