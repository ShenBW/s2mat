#ifndef _SCAN_PREPROCESSOR_H
#define _SCAN_PREPROCESSOR_H

#include "utils.h"

struct OusterPointType
{
  PCL_ADD_POINT4D;                 // quad-word XYZ
  PCL_ADD_INTENSITY;               ///< laser intensity reading
  std::uint8_t ring;               ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct VelodynePointType
{
  PCL_ADD_POINT4D;                 // quad-word XYZ
  PCL_ADD_INTENSITY;               ///< laser intensity reading
  std::uint16_t ring;              ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

typedef pcl::PointCloud<OusterPointType> OusterPointCloud;
typedef pcl::PointCloud<OusterPointType>::Ptr OusterPointCloudPtr;

typedef pcl::PointCloud<VelodynePointType> VelodynePointCloud;
typedef pcl::PointCloud<VelodynePointType>::Ptr VelodynePointCloudPtr;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint8_t,
                                                                                                       ring, ring))

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t,
                                                                                                       ring, ring))

namespace s2mat
{
class ScanPreprocessor
{
public:
  explicit ScanPreprocessor();

  virtual ~ScanPreprocessor();

  void process(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg, std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair,
               const PointCloudPtr& scan_local);

  void noringHandle(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg, const PointCloudPtr& scan_local);

  void ousterHandle(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg, const OusterPointCloudPtr& scan_local);

  void velodyneHandle(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg, const VelodynePointCloudPtr scan_local);

  void noringPointCloudToRangeImg(const PointCloudPtr& pointcloud, std::pair<int, int> rimg_shape,
                                  std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair);

  void ousterPointCloudToRangeImg(const OusterPointCloudPtr& pointcloud, std::pair<int, int> rimg_shape,
                                  std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair);

  void velodynePointCloudToRangeImg(const VelodynePointCloudPtr& pointcloud, std::pair<int, int> rimg_shape,
                                    std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair);

  void kittiPointCloudToRangeImg(const PointCloudPtr& pointcloud, std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair);

  void convertVelodyneToNormal(const VelodynePointCloudPtr& lidar_pointcloud, const PointCloudPtr& pointcloud);

  int lidar_type_;
  int lidar_lines_;
  int lidar_hresolution_;

  float blind_;
  float vfov_upper_;
  float vfov_lower_;
  float hfov_;

  bool is_ring_available_;
};
}  // namespace s2mat

#endif