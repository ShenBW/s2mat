#ifndef _RIMG_CLUSTER_H
#define _RIMG_CLUSTER_H

#include "utils.h"

namespace s2mat
{
// ascending sort
class RimgCluster
{
public:
  explicit RimgCluster();

  virtual ~RimgCluster();

  void removeGround(const PointCloudPtr& pointcloud);

  void extractInitialSeeds(const PointCloudPtr& pointcloud, const PointCloudPtr& pointcloud_gseeds);

  void estimatePlane(const PointCloudPtr& pointcloud_gseeds, Eigen::MatrixXf& normal, float& d);

  void clusterPointcloud(const PointCloudPtr& pointcloud, const std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair,
                         cv::Mat& label_image, std::unordered_map<int, PointCloudPtr>& clusters,
                         std::unordered_map<int, std::vector<int>>& clusters_idx,
                         std::unordered_map<int, std::vector<struct Pixel>>& clusters_pixels);

  bool calculateNeighborPixel(const PointCloudPtr& pointcloud, const cv::Mat& depth_image_pidx,
                              std::pair<int, int> target_pixel, std::pair<int, int> neighbor,
                              std::pair<int, int>& neighbor_pixel);

  bool calculateNeighborPixel(const PointCloudPtr& pointcloud, const cv::Mat& depth_image_pidx,
                              std::pair<int, int> target_pixel, std::pair<int, int> neighbor,
                              std::pair<int, int>& neighbor_pixel, int max_empty_count, int& empty_count);

  void labelComponents(const PointCloudPtr& pointcloud, const cv::Mat& depth_image, const cv::Mat& depth_image_pidx,
                       cv::Mat& label_image, std::unordered_map<int, std::vector<int>>& clusters_idx,
                       std::unordered_map<int, std::vector<struct Pixel>>& clusters_pixels);

  bool judgmentCondition(const PointCloudPtr& pointcloud, int idx, int idx_neighbor, float threshold);

  int lidar_lines_;
  int lidar_hresolution_;
  int cluster_size_;

  float sensor_height_;
  float vfov_;
  float hfov_;
  float max_depth_;

  bool consider_max_depth_;
};
}  // namespace s2mat

#endif