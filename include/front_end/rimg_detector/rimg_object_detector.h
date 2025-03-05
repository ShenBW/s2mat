#ifndef _RIMG_OBJECT_DETECTOR_H
#define _RIMG_OBJECT_DETECTOR_H

#include "utils.h"
#include "front_end/rimg_detector/rimg_cluster.h"
#include "rectangular_lsap/rectangular_lsap.h"

namespace smat
{
class RimgObjectDetector
{
public:
  explicit RimgObjectDetector(ros::NodeHandle nh);

  virtual ~RimgObjectDetector();

  void detect(const PointCloudPtr& static_map, const PointCloudPtr& scan_local,
              const std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair, const geometry_msgs::Pose& frame_pose,
              BoundingBoxArray prediction_msg, BoundingBoxArray& bbox_msg,
              std::unordered_map<int, std::vector<int>>& clusters_idx,
              std::unordered_map<int, std::vector<struct Pixel>>& clusters_pixels);

  void detectNoPredictions(const PointCloudPtr& static_map, const PointCloudPtr& scan_local,
                           const std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair, const geometry_msgs::Pose& frame_pose,
                           BoundingBoxArray& bbox_msg, std::unordered_map<int, std::vector<int>>& clusters_idx,
                           std::unordered_map<int, std::vector<struct Pixel>>& clusters_pixels);

  void filterCluster(const std::unordered_map<int, PointCloudPtr>& clusters_local,
                     pcl::CropBox<PointType>& static_map_crop_filter, const geometry_msgs::Pose& frame_pose,
                     BoundingBoxArray& bbox_msg);

  bool isClusterStatic(const PointCloudPtr& cluster, pcl::CropBox<PointType>& static_map_crop_filter, BoundingBox box);

  void generateBboxFromPredictions(const PointCloudPtr& scan_local, pcl::CropBox<PointType>& static_map_crop_filter,
                                   const geometry_msgs::Pose& frame_pose, const BoundingBoxArray& prediction_msg,
                                   BoundingBoxArray& pred_bbox_msg,
                                   std::unordered_map<int, std::vector<int>>& clusters_idx);

  bool calculateBboxPointcloud(const PointCloudPtr& scan, const BoundingBox& prediction_box,
                               pcl::CropBox<PointType>& crop_filter, const PointCloudPtr& bcluster,
                               std::vector<int>& idx_inside, int max_iterations);

  void bboxMatching(const BoundingBoxArray& bbox_msg_1, const BoundingBoxArray& bbox_msg_2,
                    BoundingBoxArray& output_bbox_msg, bool output_matching);

  void mergeBbox(BoundingBoxArray& src_bbox_msg, const BoundingBoxArray& dst_bbox_msg);

  ros::NodeHandle nh_;

  std::shared_ptr<RimgCluster> cluster_;
  // std::shared_ptr<SLRCluster> cluster_;

  ros::Publisher detection_bbox_pub_;
  ros::Publisher missing_bbox_pub_;
  ros::Publisher proposal_bbox_pub_;

  float voxel_size_;
};

}  // namespace smat
#endif
