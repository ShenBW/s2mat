#ifndef _VOI_BACK_END_H
#define _VOI_BACK_END_H

#include "back_end/back_end.h"

namespace s2mat
{
class VoiBackEnd : public ScanToMapBackEnd
{
public:
  explicit VoiBackEnd(ros::NodeHandle nh);

  virtual ~VoiBackEnd();

  bool readVoiParameters();

  virtual void occupancyEstimation(const PointType& curr_point, const PointCloudPtr& static_global_map,
                                   const PointCloudPtr& dynamic_global_map) override;

  bool isObservedByScan(const PointType& point, const geometry_msgs::Pose& pose, const cv::Mat& ref_rimg);

  int getObservedNum(const PointType& point, const std::vector<geometry_msgs::Pose>& scans_pose,
                     const std::vector<cv::Mat>& ref_rimgs);

  cv::Mat pointcloudToRimg(const PointCloudPtr& pointcloud);

private:
  std::pair<int, int> image_size_;

  int omp_cores_;

  float vfov_upper_;
  float vfov_lower_;
  float hfov_;

  bool use_rimg_;
};
}  // namespace s2mat

#endif
