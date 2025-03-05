#ifndef _BACK_END_H
#define _BACK_END_H

#include "utils.h"

namespace smat
{
class ScanToMapBackEnd
{
public:
  explicit ScanToMapBackEnd(ros::NodeHandle nh);

  virtual ~ScanToMapBackEnd();

  bool readParameters();

  void scanCallback(const smat::Submap::ConstPtr& msg);

  void queryScanCallback(const geometry_msgs::PointStampedConstPtr& point_msg);

  void getScans(const PointType& curr_point, std::vector<PointCloudPtr>& nearest_scans,
                  std::vector<geometry_msgs::Pose>& nearest_scans_pose);

  virtual void occupancyEstimation(const PointType& curr_point, const PointCloudPtr& static_global_map,
                                   const PointCloudPtr& dynamic_global_map) = 0;

  void pubSubmap(const PointCloudPtr& static_map, const PointType& point);

  bool isRemove(PointType& point);

  void removeThread();

  void run();

protected:
  ros::NodeHandle nh_;

  ros::Publisher submap_pub_;
  ros::Publisher static_submap_pub_;
  ros::Publisher dynamic_submap_pub_;

  ros::Subscriber preprocess_scan_sub_;
  ros::Subscriber query_sub_;

  PointType query_point_;

  PointCloudPtr scans_points_;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_scans_points_;

  float scan_radius_;
  float voxel_size_;

  bool query_flag_;
  bool normal_est_flag_;
  bool downsampling_flag_;

  int max_nearest_size_;

  std::string local_frame_;
  std::string global_frame_;

  std::mutex scan_lock_;
  std::mutex query_lock_;

  std::vector<geometry_msgs::Pose> scans_pose_;
  std::vector<double> scans_yaw_;
  std::vector<PointCloudPtr> scans_pointcloud_;
};
}  // namespace smat

#endif