#ifndef _MAP_MERGER_H
#define _MAP_MERGER_H

#include "utils.h"

namespace smat
{
typedef std::pair<PointCloudPtr, geometry_msgs::Pose> MapPosePair;
typedef std::pair<Eigen::Vector3f, int> RunningMean;

class MapMerger
{
public:
  explicit MapMerger(ros::NodeHandle nh);

  virtual ~MapMerger();

  void readParameters();

  void staticScanCallback(const smat::Submap::ConstPtr& scan_msg);

  void getGlobalScanPos(const PointType& scan_pos);

  void staticSubmapCallback(const smat::Submap::ConstPtr& submap_msg);

  void run();

  void mergeThread();

  bool isMerge(std::vector<int>& submaps_id, std::vector<std::unordered_set<struct Cell, hash_cell>>& occ_cells_vector,
               std::vector<geometry_msgs::Pose>& poses, std::vector<cv::Mat>& ref_rimgs, PointType& scan_pos);

  void incrementalMerge(std::vector<int>& submaps_id,
                        std::vector<std::unordered_set<struct Cell, hash_cell>>& occ_cells_vector,
                        std::vector<geometry_msgs::Pose>& poses, std::vector<cv::Mat>& ref_rimgs);

  bool isObservedBySubmap(const PointType& point, const geometry_msgs::Pose& pose, const cv::Mat& ref_rimg);

  void publishStaticMap(const PointType& scan_pos);

  void saveCallback(const std_msgs::EmptyConstPtr& msg);

private:
  ros::NodeHandle nh_;

  ros::Subscriber preprocess_scan_sub_;
  ros::Subscriber static_submap_sub_;
  ros::Subscriber save_sub_;

  ros::Publisher req_point_pub_;
  ros::Publisher static_map_pub_;

  PointCloudPtr submap_poses_;
  PointCloudPtr final_static_global_map_;

  PointType scan_pos_;

  std::string global_frame_;
  std::string output_static_map_path_;

  std::pair<int, int> image_size_;

  std::vector<geometry_msgs::Pose> submap_pose_vector_;
  std::vector<cv::Mat> submap_rimg_vector_;

  std::unordered_map<int, MapPosePair> submap_pose_pair_map_;
  std::unordered_map<struct Cell, int, hash_cell> occupied_map_;
  std::unordered_map<struct Cell, RunningMean, hash_cell> avg_pos_in_cells_;
  std::unordered_map<struct Cell, float, hash_cell> distance_map_;

  std::mutex scan_pos_mutex_;
  std::mutex submap_poses_mutex_;
  std::mutex submap_vectors_mutex_;

  int scan_frequency_;
  int scans_count_;
  int curr_scans_num_;
  int curr_submap_id_;
  int omp_cores_;

  float merge_dist_;
  float local_radius_;
  float vfov_upper_;
  float vfov_lower_;
  float hfov_;
  float voxel_size_;

  bool output_local_;
};
}  // namespace smat

#endif