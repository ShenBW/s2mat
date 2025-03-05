#ifndef _RIMG_FRONT_END_H
#define _RIMG_FRONT_END_H

#include "utils.h"
#include "front_end/rimg_detector/scan_preprocessor.h"
#include "front_end/rimg_detector/rimg_object_detector.h"
#include "front_end/tracks_processor/tracks_processor.h"

#include "nnt/nnt_tracker.h"
#include "nnt/nnt.h"

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/MarkerArray.h>

namespace smat
{
class RimgFrontEnd
{
public:
  explicit RimgFrontEnd(ros::NodeHandle nh);

  virtual ~RimgFrontEnd();

  bool readParameters();

  void subscribePointcloud();

  void scanCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg);

  bool getTransformPose(const ros::Time& stamp, const std::string& target_frame, const std::string& source_frame,
                        geometry_msgs::Pose& pose);

  void cropMapGlobal(const PointCloudPtr& map_crop, const geometry_msgs::Pose& frame_pose);

  void scansideRemove(const PointCloudPtr& all_scan, const PointCloudPtr& static_scan,
                      const TrackedObjects& tracked_objects, std::unordered_map<int, std::vector<int>>& clusters_idx);

  void scansideRemoveByCrop(const PointCloudPtr& all_scan, const PointCloudPtr& static_scan,
                            const TrackedObjects& tracked_objects,
                            std::unordered_map<int, std::vector<int>>& clusters_idx);

  void publishPreprocessScan(const PointCloudPtr& preprocess_scan, const geometry_msgs::Pose pose);

  void publishTracking(const PointCloudPtr& scan_global, std::unordered_map<int, std::vector<int>>& clusters_idx,
                       const TrackedObjects& tracked_msg);

  double calculateDeltaTime();

  void publishTrajectories(const BoundingBoxArray& tracking_msg);

  void initMapCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg);

  void mapCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg);

  void run();

  void submapThread();

  bool isUpdate();

  void determineTracking();

  void getTemporalData(std::vector<PointCloudPtr>& temporal_submaps,
                       std::vector<geometry_msgs::Pose>& temporal_submaps_pose,
                       std::vector<TrackedObjects>& temporal_submaps_object);

protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport img_transport_;

  ros::Subscriber init_map_sub_;
  ros::Subscriber map_sub_;

  ros::Publisher compare_map_pub_;
  ros::Publisher static_scan_pub_;
  ros::Publisher preprocess_scan_pub_;
  ros::Publisher tracking_pub_;
  ros::Publisher traj_pub_;

  image_transport::Publisher scan_rimg_pub_;

  tf::TransformListener tf_listener_;

  // normal pointcloud input
  message_filters::Subscriber<sensor_msgs::PointCloud2> scan_sub_;
  std::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> scan_filter_;

  BoundingBoxArray prediction_msg_;

  PointCloudPtr map_global_;

  std::string local_frame_;
  std::string global_frame_;

  std::shared_ptr<ScanPreprocessor> preprocessor_;
  std::shared_ptr<RimgObjectDetector> object_detector_;
  std::shared_ptr<nnt::NNTracker> tracker_;
  std::shared_ptr<TracksProcessor> tracks_processor_;

  std::deque<PointCloudPtr> scan_deque_;
  std::deque<geometry_msgs::Pose> pose_deque_;
  std::deque<TrackedObjects> object_deque_;

  std::unordered_set<int> success_track_id_;
  std::unordered_map<int, int> track_id_map_;
  std::unordered_map<int, std::vector<geometry_msgs::Pose>> traj_dict_;
  std::unordered_map<int, std::vector<geometry_msgs::Pose>> traj_candidate_dict_;

  std::mutex map_global_lock_;
  std::mutex track_id_lock_;
  std::mutex deque_lock_;
  std::mutex update_lock_;

  int omp_cores_;
  int scans_count_;
  int scan_frequency_;
  int max_num_;
  int last_track_id_;

  float max_depth_;
  float voxel_size_;

  bool init_map_;
  bool update_flag_;

  std::vector<std::vector<float>> color_map_ = {
    { 0. / 255., 0. / 255., 255. / 255. },     { 250. / 255., 128. / 255., 114. / 255. },
    { 0. / 255., 255. / 255., 0. / 255. },     { 34. / 255., 139. / 255., 34. / 255. },
    { 255. / 255., 255. / 255., 0. / 255. },   { 218. / 255., 165. / 255., 32. / 255. },
    { 205. / 255., 92. / 255., 92. / 255. },   { 255. / 255., 165. / 255., 0. / 255. },
    { 255. / 255., 0. / 255., 0. / 255. },     { 255. / 255., 0. / 255., 255. / 255. },
    { 160. / 255., 32. / 255., 240. / 255. },  { 123. / 255., 104. / 255., 238. / 255. },
    { 0. / 255., 255. / 255., 255. / 255. },   { 32. / 255., 178. / 255., 170. / 255. },
    { 188. / 255., 143. / 255., 143. / 255. }, { 210. / 255., 180. / 255., 140. / 255. },
    { 255. / 255., 105. / 255., 180. / 255. }, { 240. / 255., 128. / 255., 128. / 255. },
    { 216. / 255., 191. / 255., 216. / 255. }, { 208. / 255., 32. / 255., 144. / 255. }
  };
};
}  // namespace smat

#endif