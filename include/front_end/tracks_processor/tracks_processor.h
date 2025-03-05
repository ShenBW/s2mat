#ifndef _TRACKS_PROCESSOR_H_
#define _TRACKS_PROCESSOR_H_

#include "utils.h"

#include <smat/TrackedObjects.h>

namespace smat
{
class TracksProcessor
{
public:
  explicit TracksProcessor();

  virtual ~TracksProcessor();

  void calculateTrackDimension(pcl::ExtractIndices<PointType>& extractor, const std::vector<int>& cluster_idx,
                               TrackedObject track, Eigen::Vector3f& extent);

  void predictBboxFromTracking(TrackedObject track, BoundingBox& pred_box, double delta_time);

  void publishTrajectories(const BoundingBoxArray& tracking_msg,
                           const std::unordered_map<int, std::vector<geometry_msgs::Pose>>& traj_dict);

  void measureTracks(const std::vector<PointCloudPtr>& submaps, const std::vector<geometry_msgs::Pose>& submaps_pose,
                     const std::vector<TrackedObjects>& submaps_object, std::unordered_set<int>& success_tracking_traj);
};

}  // namespace smat
#endif