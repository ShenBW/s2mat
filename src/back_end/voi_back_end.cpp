#include "back_end/voi_back_end.h"

namespace smat
{
VoiBackEnd::VoiBackEnd(ros::NodeHandle nh) : ScanToMapBackEnd(nh)
{
  readVoiParameters();
}

VoiBackEnd::~VoiBackEnd()
{
}

bool VoiBackEnd::readVoiParameters()
{
  nh_.param<int>("omp_cores", omp_cores_, 4);

  nh_.param<int>("lidar_lines", image_size_.first, 32);
  nh_.param<int>("image_width", image_size_.second, 360);

  nh_.param<float>("lidar_vfov_upper", vfov_upper_, 22.5);
  nh_.param<float>("lidar_vfov_lower", vfov_lower_, -22.5);
  nh_.param<float>("lidar_hfov", hfov_, 360.0);

  return true;
}

bool VoiBackEnd::isObservedByScan(const PointType& point, const geometry_msgs::Pose& pose, const cv::Mat& ref_rimg)
{
  auto local_point = pcl::transformPoint(point, poseToMatrix(pose).inverse());

  const float vfov_upper = vfov_upper_;
  const float vfov_lower = vfov_lower_;
  const float hfov = hfov_;

  const int row_size = image_size_.first;
  const int col_size = image_size_.second;

  SphericalPoint sph_point = cartToSph(local_point);

  if (!isPointInVFOV(sph_point, vfov_upper, vfov_lower))
  {
    return false;
  }

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

  float ref_range = ref_rimg.at<float>(pixel_idx_row, pixel_idx_col);

  // ref_rang = 10000.0: 1. no observation, 2. removed dynamic points
  if (sph_point.r > std::min(80.0, ref_range * 0.9))
  {
    return false;
  }
  else
  {
    return true;
  }
}

int VoiBackEnd::getObservedNum(const PointType& point, const std::vector<geometry_msgs::Pose>& scans_pose,
                               const std::vector<cv::Mat>& ref_rimgs)
{  // namespace smatbserved_num = 0;
  int observed_num = 0;
  for (int i = 0; i < scans_pose.size(); ++i)
  {
    if (isObservedByScan(point, scans_pose[i], ref_rimgs[i]))
    {
      observed_num++;
    }
  }

  return observed_num;
}

cv::Mat VoiBackEnd::pointcloudToRimg(const PointCloudPtr& pointcloud)
{
  const float vfov_upper = vfov_upper_;
  const float vfov_lower = vfov_lower_;
  const float hfov = hfov_;

  const int row_size = image_size_.first;
  const int col_size = image_size_.second;

  // ROS_INFO_STREAM("row: " << row_size << " | col: " << col_size);

  cv::Mat rimg = cv::Mat(row_size, col_size, CV_32F, cv::Scalar::all(10000.0));

  int num_points = pointcloud->points.size();

#pragma omp parallel for num_threads(omp_cores_)
  for (int idx = 0; idx < num_points; ++idx)
  {
    PointType point = pointcloud->points[idx];
    SphericalPoint sph_point = cartToSph(point);

    // note about vfov: e.g., (+ vfov/2) to adjust [-15, 15] to [0, 30]
    // min and max is just for the easier (naive) boundary checks.
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

void VoiBackEnd::occupancyEstimation(const PointType& curr_point, const PointCloudPtr& static_global_map,
                                     const PointCloudPtr& dynamic_global_map)
{
  double start_time = ros::Time::now().toSec();

  std::vector<PointCloudPtr> scans;
  std::vector<geometry_msgs::Pose> scans_pose;

  getScans(curr_point, scans, scans_pose);

  double get_scans_time = ros::Time::now().toSec();
  ROS_INFO_STREAM("[Back End] get scans process time: " << get_scans_time - start_time);

  int scans_size = scans.size();

  if (scans_size < 5)
  {
    return;
  }

  ROS_INFO_STREAM("[Back End]: scans_size: " << scans_size);

  auto nearest_scan_pose = scans_pose[0];

  std::vector<PointCloudPtr> scans_at_pose;
  std::vector<cv::Mat> ref_rimgs;
  
  PointCloudPtr global_map(new PointCloud());
  std::unordered_map<struct Cell, std::unordered_set<int>, hash_cell> cell_map;
  for (int i = 0; i < scans_size; i++)
  {
#pragma omp parallel for num_threads(omp_cores_)

    for (int j = 0; j < scans[i]->points.size(); j++)
    {
      PointType point = scans[i]->points[j];
      struct Cell cell = point_to_cell(point, voxel_size_);
      cell_map[cell].insert(i);
    }

    *global_map += *scans[i];

    PointCloudPtr scan_local(new PointCloud());
    pcl::transformPointCloud(*scans[i], *scan_local, poseToMatrix(scans_pose[i]).inverse());

    ref_rimgs.push_back(pointcloudToRimg(scan_local));
  }

  double transform_scans_time = ros::Time::now().toSec();
  ROS_INFO_STREAM("[Back End] transform scans process time: " << transform_scans_time - get_scans_time);

  pcl::PointCloud<PointType>::Ptr downsampled_pointcloud(new pcl::PointCloud<PointType>());
  octreeDownsamplingWithIntensity(global_map, downsampled_pointcloud, voxel_size_);

  static_global_map->clear();
  dynamic_global_map->clear();

#pragma omp parallel for num_threads(omp_cores_)
  for (int i = 0; i < downsampled_pointcloud->points.size(); i++)
  {
    PointType point;
    point.x = downsampled_pointcloud->points[i].x;
    point.y = downsampled_pointcloud->points[i].y;
    point.z = downsampled_pointcloud->points[i].z;
    point.intensity = downsampled_pointcloud->points[i].intensity;

    int free_num = getObservedNum(point, scans_pose, ref_rimgs);

    auto cell = point_to_cell(point, voxel_size_);
    int occupied_num = cell_map[cell].size();

    float occupancy = (float)occupied_num / ((float)free_num + (float)occupied_num);

    if (occupancy >= 0.30)
    {
      static_global_map->push_back(point);
    }
    else
    {
      dynamic_global_map->push_back(point);
    }
  }

  // pcl::transformPointCloud(*static_global_map_at_pose, *static_global_map, poseToMatrix(nearest_scan_pose));
  // pcl::transformPointCloud(*dynamic_global_map_at_pose, *dynamic_global_map, poseToMatrix(nearest_scan_pose));

  // static_global_map->width = 1;
  // static_global_map->height = static_global_map->points.size();

  // dynamic_global_map->width = 1;
  // dynamic_global_map->height = dynamic_global_map->points.size();

  ROS_INFO_STREAM("[integrate]: process time: " << ros::Time::now().toSec() - start_time);
}
}  // namespace smat