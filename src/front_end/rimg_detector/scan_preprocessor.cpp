#include "front_end/rimg_detector/scan_preprocessor.h"

namespace smat
{
ScanPreprocessor::ScanPreprocessor()
{
}

ScanPreprocessor::~ScanPreprocessor()
{
}

void ScanPreprocessor::noringHandle(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
                                    const PointCloudPtr& scan_local)
{
  ros::Time stamp = pointcloud_msg->header.stamp;

  PointCloudPtr raw_scan_local(new PointCloud());
  pcl::fromROSMsg(*pointcloud_msg, *raw_scan_local);

  if (blind_ == 0.0)
  {
    *scan_local = *raw_scan_local;
  }
  else
  {
#pragma omp parallel for num_threads(omp_cores)
    for (int i = 0; i < raw_scan_local->points.size(); i++)
    {
      PointType point = raw_scan_local->points[i];
      if ((point.x * point.x + point.y * point.y) > (blind_ * blind_))
      {
        scan_local->points.push_back(point);
      }
    }
  }
}

void ScanPreprocessor::ousterHandle(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
                                    const OusterPointCloudPtr& scan_local)
{
  ros::Time stamp = pointcloud_msg->header.stamp;

  OusterPointCloudPtr raw_scan_local(new OusterPointCloud());
  pcl::fromROSMsg(*pointcloud_msg, *raw_scan_local);

  if (blind_ == 0.0)
  {
    *scan_local = *raw_scan_local;
  }
  else
  {
#pragma omp parallel for num_threads(omp_cores)
    for (int i = 0; i < raw_scan_local->points.size(); i++)
    {
      OusterPointType point = raw_scan_local->points[i];
      if ((point.x * point.x + point.y * point.y) > (blind_ * blind_))
      {
        scan_local->points.push_back(point);
      }
    }
  }
}

void ScanPreprocessor::velodyneHandle(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
                                      const VelodynePointCloudPtr scan_local)
{
  ros::Time stamp = pointcloud_msg->header.stamp;

  VelodynePointCloudPtr raw_scan_local(new VelodynePointCloud());
  pcl::fromROSMsg(*pointcloud_msg, *raw_scan_local);

  if (blind_ == 0.0)
  {
    *scan_local = *raw_scan_local;
  }
  else
  {
#pragma omp parallel for num_threads(omp_cores)
    for (int i = 0; i < raw_scan_local->points.size(); i++)
    {
      VelodynePointType point = raw_scan_local->points[i];
      if ((point.x * point.x + point.y * point.y) > (blind_ * blind_))
      {
        scan_local->points.push_back(point);
      }
    }
  }
}

void ScanPreprocessor::noringPointCloudToRangeImg(const PointCloudPtr& pointcloud, std::pair<int, int> rimg_shape,
                                                  std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair)
{
  const float vfov = vfov_upper_ - vfov_lower_;

  const int row_size = rimg_shape.first;
  const int col_size = rimg_shape.second;

  // range image initizliation
  cv::Mat rimg = cv::Mat(row_size, col_size, CV_32FC1, cv::Scalar::all(10000.0));  // float matrix, save range value
  cv::Mat rimg_ptidx =
      cv::Mat(row_size, col_size, CV_32SC1, cv::Scalar::all(-1));  // int matrix, save point (of global map) index
  // points to range img
  int num_points = pointcloud->points.size();

#pragma omp parallel for num_threads(omp_cores)

  for (int idx = 0; idx < num_points; ++idx)
  {
    PointType point = pointcloud->points[idx];
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
    {
      continue;
    }

    SphericalPoint sph_point = cartToSph(point);

    // note about vfov: e.g., (+ vfov/2) to adjust [-15, 15] to [0, 30]
    // min and max is just for the easier (naive) boundary checks.
    int lower_bound_row_idx = 0;
    // int lower_bound_col_idx = 0;
    int upper_bound_row_idx = row_size - 1;
    // int upper_bound_col_idx = col_size - 1;
    // int pixel_idx_row = int(std::min(std::max(std::round((row_size - 1) * (1 - (radToDeg(sph_point.el) + (vfov /
    // float(2.0))) / vfov)), float(lower_bound_row_idx)), float(upper_bound_row_idx)));
    int pixel_idx_row =
        int(std::min(std::max(std::round((row_size - 1) * (1 - (radToDeg(sph_point.el) - vfov_lower_) / vfov)),
                              float(lower_bound_row_idx)),
                     float(upper_bound_row_idx)));
    // int pixel_idx_col = int(std::min(
    //     std::max(std::round((col_size - 1) * ((radToDeg(sph_point.az) + (hfov / float(2.0))) / hfov)),
    //     float(lower_bound_col_idx)), float(upper_bound_col_idx)));
    int pixel_idx_col = std::round(col_size * ((radToDeg(sph_point.az) + (hfov_ / float(2.0))) / hfov_));
    if (pixel_idx_col < 0)
    {
      pixel_idx_col += col_size;
    }
    else if (pixel_idx_col >= col_size)
    {
      pixel_idx_col -= col_size;
    }

    float range = sph_point.r;

    if (range < rimg.at<float>(pixel_idx_row, pixel_idx_col))
    {
      rimg.at<float>(pixel_idx_row, pixel_idx_col) = range;
      rimg_ptidx.at<int>(pixel_idx_row, pixel_idx_col) = idx;
    }
  }

  scan_rimgs_pair = std::pair<cv::Mat, cv::Mat>(rimg, rimg_ptidx);
}

void ScanPreprocessor::ousterPointCloudToRangeImg(const OusterPointCloudPtr& pointcloud, std::pair<int, int> rimg_shape,
                                                  std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair)
{
  const int row_size = rimg_shape.first;
  const int col_size = rimg_shape.second;

  // range image initizliation
  cv::Mat rimg = cv::Mat(row_size, col_size, CV_32FC1, cv::Scalar::all(1000.0));  // float matrix, save range value
  cv::Mat rimg_ptidx =
      cv::Mat(row_size, col_size, CV_32SC1, cv::Scalar::all(-1));  // int matrix, save point (of global map) index

  // points to range img
  int num_points = pointcloud->points.size();

#pragma omp parallel for num_threads(omp_cores)

  for (int idx = 0; idx < num_points; ++idx)
  {
    OusterPointType ousterpoint = pointcloud->points[idx];
    if (std::isnan(ousterpoint.x) || std::isnan(ousterpoint.y) || std::isnan(ousterpoint.z))
    {
      continue;
    }
    PointType point;
    point.x = ousterpoint.x;
    point.y = ousterpoint.y;
    point.z = ousterpoint.z;

    SphericalPoint sph_point = cartToSph(point);

    // int lower_bound_col_idx = 0;
    // int upper_bound_col_idx = col_size - 1;

    // int pixel_idx_col = int(std::min(
    //     std::max(std::round((col_size - 1) * ((radToDeg(sph_point.az) + (hfov / float(2.0))) / hfov)),
    //     float(lower_bound_col_idx)), float(upper_bound_col_idx)));

    int pixel_idx_row = ousterpoint.ring;
    int pixel_idx_col = std::round(col_size * ((radToDeg(sph_point.az) + (hfov_ / float(2.0))) / hfov_));
    if (pixel_idx_col < 0)
    {
      pixel_idx_col += col_size;
    }
    else if (pixel_idx_col >= col_size)
    {
      pixel_idx_col -= col_size;
    }

    float range = sph_point.r;

    if (range < rimg.at<float>(pixel_idx_row, pixel_idx_col))
    {
      rimg.at<float>(pixel_idx_row, pixel_idx_col) = range;
      rimg_ptidx.at<int>(pixel_idx_row, pixel_idx_col) = idx;
    }
  }

  scan_rimgs_pair = std::pair<cv::Mat, cv::Mat>(rimg, rimg_ptidx);
}

void ScanPreprocessor::velodynePointCloudToRangeImg(const VelodynePointCloudPtr& pointcloud,
                                                    std::pair<int, int> rimg_shape,
                                                    std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair)
{
  const int row_size = rimg_shape.first;
  const int col_size = rimg_shape.second;

  // range image initizliation
  cv::Mat rimg = cv::Mat(row_size, col_size, CV_32FC1, cv::Scalar::all(1000.0));  // float matrix, save range value
  cv::Mat rimg_ptidx =
      cv::Mat(row_size, col_size, CV_32SC1, cv::Scalar::all(-1));  // int matrix, save point (of global map) index

  // points to range img
  int num_points = pointcloud->points.size();

#pragma omp parallel for num_threads(omp_cores)

  for (int idx = 0; idx < num_points; ++idx)
  {
    VelodynePointType velodynepoint = pointcloud->points[idx];
    if (std::isnan(velodynepoint.x) || std::isnan(velodynepoint.y) || std::isnan(velodynepoint.z))
    {
      continue;
    }
    PointType point;
    point.x = velodynepoint.x;
    point.y = velodynepoint.y;
    point.z = velodynepoint.z;

    SphericalPoint sph_point = cartToSph(point);

    // int lower_bound_col_idx = 0;
    // int upper_bound_col_idx = col_size - 1;

    // int pixel_idx_col = int(std::min(
    //     std::max(std::round((col_size - 1) * ((radToDeg(sph_point.az) + (hfov / float(2.0))) / hfov)),
    //     float(lower_bound_col_idx)), float(upper_bound_col_idx)));

    int pixel_idx_row = row_size - 1 - static_cast<int>(velodynepoint.ring);
    int pixel_idx_col = std::round(col_size * ((radToDeg(sph_point.az) + (hfov_ / float(2.0))) / hfov_));
    if (pixel_idx_col < 0)
    {
      pixel_idx_col += col_size;
    }
    else if (pixel_idx_col >= col_size)
    {
      pixel_idx_col -= col_size;
    }

    float range = sph_point.r;

    if (range < rimg.at<float>(pixel_idx_row, pixel_idx_col))
    {
      rimg.at<float>(pixel_idx_row, pixel_idx_col) = range;
      rimg_ptidx.at<int>(pixel_idx_row, pixel_idx_col) = idx;
    }
  }

  scan_rimgs_pair = std::pair<cv::Mat, cv::Mat>(rimg, rimg_ptidx);
}

void ScanPreprocessor::process(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
                               std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair, const PointCloudPtr& scan_local)
{
  if (!is_ring_available_)
  {
    PointCloudPtr scan_lidar(new PointCloud());
    noringHandle(pointcloud_msg, scan_lidar);

    noringPointCloudToRangeImg(scan_lidar, { lidar_lines_, lidar_hresolution_ }, scan_rimgs_pair);

    *scan_local = *scan_lidar;
  }
  else
  {
    switch (lidar_type_)
    {
      case 1: {
        OusterPointCloudPtr scan_lidar(new OusterPointCloud());
        ousterHandle(pointcloud_msg, scan_lidar);

        ousterPointCloudToRangeImg(scan_lidar, { lidar_lines_, lidar_hresolution_ }, scan_rimgs_pair);

#pragma omp parallel for num_threads(omp_cores)
        for (int i = 0; i < scan_lidar->points.size(); i++)
        {
          PointType point;
          point.x = scan_lidar->points[i].x;
          point.y = scan_lidar->points[i].y;
          point.z = scan_lidar->points[i].z;
          scan_local->points.push_back(point);
        }

        break;
      }

      case 2: {
        VelodynePointCloudPtr scan_lidar(new VelodynePointCloud());
        velodyneHandle(pointcloud_msg, scan_lidar);

        velodynePointCloudToRangeImg(scan_lidar, { lidar_lines_, lidar_hresolution_ }, scan_rimgs_pair);

#pragma omp parallel for num_threads(omp_cores_)
        for (int i = 0; i < scan_lidar->points.size(); i++)
        {
          PointType point;
          point.x = scan_lidar->points[i].x;
          point.y = scan_lidar->points[i].y;
          point.z = scan_lidar->points[i].z;
          scan_local->points.push_back(point);
        }

        break;
      }

      default:
        ROS_ERROR_STREAM("Error Lidar Type.");
        break;
    }
  }
}

void ScanPreprocessor::kittiPointCloudToRangeImg(const PointCloudPtr& pointcloud,
                                                 std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair)
{
  const int row_size = 64;
  const int col_size = 2150;

  // range image initizliation
  cv::Mat rimg = cv::Mat(row_size, col_size, CV_32FC1, cv::Scalar::all(10000.0));  // float matrix, save range value
  cv::Mat rimg_ptidx =
      cv::Mat(row_size, col_size, CV_32SC1, cv::Scalar::all(-1));  // int matrix, save point (of global map) index
  // points to range img
  int num_points = pointcloud->points.size();

#pragma omp parallel for num_threads(omp_cores)

  for (int idx = 0; idx < num_points; ++idx)
  {
    PointType point = pointcloud->points[idx];
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
    {
      continue;
    }

    // if (point.z > 3.0)
    // {
    //     continue;
    // }

    SphericalPoint sph_point = cartToSph(point);

    int pixel_idx_row;
    if (radToDeg(sph_point.el) >= -8.6)
    {
      int lower_bound_row_idx = 0;
      int upper_bound_row_idx = 31;

      pixel_idx_row = int(std::min(std::max(std::round(31 * (1 - (radToDeg(sph_point.el) + float(8.5)) / float(10.5))),
                                            float(lower_bound_row_idx)),
                                   float(upper_bound_row_idx)));
    }
    else if (radToDeg(sph_point.el) < -8.8)
    {
      int lower_bound_row_idx = 32;
      int upper_bound_row_idx = 63;

      pixel_idx_row =
          int(std::min(std::max(std::round(32 + 31 * (1 - (radToDeg(sph_point.el) + float(24.87)) / float(16))),
                                float(lower_bound_row_idx)),
                       float(upper_bound_row_idx)));
    }
    else
    {
      if (radToDeg(sph_point.el) >= -8.7)
      {
        pixel_idx_row = 31;
      }
      else
      {
        pixel_idx_row = 32;
      }
    }

    int pixel_idx_col = std::round(col_size * ((radToDeg(sph_point.az) + (hfov_ / float(2.0))) / hfov_));
    if (pixel_idx_col < 0)
    {
      pixel_idx_col += col_size;
    }
    else if (pixel_idx_col >= col_size)
    {
      pixel_idx_col -= col_size;
    }

    float range = sph_point.r;

    if (range < rimg.at<float>(pixel_idx_row, pixel_idx_col))
    {
      rimg.at<float>(pixel_idx_row, pixel_idx_col) = range;
      rimg_ptidx.at<int>(pixel_idx_row, pixel_idx_col) = idx;
    }
  }

  scan_rimgs_pair = std::pair<cv::Mat, cv::Mat>(rimg, rimg_ptidx);
}

void ScanPreprocessor::convertVelodyneToNormal(const VelodynePointCloudPtr& lidar_pointcloud,
                                               const PointCloudPtr& pointcloud)
{
#pragma omp parallel for num_threads(omp_cores)
  for (int i = 0; i < lidar_pointcloud->points.size(); i++)
  {
    PointType point;
    point.x = lidar_pointcloud->points[i].x;
    point.y = lidar_pointcloud->points[i].y;
    point.z = lidar_pointcloud->points[i].z;
    pointcloud->points.push_back(point);
  }
}
}  // namespace smat