#include "front_end/rimg_detector/rimg_cluster.h"

namespace s2mat
{
RimgCluster::RimgCluster() : consider_max_depth_(true)
{
}

RimgCluster::~RimgCluster()
{
}

void RimgCluster::estimatePlane(const PointCloudPtr& pointcloud_gseeds, Eigen::MatrixXf& normal, float& d)
{
  Eigen::Matrix3f cov;
  Eigen::Vector4f mean;
  pcl::computeMeanAndCovarianceMatrix(*pointcloud_gseeds, cov, mean);
  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  normal = (svd.matrixU().col(2));
  // mean ground seeds value
  Eigen::Vector3f seeds_mean = mean.head<3>();

  // according to normal.T*[x,y,z] = -d
  d = -(normal.transpose() * seeds_mean)(0, 0);
}

void RimgCluster::extractInitialSeeds(const PointCloudPtr& pointcloud_sorted, const PointCloudPtr& pointcloud_gseeds)
{
  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;
  // Calculate the mean height value.
  for (int i = 0; i < pointcloud_sorted->points.size() && cnt < 30; i++)
  {
    sum += pointcloud_sorted->points[i].z;
    cnt++;
  }
  // in case divide by 0
  double height_lpr = cnt != 0 ? sum / cnt : 0;

  // iterate pointcloud, filter those height is less than LPR.height + threshold
  for (int i = 0; i < pointcloud_sorted->points.size(); i++)
  {
    if (pointcloud_sorted->points[i].z < height_lpr + 1.2)
    {
      pointcloud_gseeds->points.push_back(pointcloud_sorted->points[i]);
    }
  }
  // return seeds points
}

void RimgCluster::removeGround(const PointCloudPtr& pointcloud)
{
  PointCloudPtr pointcloud_sorted(new PointCloud());
  *pointcloud_sorted = *pointcloud;

  // Sort on Z-axis value
  sort(pointcloud_sorted->begin(), pointcloud_sorted->end(), point_cmp);

  // Error point removal
  pcl::PointCloud<PointType>::iterator index = pointcloud_sorted->points.begin();
  for (int i = 0; i < pointcloud_sorted->points.size(); i++)
  {
    if (pointcloud_sorted->points[i].z < (-sensor_height_ - 0.6))
    {
      index++;
    }
    else
    {
      break;
    }
  }
  pointcloud_sorted->erase(pointcloud_sorted->begin(), index);

  // Extract init ground seeds
  PointCloudPtr pointcloud_gseeds(new PointCloud());
  extractInitialSeeds(pointcloud_sorted, pointcloud_gseeds);

  // Ground plane fitter mainloop
  float th_dist = 0.25;
  // float th_dist = 0.15;

  for (int i = 0; i < 3; i++)
  {
    Eigen::MatrixXf normal;
    float d;
    estimatePlane(pointcloud_gseeds, normal, d);
    float th_dist_d = th_dist - d;

    pointcloud_gseeds->clear();

    // pointcloud to matrix
    Eigen::MatrixXf points(pointcloud->points.size(), 3);
    for (int j = 0; j < pointcloud->points.size(); j++)
    {
      auto point = pointcloud->points[j];
      points.row(j) << point.x, point.y, point.z;
    }

    // ground plane model
    Eigen::VectorXf result = points * normal;
    // threshold filter
    for (int row = 0; row < result.rows(); row++)
    {
      if (result[row] < th_dist_d)
      {
        pointcloud_gseeds->points.push_back(pointcloud->points[row]);
        // ground points
        pointcloud->points[row].intensity = 0.0;
      }
      else
      {
        pointcloud->points[row].intensity = 1.0;
      }
    }
  }
}

bool RimgCluster::calculateNeighborPixel(const PointCloudPtr& pointcloud, const cv::Mat& depth_image_pidx,
                                         std::pair<int, int> target_pixel, std::pair<int, int> neighbor,
                                         std::pair<int, int>& neighbor_pixel)
{
  int x = target_pixel.first + neighbor.first;
  if (x < 0 || x >= depth_image_pidx.rows)
  {
    return false;
  }

  int y = target_pixel.second + neighbor.second;
  if (y < 0)
  {
    y += depth_image_pidx.cols;
  }
  else if (y >= depth_image_pidx.cols)
  {
    y -= depth_image_pidx.cols;
  }

  int pidx = depth_image_pidx.at<int>(x, y);

  if (pidx < 0)
  {
    return false;
  }

  // if (pointcloud->points[pidx].intensity == 0.0)
  // {
  //     return false;
  // }

  neighbor_pixel = { x, y };
  return true;
}

bool RimgCluster::calculateNeighborPixel(const PointCloudPtr& pointcloud, const cv::Mat& depth_image_pidx,
                                         std::pair<int, int> target_pixel, std::pair<int, int> neighbor,
                                         std::pair<int, int>& neighbor_pixel, int max_empty_count, int& empty_count)
{
  std::pair<int, int> neighbor_current = neighbor;

  while (!calculateNeighborPixel(pointcloud, depth_image_pidx, target_pixel, neighbor_current, neighbor_pixel))
  {
    empty_count++;
    if (empty_count > max_empty_count)
    {
      return false;
    }
    neighbor_current.first = (empty_count + 1) * neighbor.first;
    neighbor_current.second = (empty_count + 1) * neighbor.second;
  }

  int pidx = depth_image_pidx.at<int>(neighbor_pixel.first, neighbor_pixel.second);

  if (pointcloud->points[pidx].intensity == 0.0)
  {
    return false;
  }

  return true;
}

bool RimgCluster::judgmentCondition(const PointCloudPtr& pointcloud, int idx, int idx_neighbor, float threshold)
{
  PointType p = pointcloud->points[idx];
  PointType p_n = pointcloud->points[idx_neighbor];
  float dist = std::sqrt((p.x - p_n.x) * (p.x - p_n.x) + (p.y - p_n.y) * (p.y - p_n.y) + (p.z - p_n.z) * (p.z - p_n.z));
  if (dist < threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void RimgCluster::labelComponents(const PointCloudPtr& pointcloud, const cv::Mat& depth_image,
                                  const cv::Mat& depth_image_pidx, cv::Mat& label_image,
                                  std::unordered_map<int, std::vector<int>>& clusters_idx,
                                  std::unordered_map<int, std::vector<struct Pixel>>& clusters_pixels)
{
  // 四邻域
  // vector<pair<int8_t, int8_t>> neighbor = {{1,  0},
  //                                          {-1, 0},
  //                                          {0, 1},
  //                                          {0, -1}};
  // 八邻域
  // std::vector<std::pair<int, int>> neighbors = {{1,  0},
  //                                               {1,  1},
  //                                               {1,  -1},
  //                                               {-1, 0},
  //                                               {-1, -1},
  //                                               {-1, 1},
  //                                               {0,  1},
  //                                               {0,  -1}};
  std::vector<std::pair<int, int>> neighbors = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };

  int image_rows = lidar_lines_;
  int image_cols = lidar_hresolution_;

  int label_val = 1;
  std::queue<std::pair<int, int>> query_queue;
  for (int row = 0; row < image_rows; row++)
  {
    for (int col = 0; col < image_cols; col++)
    {
      int idx = depth_image_pidx.at<int>(row, col);
      if (pointcloud->points[idx].intensity == 0.0)
      {
        continue;
      }

      if (label_image.at<int>(row, col) == 0 && depth_image_pidx.at<int>(row, col) > -1)
      {
        query_queue.push(std::make_pair(row, col));
        label_image.at<int>(row, col) = label_val;

        std::vector<int> cluster_idx;
        cluster_idx.push_back(idx);

        std::vector<struct Pixel> cluster_pixels;
        cluster_pixels.push_back(Pixel(row, col));

        float all_dist = 0;

        while (!query_queue.empty())
        {
          std::pair<int, int> target_pixel = query_queue.front();
          int idx_target = depth_image_pidx.at<int>(target_pixel.first, target_pixel.second);
          float range = depth_image.at<float>(target_pixel.first, target_pixel.second);

          all_dist += range;
          query_queue.pop();

          for (auto neighbor : neighbors)
          {
            int empty_count = 0;
            if (neighbor.first == 0)
            {
              std::pair<int, int> neighbor_pixel;
              if (!calculateNeighborPixel(pointcloud, depth_image_pidx, target_pixel, neighbor, neighbor_pixel, 0,
                                          empty_count))
              {
                continue;
              }

              int idx_neighbor = depth_image_pidx.at<int>(neighbor_pixel.first, neighbor_pixel.second);
              float alpha = hfov_ / image_cols * M_PI / 180.0;
              float threshold = 2 * range * std::sin(alpha);
              if (threshold < 0.10)
              {
                threshold = 0.10;
              }

              if (judgmentCondition(pointcloud, idx_target, idx_neighbor, (empty_count + 1) * threshold) &&
                  label_image.at<int>(neighbor_pixel.first, neighbor_pixel.second) == 0)
              {
                query_queue.push(neighbor_pixel);
                label_image.at<int>(neighbor_pixel.first, neighbor_pixel.second) = label_val;
                cluster_idx.push_back(depth_image_pidx.at<int>(neighbor_pixel.first, neighbor_pixel.second));
                cluster_pixels.push_back(Pixel(neighbor_pixel.first, neighbor_pixel.second));
              }
            }
            else
            {
              int empty_count = 0;
              // int false_count = 0;

              std::pair<int, int> neighbor_pixel;
              if (!calculateNeighborPixel(pointcloud, depth_image_pidx, target_pixel, neighbor, neighbor_pixel, 0,
                                          empty_count))
              {
                continue;
              }

              int idx_neighbor = depth_image_pidx.at<int>(neighbor_pixel.first, neighbor_pixel.second);

              float alpha = vfov_ / image_rows * M_PI / 180.0;
              float threshold = 3 * range * std::sin(alpha);

              if (threshold < 0.20)
              {
                threshold = 0.20;
              }

              if (judgmentCondition(pointcloud, idx_target, idx_neighbor, (empty_count + 1) * threshold) &&
                  label_image.at<int>(neighbor_pixel.first, neighbor_pixel.second) == 0)
              {
                query_queue.push(neighbor_pixel);
                label_image.at<int>(neighbor_pixel.first, neighbor_pixel.second) = label_val;
                cluster_idx.push_back(depth_image_pidx.at<int>(neighbor_pixel.first, neighbor_pixel.second));
                cluster_pixels.push_back(Pixel(neighbor_pixel.first, neighbor_pixel.second));
              }
            }
          }
        }

        if (consider_max_depth_)
        {
          if (all_dist / float(cluster_idx.size()) > max_depth_)
          {
            continue;
          }
        }

        // ROS_INFO_STREAM(cluster_idx_.size());
        if (cluster_idx.size() > cluster_size_)
        {
          clusters_idx.emplace(label_val, cluster_idx);
          clusters_pixels.emplace(label_val, cluster_pixels);
        }
        label_val++;
      }
    }
  }
}

void RimgCluster::clusterPointcloud(const PointCloudPtr& pointcloud, const std::pair<cv::Mat, cv::Mat>& scan_rimgs_pair,
                                    cv::Mat& label_image, std::unordered_map<int, PointCloudPtr>& clusters,
                                    std::unordered_map<int, std::vector<int>>& clusters_idx,
                                    std::unordered_map<int, std::vector<struct Pixel>>& clusters_pixels)
{
  cv::Mat depth_image = scan_rimgs_pair.first;
  cv::Mat depth_image_pidx = scan_rimgs_pair.second;

  labelComponents(pointcloud, depth_image, depth_image_pidx, label_image, clusters_idx, clusters_pixels);

  pcl::ExtractIndices<PointType> extractor;
  extractor.setInputCloud(pointcloud);

  for (auto it = clusters_idx.begin(); it != clusters_idx.end(); ++it)
  {
    int label_val = it->first;
    auto cluster_idx = it->second;

    PointCloudPtr pointcloud_clustered(new PointCloud());
    // pcl::PointCloud<PointType>::Ptr downsampling_pointcloud_clustered(new pcl::PointCloud<PointType>());

    boost::shared_ptr<std::vector<int>> idx_ptr = boost::make_shared<std::vector<int>>(cluster_idx);
    extractor.setIndices(idx_ptr);
    // If set to true, you can extract point clouds outside the specified index
    extractor.setNegative(false);

    pointcloud_clustered->clear();
    extractor.filter(*pointcloud_clustered);

    // setIntensity(pointcloud_clustered, static_cast<float>(label_val));

    // downsampling_filter_.setInputCloud(pointcloud_clustered);
    // downsampling_pointcloud_clustered->clear();
    // downsampling_filter_.filter(*downsampling_pointcloud_clustered);

    clusters.emplace(label_val, pointcloud_clustered);
  }
}
}  // namespace s2mat