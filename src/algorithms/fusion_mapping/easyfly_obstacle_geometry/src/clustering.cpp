// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include "easyfly_obstacle_geometry/clustering.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace easyfly::obstacle_geometry
{
std::vector<DynamicObstacleObservation> extractCurrentDynamicObservations(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud, const ClusteringParams & params)
{
  std::vector<DynamicObstacleObservation> observations;
  if (cloud == nullptr || cloud->size() < static_cast<std::size_t>(
      std::max(1, params.cluster_min_points)))
  {
    return observations;
  }

  std::vector<pcl::PointIndices> clusters;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> extraction;
  extraction.setClusterTolerance(params.cluster_tolerance);
  extraction.setMinClusterSize(params.cluster_min_points);
  extraction.setMaxClusterSize(params.cluster_max_points);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(cloud);
  extraction.extract(clusters);
  observations.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    float x1 = std::numeric_limits<float>::infinity();
    float y1 = std::numeric_limits<float>::infinity();
    float z1 = std::numeric_limits<float>::infinity();
    float x2 = -std::numeric_limits<float>::infinity();
    float y2 = -std::numeric_limits<float>::infinity();
    float z2 = -std::numeric_limits<float>::infinity();
    for (const auto & index : cluster.indices) {
      const auto & point = cloud->points[index];
      x1 = std::min(x1, point.x); x2 = std::max(x2, point.x);
      y1 = std::min(y1, point.y); y2 = std::max(y2, point.y);
      z1 = std::min(z1, point.z); z2 = std::max(z2, point.z);
    }
    const double sx = static_cast<double>(x2) - x1;
    const double sy = static_cast<double>(y2) - y1;
    const double sz = static_cast<double>(z2) - z1;
    if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz) ||
      std::max({sx, sy, sz}) > params.max_obstacle_size || sz < 0.05)
    {
      continue;
    }
    observations.push_back(
      {
        (static_cast<double>(x1) + x2) * 0.5,
        (static_cast<double>(y1) + y2) * 0.5,
        (static_cast<double>(z1) + z2) * 0.5,
        sx, sy, sz});
  }
  return observations;
}

std::vector<pcl::PointIndices> clusterCloud(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud, const ClusteringParams & params)
{
  std::vector<pcl::PointIndices> clusters;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(params.cluster_tolerance);
  ec.setMinClusterSize(params.cluster_min_points);
  ec.setMaxClusterSize(params.cluster_max_points);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters);
  return clusters;
}

std::array<float, 6> clusterBounds(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointIndices & cluster)
{
  float x1 = 1e9f, y1 = 1e9f, z1 = 1e9f, x2 = -1e9f, y2 = -1e9f, z2 = -1e9f;
  for (const auto & idx : cluster.indices) {
    const auto & p = cloud.points[idx];
    x1 = std::min(x1, p.x); y1 = std::min(y1, p.y); z1 = std::min(z1, p.z);
    x2 = std::max(x2, p.x); y2 = std::max(y2, p.y); z2 = std::max(z2, p.z);
  }
  return {x1, y1, z1, x2, y2, z2};
}

ClusterGeometry outputClusterGeometry(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointIndices & cluster,
  const double max_obstacle_size)
{
  ClusterGeometry result;
  result.bounds = clusterBounds(cloud, cluster);
  const auto & b = result.bounds;
  const float x1 = b[0], y1 = b[1], z1 = b[2], x2 = b[3], y2 = b[4], z2 = b[5];
  const double sx = x2 - x1, sy = y2 - y1, sz = z2 - z1;
  if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(z1) ||
    !std::isfinite(x2) || !std::isfinite(y2) || !std::isfinite(z2) ||
    !std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz) ||
    sx < 0.0 || sy < 0.0 || sz < 0.0)
  {
    result.status = ClusterGeometryStatus::kInvalidGeometry;
    return result;
  }
  if (std::max({sx, sy, sz}) > max_obstacle_size) {
    result.status = ClusterGeometryStatus::kTooLarge;
    return result;
  }
  if (sz < 0.05) {
    result.status = ClusterGeometryStatus::kFragment;
    return result;
  }
  float center_x = 0.0F;
  float center_y = 0.0F;
  float center_z = 0.0F;
  float size_x = 0.0F;
  float size_y = 0.0F;
  float size_z = 0.0F;
  if (!tryFiniteObstacleBox(
      (x1 + x2) / 2.0, (y1 + y2) / 2.0, (z1 + z2) / 2.0,
      sx, sy, sz, center_x, center_y, center_z, size_x, size_y, size_z))
  {
    // PCL points are expected to be finite, but the message fields
    // are float32.  A finite double can still overflow on the
    // conversion; reject the complete frame instead of publishing
    // a NaN/Inf or negative-size ObstacleInfo.
    result.status = ClusterGeometryStatus::kInvalidFloatBox;
    return result;
  }
  result.fields = {center_x, center_y, center_z, size_x, size_y, size_z};
  return result;
}
}  // namespace easyfly::obstacle_geometry
