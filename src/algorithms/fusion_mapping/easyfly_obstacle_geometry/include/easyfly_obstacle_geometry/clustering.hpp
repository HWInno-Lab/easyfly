// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#ifndef EASYFLY_OBSTACLE_GEOMETRY__CLUSTERING_HPP_
#define EASYFLY_OBSTACLE_GEOMETRY__CLUSTERING_HPP_

#include <array>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include "easyfly_obstacle_geometry/dynamic_obstacle_tracker.hpp"

namespace easyfly::obstacle_geometry
{
struct ClusteringParams
{
  double cluster_tolerance{0.4};
  int cluster_min_points{5};
  int cluster_max_points{100000};
  double max_obstacle_size{8.0};
};
enum class ClusterGeometryStatus
{
  kAccepted, kInvalidGeometry, kTooLarge, kFragment, kInvalidFloatBox
};
struct ClusterGeometry
{
  ClusterGeometryStatus status{ClusterGeometryStatus::kAccepted};
  // xmin,ymin,zmin,xmax,ymax,zmax, then float32 center XYZ / size XYZ.
  std::array<float, 6> bounds{};
  std::array<float, 6> fields{};
};
// Synchronous read-only borrowing; no cloud retained. Parameters are validated by caller.
std::vector<pcl::PointIndices> clusterCloud(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud, const ClusteringParams & params = {});
std::vector<DynamicObstacleObservation> extractCurrentDynamicObservations(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud, const ClusteringParams & params = {});
// Indices must belong to the supplied cloud. Output and tracker paths intentionally
// preserve their original, different float arithmetic and size/fragment gates.
std::array<float, 6> clusterBounds(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointIndices & cluster);
ClusterGeometry outputClusterGeometry(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointIndices & cluster,
  double max_obstacle_size = 8.0);
}  // namespace easyfly::obstacle_geometry
#endif  // EASYFLY_OBSTACLE_GEOMETRY__CLUSTERING_HPP_
