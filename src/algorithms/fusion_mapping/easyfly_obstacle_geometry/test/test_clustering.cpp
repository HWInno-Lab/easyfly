// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "easyfly_obstacle_geometry/clustering.hpp"

namespace easyfly::obstacle_geometry
{
namespace
{
pcl::PointCloud<pcl::PointXYZI>::Ptr scene()
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (float origin : {0.F, 3.F}) {
    for (int x = 0; x < 3; ++x) {
      for (int z = 0; z < 3; ++z) {
        pcl::PointXYZI p;
        p.x = origin + x * .1F; p.y = 0.F; p.z = .5F + z * .1F; p.intensity = 7.F;
        cloud->push_back(p);
      }
    }
  }
  return cloud;
}
TEST(Clustering, SeparatesCurrentObjectsWithoutMutatingCloud)
{
  auto cloud = scene();
  const auto before = *cloud;
  auto clusters = clusterCloud(cloud);
  ASSERT_EQ(clusters.size(), 2U);
  auto observations = extractCurrentDynamicObservations(cloud);
  ASSERT_EQ(observations.size(), 2U);
  EXPECT_NEAR(std::abs(observations[1].center_x - observations[0].center_x), 3., 1e-6);
  for (const auto & c : clusters) {
    ASSERT_EQ(c.indices.size(), 9U);
    const auto geometry = outputClusterGeometry(*cloud, c);
    EXPECT_EQ(geometry.status, ClusterGeometryStatus::kAccepted);
    EXPECT_NEAR(geometry.fields[5], .2, 1e-6);
  }
  ASSERT_EQ(cloud->size(), before.size());
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    EXPECT_EQ(cloud->points[i].x, before.points[i].x);
    EXPECT_EQ(cloud->points[i].intensity, 7.F);
  }
}
TEST(Clustering, NullAndUndersizedTrackerInputIsAnEmptyObservation)
{
  EXPECT_TRUE(extractCurrentDynamicObservations(nullptr).empty());
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  EXPECT_TRUE(extractCurrentDynamicObservations(cloud).empty());
  cloud->push_back(pcl::PointXYZI{});
  EXPECT_TRUE(extractCurrentDynamicObservations(cloud).empty());
}
TEST(Clustering, PclSizeGatesAndGeometryGatesRemainSeparate)
{
  auto cloud = scene();
  ClusteringParams params;
  params.cluster_min_points = 10;
  EXPECT_TRUE(clusterCloud(cloud, params).empty());
  params.cluster_min_points = 5;
  params.cluster_max_points = 8;
  EXPECT_TRUE(clusterCloud(cloud, params).empty());
  params.cluster_max_points = 100;
  params.max_obstacle_size = .1;
  EXPECT_EQ(clusterCloud(cloud, params).size(), 2U);
  EXPECT_TRUE(extractCurrentDynamicObservations(cloud, params).empty());
  auto clusters = clusterCloud(cloud);
  EXPECT_EQ(outputClusterGeometry(*cloud, clusters[0], .1).status, ClusterGeometryStatus::kTooLarge);
  for (auto & p : cloud->points) {p.z = .5F;}
  EXPECT_TRUE(extractCurrentDynamicObservations(cloud).empty());
  EXPECT_EQ(outputClusterGeometry(*cloud, clusters[0]).status, ClusterGeometryStatus::kFragment);
}
TEST(Clustering, OutputRetainsFloatArithmeticAndRejectsSizeOverflow)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI a, b;
  a.x = -.1F; b.x = .3F; a.y = b.y = 0.F; a.z = .5F; b.z = .7F;
  cloud.push_back(a); cloud.push_back(b);
  pcl::PointIndices indices;
  indices.indices = {0, 1};
  auto g = outputClusterGeometry(cloud, indices);
  ASSERT_EQ(g.status, ClusterGeometryStatus::kAccepted);
  EXPECT_EQ(g.fields[0], static_cast<float>((a.x + b.x) / 2.0));
  EXPECT_EQ(g.fields[3], b.x - a.x);
  const double widened_size = static_cast<double>(b.x) - a.x;
  EXPECT_NE(static_cast<double>(g.fields[3]), widened_size);
  cloud[0].x = -std::numeric_limits<float>::max();
  cloud[1].x = std::numeric_limits<float>::max();
  EXPECT_EQ(outputClusterGeometry(cloud, indices, 1e300).status,
    ClusterGeometryStatus::kInvalidGeometry);
}
TEST(Clustering, OutputSentinelAndEmptyIndexPolicyPreserved)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI p; p.x = 2e9F; p.y = 0.F; p.z = .5F;
  cloud.push_back(p);
  pcl::PointIndices indices; indices.indices = {0};
  const auto bounds = clusterBounds(cloud, indices);
  EXPECT_EQ(bounds[0], 1e9F);  // Historical output sentinel, intentionally unchanged.
  EXPECT_EQ(bounds[3], 2e9F);
  indices.indices.clear();
  EXPECT_EQ(outputClusterGeometry(cloud, indices).status, ClusterGeometryStatus::kInvalidGeometry);
}
}  // namespace
}  // namespace easyfly::obstacle_geometry
