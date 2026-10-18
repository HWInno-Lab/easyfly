// Copyright 2026 EasyFly Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <limits>
#include <vector>
#include "easyfly_ground_segmentation/ground_spatial_support.hpp"

namespace easyfly::ground
{
namespace
{
struct Point {double x, y, z;};
std::vector<Point> floorCloud()
{
  std::vector<Point> cloud;
  for (int x = -30; x <= 30; ++x) {
    for (int y = -30; y <= 30; ++y) {
      if (x * x + y * y >= 100 && x * x + y * y <= 900) {
        cloud.push_back({x * 0.1, y * 0.1, 0.0});
      }
    }
  }
  return cloud;
}
GroundVehiclePose landed()
{
  GroundVehiclePose pose;
  pose.z = 0.054;
  pose.connected = pose.pose_valid = true;
  pose.armed = false;
  pose.state_age_s = 0;
  return pose;
}
GroundPlaneCandidate supported()
{
  GroundPlaneCandidate candidate;
  const auto cloud = floorCloud();
  candidate.spatial_support = groundSpatialSupport(cloud, candidate.model, landed());
  candidate.inlier_count = cloud.size();
  candidate.sample_count = cloud.size() * 20;  // unchanged low global ratio
  return candidate;
}
TEST(GroundSpatialSupport, SurroundingFloorIsIndependentOfCeilingPointCount)
{
  auto cloud = floorCloud();
  const auto size = cloud.size();
  const auto baseline = groundSpatialSupport(cloud, GroundPlaneModel{}, landed());
  for (int i = 0; i < 20000; ++i) {
    cloud.push_back({1.5, 0, 5.0});
  }
  const auto proof = groundSpatialSupport(cloud, GroundPlaneModel{}, landed());
  EXPECT_TRUE(proof.qualified());
  EXPECT_EQ(proof.points, baseline.points);
  EXPECT_EQ(proof.cells, baseline.cells);
  EXPECT_EQ(proof.eligible_cells, baseline.eligible_cells);
  EXPECT_EQ(cloud.size(), size + 20000);  // evidence never filters obstacles
}
TEST(GroundSpatialSupport, DuplicatesCannotCreateIndependentSupport)
{
  const std::vector<Point> cloud(20000, Point{1.5, 0, 0});
  EXPECT_FALSE(groundSpatialSupport(cloud, GroundPlaneModel{}, landed()).qualified());
}
TEST(GroundSpatialSupport, OneSidedFloorOrSmallPlatformCannotInitialize)
{
  auto cloud = floorCloud();
  cloud.erase(
    std::remove_if(
      cloud.begin(), cloud.end(),
      [](const Point & p) {return p.x < 0;}), cloud.end());
  EXPECT_FALSE(groundSpatialSupport(cloud, GroundPlaneModel{}, landed()).qualified());
  EXPECT_FALSE(
    groundSpatialSupport(
      std::vector<Point>(1000, {0.3, 0.3, 0}),
      GroundPlaneModel{}, landed()).qualified());
}
TEST(GroundSpatialSupport, LowOutliersAndStepsCountAgainstConfidence)
{
  auto cloud = floorCloud();
  for (auto & point : cloud) {
    if (point.x > 0) {
      point.z = -0.25;
    }
  }
  EXPECT_FALSE(groundSpatialSupport(cloud, GroundPlaneModel{}, landed()).qualified());
}
TEST(GroundSpatialSupport, NearBodyCeilingTiltAndNonfiniteEvidenceFail)
{
  auto cloud = floorCloud();
  auto vehicle = landed();
  vehicle.z = 0.01;
  EXPECT_FALSE(groundSpatialSupport(cloud, GroundPlaneModel{}, vehicle).qualified());
  vehicle.z = 1.0;
  EXPECT_FALSE(groundSpatialSupport(cloud, GroundPlaneModel{}, vehicle).qualified());
  GroundPlaneModel tilted;
  tilted.normal = {std::sin(0.11), 0, std::cos(0.11)};
  EXPECT_FALSE(groundSpatialSupport(cloud, tilted, landed()).qualified());
  cloud.push_back({std::numeric_limits<double>::quiet_NaN(), 0, 0});
  EXPECT_FALSE(groundSpatialSupport(cloud, GroundPlaneModel{}, landed()).qualified());
}
TEST(GroundSpatialSupport, ExplicitPolicyAndFiveConsistentDisarmedFramesRequired)
{
  const auto candidate = supported();
  ASSERT_TRUE(candidate.spatial_support.qualified());
  GroundPlaneTracker legacy;
  for (int i = 0; i < 6; ++i) {
    legacy.update(candidate, landed());
  }
  EXPECT_FALSE(legacy.initialized());
  GroundPlaneTracker::Params params;
  params.spatial_support_enabled = true;
  GroundPlaneTracker spatial(params);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(spatial.update(candidate, landed()), GroundPlaneUpdate::kPending);
  }
  EXPECT_EQ(spatial.update(candidate, landed()), GroundPlaneUpdate::kInitialized);
  auto vehicle = landed();
  vehicle.armed = true;
  GroundPlaneTracker airborne(params);
  for (int i = 0; i < 6; ++i) {
    airborne.update(candidate, vehicle);
  }
  EXPECT_FALSE(airborne.initialized());
  EXPECT_EQ(spatial.update(candidate, vehicle), GroundPlaneUpdate::kFrozen);
  vehicle.armed = false;
  vehicle.state_age_s = 2.0;
  GroundPlaneTracker stale(params);
  for (int i = 0; i < 6; ++i) {
    stale.update(candidate, vehicle);
  }
  EXPECT_FALSE(stale.initialized());
}
TEST(GroundSpatialSupport, InterruptedSupportResetsConfirmation)
{
  GroundPlaneTracker::Params params;
  params.spatial_support_enabled = true;
  GroundPlaneTracker tracker(params);
  auto candidate = supported();
  for (int i = 0; i < 4; ++i) {
    tracker.update(candidate, landed());
  }
  auto bad = candidate;
  bad.spatial_support.sector_cells[0] = 0;
  EXPECT_EQ(tracker.update(bad, landed()), GroundPlaneUpdate::kRejected);
  for (int i = 0; i < 4; ++i) {
    tracker.update(candidate, landed());
  }
  EXPECT_FALSE(tracker.initialized());
}
TEST(GroundSpatialSupport, TrustedPlaneDoesNotRemoveLowObstacle)
{
  GroundPlaneTracker::Params params;
  params.spatial_support_enabled = true;
  GroundPlaneTracker tracker(params);
  for (int i = 0; i < 5; ++i) {
    tracker.update(supported(), landed());
  }
  ASSERT_TRUE(tracker.initialized());
  EXPECT_FALSE(tracker.isGroundPoint(1.5, 0, 0.2, 0.12));
  EXPECT_TRUE(tracker.isGroundPoint(1.5, 0, 0.0, 0.12));
}
}  // namespace
}  // namespace easyfly::ground
