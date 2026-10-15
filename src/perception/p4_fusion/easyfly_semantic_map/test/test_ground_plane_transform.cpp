// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_semantic_map/ground_plane_transform.hpp"
namespace easyfly_semantic_map
{
TEST(GroundPlaneTransform, OdomHeightChangeTransportsPlaneWithoutRefitting)
{
  GroundPlaneTracker tracker;
  GroundPlaneCandidate candidate;
  candidate.inlier_count = candidate.sample_count = 500;
  GroundVehiclePose vehicle;
  vehicle.connected = vehicle.pose_valid = true; vehicle.armed = false;
  vehicle.state_age_s = 0; vehicle.z = 0.054;
  for (int i = 0; i < 3; ++i) {tracker.update(candidate, vehicle);}
  ASSERT_TRUE(tracker.initialized());
  vehicle.armed = true;
  candidate.model.offset = -1.0;
  EXPECT_EQ(tracker.update(candidate, vehicle), GroundPlaneUpdate::kFrozen);
  auto basis = Eigen::Isometry3d::Identity();
  basis.translation().z() = 0.3;
  const auto world = groundPlaneInWorld(tracker.model(), basis);
  EXPECT_NEAR(world.signedDistance(2, 1, -0.3), 0.0, 1e-9);
  EXPECT_NEAR(world.signedDistance(2, 1, -0.1), 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(tracker.model().offset, 0.0);
}
TEST(GroundPlaneTransform, RotationAndTranslationPreserveSurfaceAndObstacleDistances)
{
  Eigen::Isometry3d basis = Eigen::Isometry3d::Identity();
  basis.linear() = Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitY()).toRotationMatrix();
  basis.translation() = Eigen::Vector3d(.1, -.2, .3);
  GroundPlaneModel ground;
  const auto world = groundPlaneInWorld(ground, basis);
  for (double z : {0.0, 0.2, 1.0}) {
    const Eigen::Vector3d point = basis.inverse() * Eigen::Vector3d(1.0, 2.0, z);
    EXPECT_NEAR(world.signedDistance(point.x(), point.y(), point.z()), z, 1e-9);
  }
}
TEST(GroundPlaneTransform, BasisJumpsAndInvalidTimesFailClosed)
{
  const auto identity = Eigen::Isometry3d::Identity();
  auto change = Eigen::Isometry3d::Identity();
  change.translation().z() = .02;
  EXPECT_TRUE(groundBasisContinuous(identity, change));
  change.translation().z() = .3;
  EXPECT_FALSE(groundBasisContinuous(identity, change));
  change = identity;
  change.linear() = Eigen::AngleAxisd(.11, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(groundBasisContinuous(identity, change));
  EXPECT_TRUE(groundBasisStampFresh(10.0, 9.9, 1.0));
  EXPECT_FALSE(groundBasisStampFresh(10.0, 8.0, 1.0));
  EXPECT_FALSE(groundBasisStampFresh(10.0, 10.3, 1.0));
  EXPECT_FALSE(groundBasisStampFresh(10.0, 0.0, 1.0));
}
}  // namespace easyfly_semantic_map
