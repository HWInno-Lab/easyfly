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
#include "easyfly_trajectory_optimizer/plan.hpp"
namespace et = easyfly::trajectory;
static et::Grid grid()
{
  et::Grid g; g.size = {30, 30, 18}; g.resolution = .2; g.origin = {-2., -3., 0.};
  g.data.assign(g.size.prod(), 0); return g;
}
TEST(GridContract, UnknownOccupiedAndOutsideAreNeverFree) {
  auto g = grid(); et::Vec p{0., 0., 1.2};
  EXPECT_TRUE(g.valid()); EXPECT_TRUE(g.free(p));
  g.data[g.index(g.cell(p))] = -1; EXPECT_FALSE(g.free(p));
  EXPECT_FALSE(g.free(et::Vec{100., 0., 1.2}));
  g.data[0] = 1; EXPECT_FALSE(g.valid());
}
TEST(GridContract, DiagonalCannotCutAnOccupiedCorner) {
  auto g = grid(); g.data.assign(g.data.size(), 100);
  g.data[g.index(et::Cell{4, 4, 4})] = 0; g.data[g.index(et::Cell{5, 5, 4})] = 0;
  EXPECT_THROW(
    et::search(
      g, g.center({4, 4, 4}), g.center({5, 5, 4}),
      et::Steady::now() + std::chrono::seconds(1)), std::runtime_error);
}
TEST(GridContract, DeadlineIsEnforcedBeforeAnExecutablePath) {
  auto g = grid(); EXPECT_THROW(
    et::search(
      g, {0., 0., 1.2}, {2., 0., 1.2},
      et::Steady::now()), std::runtime_error);
}
TEST(Gcopter, ActualOptimizerPreservesEndpointsAndStrictDynamics) {
  et::Request r; r.grid = grid(); r.start = {0., 0., 1.2}; r.velocity = {.02, 0., 0.};
  r.goal = {2., 0., 1.2};
  auto p = et::optimize(r, et::Steady::now() + std::chrono::seconds(2));
  EXPECT_GT(p.trajectory.getPieceNum(), 0); EXPECT_TRUE(std::isfinite(p.cost));
  EXPECT_LT((p.trajectory.getPos(0) - r.start).norm(), 1e-5);
  EXPECT_LT((p.trajectory.getVel(0) - r.velocity).norm(), 1e-4);
  EXPECT_LT((p.trajectory.getPos(p.trajectory.getTotalDuration()) - r.goal).norm(), 1e-5);
  EXPECT_LE(p.maximum_speed, r.max_speed); EXPECT_LE(p.maximum_acceleration, r.max_acceleration);
  EXPECT_LE(p.maximum_jerk, r.max_jerk); EXPECT_TRUE(et::collisionFree(r.grid, p.trajectory));
}
TEST(Gcopter, ActualOptimizerClimbsOverAFullWidthWall) {
  et::Request r; r.grid = grid(); r.start = {0., 0., 1.2}; r.velocity.setZero();
  r.goal = {2., 0., 1.2};
  for (int z = 0; z < 9; ++z) {
    for (int y = 0; y < r.grid.size.y(); ++y) {
      r.grid.data[r.grid.index({15, y, z})] = 100;
    }
  }
  auto p = et::optimize(r, et::Steady::now() + std::chrono::seconds(2));
  bool high = false;
  for (double t = 0; t < p.trajectory.getTotalDuration(); t += .02) {
    if (std::abs(p.trajectory.getPos(t).x() - 1.1) < .15) {
      high |= p.trajectory.getPos(t).z() >= 1.8;
    }
  }
  EXPECT_TRUE(high); EXPECT_TRUE(et::collisionFree(r.grid, p.trajectory));
}
TEST(Gcopter, RecordedFuelViewpointKeepsMeasuredVelocityAndFitsControllerDynamics) {
  const auto r = et::readRequest(TRACKING_REQUEST_FROM_TEST);
  const auto p = et::optimize(r, et::Steady::now() + std::chrono::seconds(2));
  EXPECT_LT((p.trajectory.getPos(0) - r.start).norm(), 1e-5);
  EXPECT_LT((p.trajectory.getVel(0) - r.velocity).norm(), 1e-4);
  EXPECT_LT((p.trajectory.getPos(p.trajectory.getTotalDuration()) - r.goal).norm(), 1e-5);
  EXPECT_LE(p.maximum_speed, r.max_speed);
  EXPECT_LE(p.maximum_acceleration, et::kTrackingAcceleration);
  EXPECT_LE(p.maximum_jerk, et::kTrackingJerk);
  EXPECT_TRUE(et::collisionFree(r.grid, p.trajectory));
}
TEST(ContinuousValidation, RejectsAnObstacleBetweenEndpointSamples) {
  auto g = grid(); g.data[g.index(g.cell(et::Vec{1., 0., 1.2}))] = 100;
  Eigen::Matrix<double, 3, 6> c = Eigen::Matrix<double, 3, 6>::Zero();
  c(0, 4) = 2.; c(2, 5) = 1.2;
  Trajectory<5> trajectory; trajectory.emplace_back(1., c);
  EXPECT_TRUE(g.free(trajectory.getPos(0))); EXPECT_TRUE(g.free(trajectory.getPos(1.)));
  EXPECT_FALSE(et::collisionFree(g, trajectory));
}
TEST(Gcopter, NarrowCorridorReservesInteriorWithoutLosingMeasuredEndpoints) {
  et::Request r; r.grid = grid(); r.start = {2.37, .008, 1.43};
  r.goal = {-.013, .009, 1.44}; r.velocity = {.06, -.005, .006};
  for (int z = 0; z < r.grid.size.z(); ++z) {
    for (int y = 0; y < r.grid.size.y(); ++y) {
      for (int x = 0; x < r.grid.size.x(); ++x) {
        if (y < 14 || y > 15) {r.grid.data[r.grid.index({x, y, z})] = 100;}}
    }
  }
  auto p = et::optimize(r, et::Steady::now() + std::chrono::seconds(2));
  EXPECT_TRUE(et::collisionFree(r.grid, p.trajectory));
  for (const auto & box:p.boxes) {
    EXPECT_GT(box.low.y(), -.2); EXPECT_LT(box.high.y(), .2);
  }
  EXPECT_LT((p.trajectory.getPos(0) - r.start).norm(), 1e-5);
  EXPECT_LT((p.trajectory.getPos(p.trajectory.getTotalDuration()) - r.goal).norm(), 1e-5);
}
TEST(GridContract, FullyObservedWallHasNoThreeDimensionalRoute) {
  auto g = grid();
  for (int z = 0; z < g.size.z(); ++z) {
    for (int y = 0; y < g.size.y(); ++y) {
      g.data[g.index({15, y, z})] = 100;
    }
  }
  EXPECT_THROW(
    et::search(
      g, {0., 0., 1.3}, {2., 0., 1.3},
      et::Steady::now() + std::chrono::seconds(1)), std::runtime_error);
}

TEST(Gcopter, LongCorridorRetainsObservedEndpointBrakingRoom) {
  auto g = grid();
  const et::Vec start{2.97, 0.01, 1.3}, goal{-0.83, 0., 1.3};
  auto route = et::search(g, start, goal, et::Steady::now() + std::chrono::seconds(1));
  auto boxes = et::corridors(g, route, et::Steady::now() + std::chrono::seconds(1));
  EXPECT_GT(boxes.front().high.x(), start.x() + .15);
  for (const auto & box:boxes) {
    EXPECT_TRUE(g.freeBox(g.cell(box.low), g.cell(box.high)));
  }
}
