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
#include <atomic>
#include <stdexcept>
#include "easyfly_global_planner/adapters/planner_factory.hpp"
#include "easyfly_global_planner/astar.hpp"
namespace gp = easyfly::grid_planning;
TEST(AStarAdapter, NativePathAndIdentityArePreserved)
{
  easyfly_global_planner::OccupancyGrid2d grid;
  grid.width = grid.height = 40;
  grid.resolution = .2;
  grid.origin_x = grid.origin_y = -4;
  grid.data.assign(1600, 0);
  for (int row = 7; row < 33; ++row) {
    grid.data[row * 40 + 20] = 100;
  }
  auto planner = easyfly_global_planner::make_planner("astar");
  for (int i = 0; i < 30; ++i) {
    gp::Request request;
    request.grid = {40, 40, .2, -4, -4, grid.data.data(), grid.data.size()};
    request.map = {7, static_cast<std::uint64_t>(i), 1500000000};
    request.start = {-3.13, -2.5 + i * .1};
    request.goal = {3.17, 2.51 - i * .1};
    const auto native = easyfly_global_planner::planPath(
      grid,
      request.start.x, request.start.y, request.goal.x, request.goal.y);
    const auto output = planner->plan(request, {});
    ASSERT_TRUE(native.ok);
    ASSERT_EQ(output.status, gp::Status::ok);
    EXPECT_EQ(output.map.epoch, 7u);
    EXPECT_EQ(output.map.revision, request.map.revision);
    EXPECT_EQ(output.map.source_stamp_ns, 1500000000);
    EXPECT_EQ(output.expansions, native.expansions);
    ASSERT_EQ(output.waypoints.size(), native.waypoints.size());
    for (std::size_t j = 0; j < output.waypoints.size(); ++j) {
      EXPECT_DOUBLE_EQ(output.waypoints[j].x, native.waypoints[j].first);
      EXPECT_DOUBLE_EQ(output.waypoints[j].y, native.waypoints[j].second);
    }
  }
}
TEST(AStarAdapter, ControlsAndErrorsNeverReturnAnExecutablePartialPath)
{
  std::vector<std::int8_t> cells(10000, 0);
  gp::Request request;
  request.grid = {100, 100, .2, -10, -10, cells.data(), cells.size()};
  request.start = {-5, 0}; request.goal = {5, 0};
  auto planner = easyfly_global_planner::make_planner("astar");
  EXPECT_TRUE(planner->capabilities().cancellation);
  gp::SearchControl control;
  control.expansion_limit = 7;
  auto result = planner->plan(request, control);
  EXPECT_EQ(result.status, gp::Status::budget_exhausted);
  EXPECT_EQ(result.expansions, 7u);
  EXPECT_TRUE(result.waypoints.empty());
  std::atomic_bool cancel{true};control.cancel_requested = &cancel;
  EXPECT_EQ(planner->plan(request, control).status, gp::Status::canceled);
  cancel = false; control.deadline = gp::SearchControl::Clock::now();
  EXPECT_EQ(planner->plan(request, control).status, gp::Status::timed_out);
  EXPECT_EQ(planner->plan(request, {}).status, gp::Status::ok);
  request.grid.cell_count--;
  EXPECT_EQ(planner->plan(request, {}).status, gp::Status::invalid_input);
  request.grid.cell_count++;request.goal.x = 10;
  EXPECT_EQ(planner->plan(request, {}).status, gp::Status::out_of_bounds);
  request.goal.x = NAN;
  EXPECT_EQ(planner->plan(request, {}).status, gp::Status::invalid_input);
}
TEST(AStarAdapter, ExplicitUnknownThresholdAndResourcePolicy)
{
  std::vector<std::int8_t> cells(100, -1);
  gp::Request request;
  request.grid = {10, 10, 1, 0, 0, cells.data(), cells.size()};
  request.start = {1.1, 1.1};request.goal = {8.1, 8.1};
  auto planner = easyfly_global_planner::make_planner("astar");
  EXPECT_EQ(planner->plan(request, {}).status, gp::Status::ok);
  request.grid.unknown_policy = gp::UnknownPolicy::occupied;
  EXPECT_EQ(planner->plan(request, {}).status, gp::Status::start_in_obstacle);
  cells.assign(100, 60);request.grid.cells = cells.data();
  request.grid.occupied_threshold = 70;
  EXPECT_EQ(planner->plan(request, {}).status, gp::Status::ok);
  request.grid.occupied_threshold = 50;
  EXPECT_NE(planner->plan(request, {}).status, gp::Status::ok);
  planner = easyfly_global_planner::make_planner("astar", {99, 5});
  EXPECT_EQ(planner->plan(request, {}).status, gp::Status::resource_limit);
}
TEST(AStarAdapter, UnknownBackendAndInvalidParametersFailExplicitly)
{
  EXPECT_THROW(easyfly_global_planner::make_planner("rrt"), std::invalid_argument);
  EXPECT_THROW(easyfly_global_planner::make_planner("astar", {0, 5}), std::invalid_argument);
  EXPECT_THROW(easyfly_global_planner::make_planner("astar", {100, -1}), std::invalid_argument);
}
