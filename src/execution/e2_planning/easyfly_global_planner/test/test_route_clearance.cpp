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
#include <chrono>
#include <fstream>
#include "easyfly_global_planner/route_clearance.hpp"
using easyfly::astar::OccupancyGrid2d;
using easyfly::grid_planning::Point;
using easyfly_global_planner::widen_route_corners;
static void expect_free(const OccupancyGrid2d & g, const std::vector<Point> & p)
{
  for (std::size_t i = 0; i < p.size(); ++i) {
    EXPECT_FALSE(g.worldOccupied(p[i].x, p[i].y));
    if (i) {
      EXPECT_TRUE(
        easyfly::astar::detail::lineOfSight(
          g, p[i - 1].x, p[i - 1].y, p[i].x, p[i].y));
    }
  }
}
TEST(RouteClearance, RecordedShortCornerCanBeTraversedWithoutClearingCells)
{
  OccupancyGrid2d g;
  Point start, goal;
  std::ifstream input(RECORDED_CORNER_GRID);
  ASSERT_TRUE(input.good());
  input >> g.width >> g.height >> g.resolution >> g.origin_x >> g.origin_y
  >> start.x >> start.y >> goal.x >> goal.y;
  for (int i = 0; i < g.width * g.height; ++i) {
    int value;
    ASSERT_TRUE(static_cast<bool>(input >> value));
    g.data.push_back(static_cast<int8_t>(value));
  }
  // Corner from the actual preceding return route. Its next segment ceased
  // to be visible from the measured stop position; EGO refused the <20cm leg.
  const std::vector<Point> route{start, {0.700000755, -1.099999271}, goal};
  ASSERT_LT(std::hypot(route[1].x - start.x, route[1].y - start.y), 0.2);
  ASSERT_FALSE(easyfly::astar::detail::lineOfSight(g, start.x, start.y, goal.x, goal.y));
  const auto original_cells = g.data;
  const auto widened = widen_route_corners(g, route, 2 * g.resolution);
  ASSERT_EQ(widened.size(), route.size());
  EXPECT_GT(std::hypot(widened[1].x - start.x, widened[1].y - start.y), 0.2);
  EXPECT_DOUBLE_EQ(widened.front().x, start.x);
  EXPECT_DOUBLE_EQ(widened.front().y, start.y);
  EXPECT_DOUBLE_EQ(widened.back().x, goal.x);
  EXPECT_DOUBLE_EQ(widened.back().y, goal.y);
  EXPECT_EQ(g.data, original_cells);
  expect_free(g, widened);
}
TEST(RouteClearance, StraightRoutesAndExactEndpointsStayUnchanged)
{
  OccupancyGrid2d g{10, 10, 1., 0., 0., std::vector<int8_t>(100, 0)};
  const std::vector<Point> straight{{1.1, 2.2}, {2.1, 2.2}, {3.1, 2.2}};
  const auto output = widen_route_corners(g, straight, 0.4);
  for (std::size_t i = 0; i < straight.size(); ++i) {
    EXPECT_DOUBLE_EQ(straight[i].x, output[i].x);
    EXPECT_DOUBLE_EQ(straight[i].y, output[i].y);
  }
  const std::vector<Point> direct{straight.front(), straight.back()};
  const auto no_corner = widen_route_corners(g, direct, 0.4);
  EXPECT_DOUBLE_EQ(direct.back().x, no_corner.back().x);
  EXPECT_DOUBLE_EQ(direct.back().y, no_corner.back().y);
}
TEST(RouteClearance, InsufficientSpaceKeepsOriginalSafeRoute)
{
  OccupancyGrid2d g{10, 10, 1., 0., 0., std::vector<int8_t>(100, 100)};
  for (int x = 1; x <= 5; ++x) {
    g.data[10 + x] = 0;
  }
  for (int y = 1; y <= 6; ++y) {
    g.data[y * 10 + 5] = 0;
  }
  const std::vector<Point> route{{1.5, 1.000001}, {5.999999, 1.000001}, {5.999999, 6.5}};
  expect_free(g, route);
  const auto output = widen_route_corners(g, route, 2.);
  for (std::size_t i = 0; i < route.size(); ++i) {
    EXPECT_DOUBLE_EQ(route[i].x, output[i].x);
    EXPECT_DOUBLE_EQ(route[i].y, output[i].y);
  }
  expect_free(g, output);
}
TEST(RouteClearance, ExpiredPlanningBudgetCannotReturnAnExecutableRoute)
{
  OccupancyGrid2d g{10, 10, 1., 0., 0., std::vector<int8_t>(100, 0)};
  const std::vector<Point> route{{1., 1.}, {3., 1.}, {3., 3.}};
  easyfly::astar::SearchControl control;
  control.deadline = easyfly::astar::SearchControl::Clock::now() - std::chrono::seconds(1);
  EXPECT_THROW(widen_route_corners(g, route, 0.4, control), easyfly::astar::AStarResult::Fail);
}
TEST(RouteClearance, RecordedTrackingOffsetNeedsMoreThanTwoCellsOfReserve)
{
  OccupancyGrid2d g;
  Point start, goal;
  std::ifstream input(RECORDED_TRACKING_GRID);
  ASSERT_TRUE(input.good());
  input >> g.width >> g.height >> g.resolution >> g.origin_x >> g.origin_y
  >> start.x >> start.y >> goal.x >> goal.y;
  for (int i = 0; i < g.width * g.height; ++i) {
    int value;
    ASSERT_TRUE(static_cast<bool>(input >> value));
    g.data.push_back(static_cast<int8_t>(value));
  }
  const auto original = easyfly::astar::planPath(g, start.x, start.y, goal.x, goal.y, 0);
  ASSERT_TRUE(original.ok);
  std::vector<Point> route;
  for (auto p : original.waypoints) {
    route.push_back({p.first, p.second});
  }
  ASSERT_EQ(route.size(), 4u);
  const auto tight = widen_route_corners(g, route, 2 * g.resolution);
  double applied = 0.;
  const auto reserved = widen_route_corners(g, route, 1.0 + g.resolution, {}, &applied);
  ASSERT_GE(applied, 1.0);
  expect_free(g, reserved);
  OccupancyGrid2d updated;
  Point ignored_start, ignored_goal;
  std::ifstream later(RECORDED_TRACKING_UPDATE);
  later >> updated.width >> updated.height >> updated.resolution >> updated.origin_x
  >> updated.origin_y >> ignored_start.x >> ignored_start.y >> ignored_goal.x >> ignored_goal.y;
  for (int i = 0; i < updated.width * updated.height; ++i) {
    int value;
    ASSERT_TRUE(static_cast<bool>(later >> value));
    updated.data.push_back(static_cast<int8_t>(value));
  }
  // Measured VIO displacement from the preceding planned crossing, including
  // reference mismatch and execution deviation. Query the actual later map;
  // this regression never rewrites runtime measurements.
  const Point observed{2.5435593568089563, -0.5396316840070624};
  const double vx = tight[2].x - tight[1].x, vy = tight[2].y - tight[1].y;
  const double fraction = ((observed.x - tight[1].x) * vx +
    (observed.y - tight[1].y) * vy) / (vx * vx + vy * vy);
  const double dx = observed.x - (tight[1].x + fraction * vx);
  const double dy = observed.y - (tight[1].y + fraction * vy);
  EXPECT_FALSE(
    easyfly::astar::detail::lineOfSight(
      updated, tight[1].x + dx, tight[1].y + dy, tight[2].x + dx, tight[2].y + dy));
  EXPECT_TRUE(
    easyfly::astar::detail::lineOfSight(
      updated, reserved[1].x + dx, reserved[1].y + dy, reserved[2].x + dx, reserved[2].y + dy));
  EXPECT_DOUBLE_EQ(route.back().x, reserved.back().x);
  EXPECT_DOUBLE_EQ(route.back().y, reserved.back().y);
}
