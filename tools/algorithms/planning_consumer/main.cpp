// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <stdexcept>
#include "easyfly_astar/astar.hpp"
#include "easyfly_coverage_planning/search_geometry.hpp"
#include "easyfly_global_planner/adapters/planner_factory.hpp"
#include "easyfly_global_planner/astar.hpp"
namespace gp = easyfly::grid_planning;
void require(bool condition) {if (!condition) {throw std::runtime_error("installed planning check failed");}}
int main()
{
  std::vector<std::int8_t> cells(400, 0);
  gp::Request request;
  request.grid = {20, 20, .5, -5, -5, cells.data(), cells.size()};
  request.start = {-3.17, -.13}; request.goal = {3.23, .17};
  request.map = {8, 19, 2000000000};
  auto planner = easyfly_global_planner::make_planner("astar");
  auto result = planner->plan(request, {});
  require(result.status == gp::Status::ok && result.waypoints.size() == 2);
  require(result.map.epoch == 8 && result.map.revision == 19 && result.map.source_stamp_ns == 2000000000);
  require(result.waypoints.front().x == -3.17 && result.waypoints.back().x == 3.23);
  gp::SearchControl control; control.expansion_limit = 2;
  result = planner->plan(request, control);
  require(result.status == gp::Status::budget_exhausted && result.expansions == 2 && result.waypoints.empty());
  std::atomic_bool canceled{true}; control.cancel_requested = &canceled;
  require(planner->plan(request, control).status == gp::Status::canceled);
  canceled = false; control.deadline = gp::SearchControl::Clock::now();
  require(planner->plan(request, control).status == gp::Status::timed_out);
  require(planner->plan(request, {}).status == gp::Status::ok);
  easyfly_global_planner::OccupancyGrid2d legacy;
  legacy.width = legacy.height = 20; legacy.resolution = .5;
  legacy.origin_x = legacy.origin_y = -5; legacy.data = cells;
  require(easyfly_global_planner::planPath(legacy, -3.17, -.13, 3.23, .17).ok);
  const auto route = easyfly::coverage::searchRoute(0, 0, 4, 3, 1.5, 2, {});
  require(route.size() == 4 && route.front().completed_area == 0 && route.back().completed_area == 12);
  bool rejected = false;
  try {planner = easyfly_global_planner::make_planner("rrt");}
  catch (const std::invalid_argument &) {rejected = true;}
  require(rejected);
}
