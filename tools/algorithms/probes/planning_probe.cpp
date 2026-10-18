// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#ifdef EASYFLY_LEGACY_PLANNING
#include "easyfly_global_planner/astar.hpp"
#include "easyfly_mission_fsm/search_geometry.hpp"
namespace astar = easyfly_global_planner;
namespace coverage = easyfly_mission_fsm;
#else
#include "easyfly_astar/astar.hpp"
#include "easyfly_coverage_planning/search_geometry.hpp"
namespace astar = easyfly::astar;
namespace coverage = easyfly::coverage;
#endif
int main()
{
  std::cout << std::hexfloat;
  std::uint32_t seed = 187;
  for (int scenario = 0; scenario < 200; ++scenario) {
    astar::OccupancyGrid2d grid;
    grid.width = grid.height = 32;
    grid.resolution = .2;
    grid.origin_x = grid.origin_y = -3.2;
    grid.data.resize(1024);
    for (auto & cell : grid.data) {
      seed = seed * 1664525u + 1013904223u;
      cell = seed % 100 < 18 ? 100 : (seed % 11 == 0 ? -1 : 0);
    }
    grid.data[3 * 32 + 3] = grid.data[28 * 32 + 28] = 0;
    const auto result = astar::planPath(grid, -2.55, -2.51, 2.49, 2.51);
    std::cout << scenario << ' ' << result.ok << ' ' << static_cast<int>(result.fail);
    for (auto point : result.waypoints) {std::cout << ' ' << point.first << ' ' << point.second;}
    std::cout << '\n';
  }
  for (int i = 1; i <= 100; ++i) {
    try {
      const auto route = coverage::searchRoute(0, 0, i * .5, i * .4, 1.5, .2 + i * .02, {});
      std::cout << "coverage " << i;
      for (auto point : route) {
        std::cout << ' ' << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.completed_area;
      }
      std::cout << '\n';
    } catch (const std::invalid_argument & e) {std::cout << "reject " << i << ' ' << e.what() << '\n';}
  }
}
