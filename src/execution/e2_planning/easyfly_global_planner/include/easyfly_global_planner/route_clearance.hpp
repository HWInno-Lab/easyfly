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
#pragma once
#include <cmath>
#include <vector>
#include "easyfly_astar/astar.hpp"
namespace easyfly_global_planner
{
// Move intermediate corners outwards, leaving both requested endpoints intact.
// Every candidate segment is checked against the original conservative grid.
// A narrow passage retains its original path; no occupied cell is cleared.
inline std::vector<easyfly::grid_planning::Point> widen_route_corners(
  const easyfly::astar::OccupancyGrid2d & grid,
  const std::vector<easyfly::grid_planning::Point> & path, double margin,
  const easyfly::astar::SearchControl & control = {}, double * applied_margin = nullptr)
{
  if (applied_margin) {*applied_margin = 0.;}
  easyfly::astar::detail::SearchContext search{control};
  search.poll();  // Propagate interruption, never return a partially conditioned path.
  if (path.size() < 3 || !std::isfinite(margin) || margin <= 0) {return path;}
  for (int attempt = 0; attempt < 4; ++attempt, margin *= 0.5) {
    auto candidate = path;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      search.poll();
      const double ax = path[i].x - path[i - 1].x, ay = path[i].y - path[i - 1].y;
      const double bx = path[i + 1].x - path[i].x, by = path[i + 1].y - path[i].y;
      const double a = std::hypot(ax, ay), b = std::hypot(bx, by);
      if (a < 1e-8 || b < 1e-8) {continue;}
      const double x = ax / a - bx / b, y = ay / a - by / b;
      const double norm = std::hypot(x, y);
      if (norm < 1e-6) {continue;}
      candidate[i].x += margin * x / norm;
      candidate[i].y += margin * y / norm;
    }
    bool free = true;
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      search.poll();
      const auto p = candidate[i];
      if (grid.worldOccupied(p.x, p.y) || (i && !easyfly::astar::detail::lineOfSight(
          grid, candidate[i - 1].x, candidate[i - 1].y, p.x, p.y, &search)))
      {
        free = false;
        break;
      }
    }
    search.poll();
    if (free) {
      if (applied_margin) {*applied_margin = margin;}
      return candidate;
    }
  }
  search.poll();
  return path;
}
}  // namespace easyfly_global_planner
