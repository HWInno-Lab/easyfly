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
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <easyfly_msgs/msg/voxel_grid.hpp>

namespace easyfly_global_planner
{
struct LocalVoxelTarget {double x, y; bool partial;};
inline bool observedVoxelEndpoint(
  const easyfly_msgs::msg::VoxelGrid & map,
  double x, double y, double z)
{
  const auto res = map.resolution, radius = map.inflation_radius;
  if (!map.valid || !std::isfinite(res) || res < .05 || res > 1. ||
    !std::isfinite(radius) || radius < .65 || radius > 1.) {return false;}
  const std::array<double, 3> origin{map.origin.x, map.origin.y, map.origin.z}, point{x, y, z};
  const std::array<int, 3> size{static_cast<int>(map.size_x), static_cast<int>(map.size_y),
    static_cast<int>(map.size_z)};
  std::array<int, 3> cell;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(point[i]) || !std::isfinite(origin[i]) || size[i] < 1 || size[i] > 512 ||
      point[i] < origin[i] || point[i] >= origin[i] + size[i] * res) {return false;}
    cell[i] = static_cast<int>(std::floor((point[i] - origin[i]) / res));
  }
  if (uint64_t(size[0]) * size[1] * size[2] != map.data.size() || map.data.size() > 1000000) {
    return false;
  }
  const auto index = [&size](int cx, int cy, int cz) {return cx + size[0] * (cy + size[1] * cz);};
  if (map.data[index(cell[0], cell[1], cell[2])] != 0) {return false;}
  const int extent = static_cast<int>(std::ceil(radius / res)) + 1;
  for (int dz = -extent; dz <= extent; ++dz) {
    for (int dy = -extent; dy <= extent; ++dy) {
      for (int dx = -extent; dx <= extent; ++dx) {
        const int cx = cell[0] + dx, cy = cell[1] + dy, cz = cell[2] + dz;
        if (cx < 0 || cx >= size[0] || cy < 0 || cy >= size[1] || cz < 0 || cz >= size[2] ||
          map.data[index(cx, cy, cz)] == -1) {return false;}
      }
    }
  }
  return true;
}
inline std::optional<LocalVoxelTarget> rollingTarget(
  const easyfly_msgs::msg::VoxelGrid & map,
  double start_x, double start_y, double goal_x, double goal_y, double z, double step)
{
  if (!std::isfinite(start_x) || !std::isfinite(start_y) || !std::isfinite(goal_x) ||
    !std::isfinite(goal_y) || !std::isfinite(step) || step < .5 || step > 8.) {return {};}
  const double dx = goal_x - start_x, dy = goal_y - start_y, distance = std::hypot(dx, dy);
  if (distance <= step + .35 && observedVoxelEndpoint(map, goal_x, goal_y, z)) {
    return LocalVoxelTarget{goal_x, goal_y, false};
  }
  // This only chooses a progress endpoint in observed space. GCOPTER must still
  // solve and continuously certify the complete route, including any detour.
  for (double advance = std::min(step, distance - .35); advance >= .5; advance -= .2) {
    const double x = start_x + dx * advance / distance, y = start_y + dy * advance / distance;
    if (observedVoxelEndpoint(map, x, y, z)) {return LocalVoxelTarget{x, y, true};}
  }
  return {};
}
}  // namespace easyfly_global_planner
