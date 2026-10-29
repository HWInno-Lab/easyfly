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
#include <cstddef>
#include <nav_msgs/msg/occupancy_grid.hpp>
namespace easyfly_global_planner
{
inline constexpr char kWorldFrame[] = "odom";
// semantic_map reset protocol: zero stamp + all UNKNOWN cells is explicitly
// not ready.  A valid map has a non-zero stamp and at least one known cell;
// unknown cells may still occur inside an otherwise valid map.
inline bool occupancyGridReady(const nav_msgs::msg::OccupancyGrid & grid)
{
  if (grid.header.stamp.sec == 0 && grid.header.stamp.nanosec == 0) {return false;}
  if (grid.header.frame_id != kWorldFrame ||
    !std::isfinite(grid.info.resolution) || grid.info.resolution <= 0.0f ||
    !std::isfinite(grid.info.origin.position.x) ||
    !std::isfinite(grid.info.origin.position.y) ||
    grid.info.width == 0 || grid.info.height == 0)
  {
    return false;
  }
  const auto expected = static_cast<std::size_t>(grid.info.width) * grid.info.height;
  if (grid.data.size() != expected) {return false;}
  for (const auto cell : grid.data) {
    if (cell != -1) {return true;}
  }
  return false;
}
}  // namespace easyfly_global_planner
