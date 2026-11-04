// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#pragma once
#include <algorithm>
#include <cstdint>
#include <string>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <easyfly_msgs/msg/obstacle_info_array.hpp>
namespace easyfly_semantic_map
{
// Map reset protocol shared with consumers:
//   * /obstacles: empty array, zero header stamp;
//   * /occupancy_grid: valid dimensions but every cell is UNKNOWN (-1), zero
//     header stamp.
// A valid semantic map always has a non-zero stamp and at least one known grid
// cell (free=0 or occupied=100).  Thus a reset is not mistaken for free space.
inline bool zeroStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}
inline nav_msgs::msg::OccupancyGrid makeInvalidGrid(
  const double width_m, const double height_m, const double resolution,
  const std::string & frame_id)
{
  nav_msgs::msg::OccupancyGrid grid;
  const double safe_resolution = std::max(1e-3, resolution);
  grid.header.frame_id = frame_id;
  grid.info.resolution = static_cast<float>(safe_resolution);
  grid.info.width = static_cast<uint32_t>(std::max(1.0, width_m / safe_resolution));
  grid.info.height = static_cast<uint32_t>(std::max(1.0, height_m / safe_resolution));
  grid.info.origin.position.x = -width_m / 2.0;
  grid.info.origin.position.y = -height_m / 2.0;
  grid.info.origin.orientation.w = 1.0;
  const auto cells = static_cast<std::size_t>(grid.info.width) * grid.info.height;
  grid.data.assign(cells, -1);
  return grid;
}
inline easyfly_msgs::msg::ObstacleInfoArray makeInvalidObstacles(
  const std::string & frame_id)
{
  easyfly_msgs::msg::ObstacleInfoArray obstacles;
  obstacles.header.frame_id = frame_id;
  return obstacles;
}
}  // namespace easyfly_semantic_map
