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
#ifndef EASYFLY_TARGET_TRACKING__FOLLOW_OFFSET_HPP_
#define EASYFLY_TARGET_TRACKING__FOLLOW_OFFSET_HPP_

#include <array>
#include <cmath>

namespace easyfly_target_tracking
{
// Capture once per goal. Keeping this odom offset also across ID reacquisition
// preserves lateral target motion instead of absorbing it into a new bearing.
inline std::array<double, 2> followOffset(
  double target_x, double target_y, double uav_x, double uav_y,
  double uav_yaw, double distance)
{
  const double dx = target_x - uav_x;
  const double dy = target_y - uav_y;
  const double length = std::hypot(dx, dy);
  if (length > 0.3) {
    return {-distance * dx / length, -distance * dy / length};
  }
  // With no reliable horizontal bearing, stay behind the current heading.
  return {-distance * std::cos(uav_yaw), -distance * std::sin(uav_yaw)};
}
}  // namespace easyfly_target_tracking
#endif  // EASYFLY_TARGET_TRACKING__FOLLOW_OFFSET_HPP_
