// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_target_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
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
