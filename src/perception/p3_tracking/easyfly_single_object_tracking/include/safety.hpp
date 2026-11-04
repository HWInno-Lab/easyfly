// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_single_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
namespace ef_safety
{
inline double steady()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline int64_t stamp(const std_msgs::msg::Header & h)
{
  if (h.stamp.sec < 0 || h.stamp.nanosec >= 1000000000u) {
    return 0;
  }
  return static_cast<int64_t>(h.stamp.sec) * 1000000000LL + h.stamp.nanosec;
}
struct ClockGate
{
  int64_t last{0};
  double progress{0};

  bool check(int64_t now, double age)
  {
    const double s = steady();
    const bool backwards = last > 0 && now < last;

    if (now != last) {
      progress = s;
    }
    last = now;
    return now > 0 && !backwards && s - progress <= age;
  }
};

inline bool fresh(
  const std_msgs::msg::Header & h, int64_t now, double receipt, double age,
  const std::string & frame)
{
  const auto source = stamp(h);
  const double source_age = static_cast<double>(now - source) * 1e-9;

  return now > 0 && source > 0 && std::isfinite(receipt) && std::isfinite(age) && age > 0 &&
         h.frame_id == frame && source_age >= -std::min(0.2, age) && source_age <= age &&
         steady() - receipt >= 0 && steady() - receipt <= age;
}
}  // namespace ef_safety
