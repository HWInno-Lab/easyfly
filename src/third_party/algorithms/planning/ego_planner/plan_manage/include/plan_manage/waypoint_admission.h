// Copyright 2026 EasyFly Project
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

#ifndef PLAN_MANAGE__WAYPOINT_ADMISSION_H_
#define PLAN_MANAGE__WAYPOINT_ADMISSION_H_

#include <cmath>
#include <cstdint>
#include <string>

namespace ego_planner
{
namespace waypoint_admission
{

// RViz and the local planner use either spelling for the same odom frame.
// Do not accept an empty frame or silently reinterpret a map/world frame.
inline bool validOdomFrame(const std::string & frame_id) noexcept
{
  return frame_id == "odom" || frame_id == "/odom";
}

// A builtin_interfaces/Time value is valid only when it is non-negative,
// normalized, and not the all-zero sentinel commonly used by an un-stamped
// message.  The finite check is represented by the integer range checks.
inline bool validStampFields(
  const std::int32_t sec, const std::uint32_t nanosec) noexcept
{
  return sec >= 0 && nanosec < 1000000000U && (sec > 0 || nanosec > 0U);
}

inline bool validTimestampWindow(
  const double age, const double freshness_timeout,
  const double future_tolerance) noexcept
{
  return std::isfinite(age) && std::isfinite(freshness_timeout) &&
         std::isfinite(future_tolerance) && freshness_timeout > 0.0 &&
         freshness_timeout <= 60.0 && future_tolerance >= 0.0 &&
         future_tolerance <= 5.0 && age >= -future_tolerance &&
         age <= freshness_timeout;
}

inline bool nonDecreasingStamp(
  const std::int64_t stamp_ns, const std::int64_t previous_ns,
  const bool have_previous) noexcept
{
  return stamp_ns > 0 && (!have_previous || previous_ns <= 0 ||
         stamp_ns >= previous_ns);
}

inline bool validFinitePosition(
  const double x, const double y, const double z) noexcept
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

inline double boundedFutureTolerance(
  const double configured, const double fallback = 0.2) noexcept
{
  const double safe_fallback =
    std::isfinite(fallback) && fallback >= 0.0 && fallback <= 5.0 ? fallback : 0.2;
  return std::isfinite(configured) && configured >= 0.0 && configured <= 5.0 ?
         configured : safe_fallback;
}

// A low/zero altitude from a 2-D RViz goal means "keep current altitude".
// Invalid odometry is never converted into a usable fallback.
inline bool selectGoalAltitude(
  const double requested_z, const double current_z, double & selected_z,
  const double explicit_minimum = 0.5) noexcept
{
  if (!std::isfinite(explicit_minimum) || explicit_minimum < 0.0 ||
    !std::isfinite(requested_z) || !std::isfinite(current_z))
  {
    return false;
  }
  if (requested_z >= explicit_minimum) {
    selected_z = requested_z;
    return true;
  }
  if (current_z < explicit_minimum) {
    return false;
  }
  selected_z = current_z;
  return true;
}

}    // namespace waypoint_admission
}  // namespace ego_planner

#endif  // PLAN_MANAGE__WAYPOINT_ADMISSION_H_
