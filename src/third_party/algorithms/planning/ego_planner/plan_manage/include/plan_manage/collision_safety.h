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

#ifndef PLAN_MANAGE__COLLISION_SAFETY_H_
#define PLAN_MANAGE__COLLISION_SAFETY_H_

#include <cmath>

namespace ego_planner
{
namespace collision_safety
{

constexpr double kDefaultEmergencyTime = 1.0;
constexpr double kMaxEmergencyTime = 60.0;
constexpr double kMaxTrajectoryDuration = 300.0;

inline bool validTrajectoryDuration(const double duration) noexcept
{
  return std::isfinite(duration) && duration > 0.0 &&
         duration <= kMaxTrajectoryDuration;
}

inline double sanitizeEmergencyTime(
  const double configured,
  const double fallback = kDefaultEmergencyTime) noexcept
{
  const double safe_fallback =
    std::isfinite(fallback) && fallback > 0.0 && fallback <= kMaxEmergencyTime ?
    fallback : kDefaultEmergencyTime;
  return std::isfinite(configured) && configured > 0.0 &&
         configured <= kMaxEmergencyTime ? configured : safe_fallback;
}

// Equality is intentionally included: at the configured margin there is no
// longer enough budget to synchronously plan and publish another trajectory.
// Non-finite input is fail-closed as well.
inline bool emergencyWindowImminent(
  const double remaining, const double emergency_time) noexcept
{
  return !std::isfinite(remaining) ||
         !std::isfinite(emergency_time) || emergency_time <= 0.0 ||
         remaining <= emergency_time;
}

}    // namespace collision_safety
}  // namespace ego_planner

#endif  // PLAN_MANAGE__COLLISION_SAFETY_H_
