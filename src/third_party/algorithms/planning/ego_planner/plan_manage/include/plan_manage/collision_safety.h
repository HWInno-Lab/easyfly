#ifndef EGO_PLANNER_COLLISION_SAFETY_H_
#define EGO_PLANNER_COLLISION_SAFETY_H_

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

}  // namespace collision_safety
}  // namespace ego_planner

#endif  // EGO_PLANNER_COLLISION_SAFETY_H_
