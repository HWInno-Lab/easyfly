#pragma once
#include <string>
#include <cmath>
#include <limits>
namespace ego_planner {
inline double navigationAxisVelocityLimit(double speed, double tolerance) {
  if (!std::isfinite(speed) || speed <= 0.0 || !std::isfinite(tolerance) ||
      tolerance < 0.0 || tolerance > 1.0) return std::numeric_limits<double>::quiet_NaN();
  // UniformBspline's feasibility checker adds 1e-4 to each axis limit.
  const double cap = (speed / std::sqrt(3.0) - 1e-4) / (1.0 + tolerance);
  return cap > 0.0 ? cap : std::numeric_limits<double>::quiet_NaN();
}
inline bool matchesNavigationSession(const std::string &candidate, const std::string &active) {
  return candidate.size() <= 128 && candidate == active;
}
inline bool runtimeTrajectoryBelongsToSession(bool managed,
  const std::string &published, const std::string &active) {
  return !managed || (!active.empty() && matchesNavigationSession(published, active));
}
}  // namespace ego_planner
