#ifndef PLAN_ENV_VIRTUAL_CEILING_H
#define PLAN_ENV_VIRTUAL_CEILING_H

#include <cmath>

namespace plan_env
{

// A negative value at or below this sentinel keeps the historical
// "virtual ceiling disabled" configuration.  Any other non-finite value is
// invalid and must fail closed instead of silently disabling the bound.
constexpr double kVirtualCeilingDisabledThreshold = -0.5;

// Collision callers use a truthy negative value for out-of-map samples.  The
// hard check is deliberately separate from the one-voxel visualization mark:
// marking a single voxel at the ceiling can otherwise be stepped over by a
// sparse trajectory sample.
inline bool virtualCeilingBlocks(const double z, const double ceiling)
{
  if (!std::isfinite(z)) {
    return true;
  }
  if (!std::isfinite(ceiling)) {
    return true;
  }
  if (ceiling <= kVirtualCeilingDisabledThreshold) {
    return false;
  }
  return z >= ceiling;
}

}  // namespace plan_env

#endif  // PLAN_ENV_VIRTUAL_CEILING_H
