#ifndef EGO_PLANNER_TRAJECTORY_SEED_SAFETY_H
#define EGO_PLANNER_TRAJECTORY_SEED_SAFETY_H

#include <algorithm>
#include <cmath>

namespace ego_planner
{
namespace trajectory_seed_safety
{

// Bound only the inserted midpoint of a random polynomial seed to the
// configured map altitude corridor.  The random branch is intended to add an
// escape route around an obstacle; an unconstrained vertical offset can put
// later control points above the hard virtual ceiling (or below the planning
// floor), where GridMap correctly reports occupied.  Do not clamp to the
// endpoint altitude: when the ceiling is disabled, generic 3-D planning must
// retain its vertical exploration.
//
// The hard corridor is used only to validate the generated endpoints.  The
// seed corridor may be contracted (for example, by one voxel) and is applied
// only to the inserted midpoint.  +/-infinity can be used to represent an
// unconfigured bound; NaN, reversed corridors, and a seed corridor outside
// the hard corridor are rejected so an invalid map configuration fails
// closed.
inline bool boundRandomMidpointAltitude(
  const double start_z,
  const double target_z,
  const double candidate_z,
  const double hard_lower_bound,
  const double hard_upper_bound,
  const double seed_lower_bound,
  const double seed_upper_bound,
  double &bounded_z)
{
  const bool hard_lower_unbounded =
    std::isinf(hard_lower_bound) && hard_lower_bound < 0.0;
  const bool hard_upper_unbounded =
    std::isinf(hard_upper_bound) && hard_upper_bound > 0.0;
  const bool seed_lower_unbounded =
    std::isinf(seed_lower_bound) && seed_lower_bound < 0.0;
  const bool seed_upper_unbounded =
    std::isinf(seed_upper_bound) && seed_upper_bound > 0.0;
  if (!std::isfinite(start_z) || !std::isfinite(target_z) ||
      !std::isfinite(candidate_z) ||
      (!std::isfinite(hard_lower_bound) && !hard_lower_unbounded) ||
      (!std::isfinite(hard_upper_bound) && !hard_upper_unbounded) ||
      (!std::isfinite(seed_lower_bound) && !seed_lower_unbounded) ||
      (!std::isfinite(seed_upper_bound) && !seed_upper_unbounded) ||
      hard_lower_bound > hard_upper_bound ||
      seed_lower_bound > seed_upper_bound ||
      (!hard_lower_unbounded && seed_lower_bound < hard_lower_bound) ||
      (!hard_upper_unbounded && seed_upper_bound > hard_upper_bound)) {
    return false;
  }

  // Both endpoints are part of the generated polynomial and therefore must
  // themselves be inside the actual hard corridor.  They may legitimately
  // lie in the contracted seed margin; only the inserted point is clamped.
  if ((!hard_lower_unbounded &&
       (start_z < hard_lower_bound || target_z < hard_lower_bound)) ||
      (!hard_upper_unbounded &&
       (start_z > hard_upper_bound || target_z > hard_upper_bound))) {
    return false;
  }

  bounded_z = candidate_z;
  if (!seed_lower_unbounded) {
    bounded_z = std::max(bounded_z, seed_lower_bound);
  }
  if (!seed_upper_unbounded) {
    bounded_z = std::min(bounded_z, seed_upper_bound);
  }
  return std::isfinite(bounded_z) &&
         (seed_lower_unbounded || bounded_z >= seed_lower_bound) &&
         (seed_upper_unbounded || bounded_z <= seed_upper_bound) &&
         (hard_lower_unbounded || bounded_z >= hard_lower_bound) &&
         (hard_upper_unbounded || bounded_z <= hard_upper_bound);
}

}  // namespace trajectory_seed_safety
}  // namespace ego_planner

#endif  // EGO_PLANNER_TRAJECTORY_SEED_SAFETY_H
