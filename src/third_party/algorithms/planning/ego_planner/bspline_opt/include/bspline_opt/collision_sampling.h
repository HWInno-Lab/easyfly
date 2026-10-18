#ifndef BSPLINE_OPT_COLLISION_SAMPLING_H_
#define BSPLINE_OPT_COLLISION_SAMPLING_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ego_planner
{
namespace collision_sampling
{

constexpr std::size_t kMaxIntervals = 20000;
constexpr double kMaxTrajectoryDurationSeconds = 3600.0;

// Return a bounded, monotonic set of time samples whose spatial displacement
// between adjacent samples is conservatively bounded by spatial_resolution.
// max_speed_bound must be an upper bound on the trajectory speed (for a
// UniformBspline this is obtained from the derivative control points), not the
// chord between the two endpoints.  In particular, a loop or a zero-chord
// trajectory still receives interior samples.
inline std::vector<double> boundedTrajectorySampleTimes(
  const double start,
  const double end,
  const double max_speed_bound,
  const double spatial_resolution,
  const std::size_t max_intervals = kMaxIntervals)
{
  if (!std::isfinite(start) || !std::isfinite(end) ||
      !std::isfinite(max_speed_bound) || !std::isfinite(spatial_resolution) ||
      end < start || max_speed_bound < 0.0 || spatial_resolution <= 0.0 ||
      max_intervals == 0 || max_intervals > kMaxIntervals)
  {
    return {};
  }

  const double duration = end - start;
  if (!std::isfinite(duration) || duration > kMaxTrajectoryDurationSeconds)
  {
    return {};
  }

  if (duration == 0.0)
  {
    return {start};
  }

  // A derivative/speed bound gives a conservative time step even when the
  // trajectory returns to its starting point or has strong curvature.
  double step = duration;
  if (max_speed_bound > 0.0)
  {
    step = spatial_resolution / max_speed_bound;
  }
  if (!std::isfinite(step) || step <= 0.0)
  {
    return {};
  }
  // The budget is a hard upper bound.  Never silently coarsen a requested
  // safety spacing: doing so could skip a narrow obstacle.  Callers must
  // reject the trajectory/window when the conservative sample budget is too
  // small.
  const double required_intervals = duration / step;
  if (!std::isfinite(required_intervals) ||
      required_intervals > static_cast<double>(max_intervals) + 1e-9)
  {
    return {};
  }

  std::vector<double> samples;
  samples.reserve(max_intervals + 1);
  samples.push_back(start);

  double current = start;
  for (std::size_t i = 0; i < max_intervals; ++i)
  {
    const double next = current + step;
    if (!std::isfinite(next) || next <= current || next >= end)
    {
      break;
    }
    samples.push_back(next);
    current = next;
  }

  // Always test the terminal point, including when the regular step does not
  // divide the duration exactly.
  if (samples.back() != end)
  {
    samples.push_back(end);
  }
  return samples;
}

// Full validation used before a trajectory is accepted/published.  The
// caller supplies a conservative speed bound; the exact endpoint is always
// included even when the regular step does not divide the duration.
inline std::vector<double> fullTrajectorySampleTimes(
  const double start,
  const double end,
  const double max_speed_bound,
  const double spatial_resolution)
{
  return boundedTrajectorySampleTimes(
    start, end, max_speed_bound, spatial_resolution, kMaxIntervals);
}

}  // namespace collision_sampling
}  // namespace ego_planner

#endif  // BSPLINE_OPT_COLLISION_SAMPLING_H_
