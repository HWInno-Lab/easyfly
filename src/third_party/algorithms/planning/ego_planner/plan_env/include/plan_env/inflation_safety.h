#ifndef PLAN_ENV_INFLATION_SAFETY_H
#define PLAN_ENV_INFLATION_SAFETY_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace plan_env
{

// These limits protect both the integer conversion and the temporary kernel
// used by GridMap.  The normal EasyFly configuration (0.80 m at 0.10 m/cell)
// is far below them.  A malformed override is rejected instead of silently
// turning into an unbounded allocation or loop.
  constexpr double kMaxInflationMeters = 4.0;
  constexpr int kMaxInflationStep = 64;
  constexpr std::size_t kMaxInflationKernelElements = 1000000;
  // Trusted point clouds are obstacle surfaces rather than solid volumes.  A
  // bounded vertical margin closes small vertical sampling gaps without
  // allowing a malformed parameter to turn the cloud callback into an
  // unbounded z-loop.
  constexpr double kMaxTrustedCloudVerticalInflationMeters = 2.0;
  constexpr int kMaxTrustedCloudVerticalStep = 32;

  struct InflationKernelConfig
  {
    int step {0};
    std::size_t kernel_elements {0};
  };

  inline bool computeTrustedCloudVerticalStep(
    double resolution,
    double inflation,
    int map_voxel_z,
    int & step)
  {
    if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(inflation) || inflation < 0.0 ||
      inflation > kMaxTrustedCloudVerticalInflationMeters || map_voxel_z <= 0)
    {
      return false;
    }

    const long double ratio = static_cast < long double > (inflation) /
      static_cast < long double > (resolution);
    if (!std::isfinite(ratio) || ratio < 0.0L ||
      ratio > static_cast < long double > (kMaxTrustedCloudVerticalStep))
    {
      return false;
    }

    const long double rounded = std::ceil(ratio);
    if (!std::isfinite(rounded) || rounded < 0.0L ||
      rounded > static_cast < long double > (kMaxTrustedCloudVerticalStep))
    {
      return false;
    }
    const int candidate = static_cast < int > (rounded);
    if (candidate >= map_voxel_z) {
      return false;
    }
    step = candidate;
    return true;
  }

  inline bool checkedKernelElements(int step, std::size_t & elements)
  {
    if (step < 0) {
      return false;
    }

    const auto max_size = std::numeric_limits < std::size_t > ::max();
    const auto step_u = static_cast < std::size_t > (step);
    if (step_u > (max_size - 1U) / 2U) {
      return false;
    }
    const std::size_t side = 2U * step_u + 1U;
    if (side > max_size / side) {
      return false;
    }
    const std::size_t square = side * side;
    if (square > max_size / side) {
      return false;
    }
    elements = square * side;
    return elements <= kMaxInflationKernelElements;
  }

// Convert metres to a bounded integer voxel radius.  map_voxels are supplied
// by GridMap after its map dimensions have been checked.  Requiring the
// radius to fit the smallest map dimension prevents a valid-looking
// parameter from allocating a kernel larger than the map it operates on.
  inline bool computeInflationKernel(
    double resolution,
    double inflation,
    int map_voxel_x,
    int map_voxel_y,
    int map_voxel_z,
    InflationKernelConfig & result)
  {
    if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(inflation) || inflation <= 0.0 ||
      inflation > kMaxInflationMeters ||
      map_voxel_x <= 0 || map_voxel_y <= 0 || map_voxel_z <= 0)
    {
      return false;
    }

    // long double keeps the ratio check meaningful for very small resolutions
    // without first overflowing a double-to-int conversion.
    const long double ratio = static_cast < long double > (inflation) /
      static_cast < long double > (resolution);
    if (!std::isfinite(ratio) || ratio <= 0.0L ||
      ratio > static_cast < long double > (kMaxInflationStep))
    {
      return false;
    }

    const long double rounded = std::ceil(ratio);
    if (!std::isfinite(rounded) || rounded < 1.0L ||
      rounded > static_cast < long double > (kMaxInflationStep))
    {
      return false;
    }
    const int step = static_cast < int > (rounded);

    const int map_step_limit =
      std::min(map_voxel_x, std::min(map_voxel_y, map_voxel_z)) - 1;
    if (map_step_limit < 1 || step > map_step_limit) {
      return false;
    }

    std::size_t elements = 0;
    if (!checkedKernelElements(step, elements)) {
      return false;
    }
    result.step = step;
    result.kernel_elements = elements;
    return true;
  }

}  // namespace plan_env

#endif  // PLAN_ENV_INFLATION_SAFETY_H
