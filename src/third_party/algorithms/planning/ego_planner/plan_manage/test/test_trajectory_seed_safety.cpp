#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <plan_env/virtual_ceiling.h>
#include <plan_manage/trajectory_seed_safety.h>

namespace
{

using ego_planner::trajectory_seed_safety::boundRandomMidpointAltitude;

TEST(TrajectorySeedSafety, ClampsRandomVerticalExcursionBelowHardCeiling)
{
  double bounded = 0.0;
  // This is the G8 horizontal E1 shape: both endpoints hover at 1.49 m, but
  // the historical random vertical offset can produce a multi-metre seed.
  ASSERT_TRUE(boundRandomMidpointAltitude(
    1.49, 1.49, 4.70, 0.50, 2.00, 0.60, 1.90, bounded));
  EXPECT_DOUBLE_EQ(bounded, 1.90);
  EXPECT_FALSE(plan_env::virtualCeilingBlocks(bounded, 2.0));
}

TEST(TrajectorySeedSafety, KeepsSeedOutOfLegacyCeilingVoxel)
{
  constexpr double map_origin_z = 0.50;
  constexpr double resolution = 0.10;
  constexpr double virtual_ceiling = 2.00;
  constexpr double resolution_inv = 1.0 / resolution;
  const int ceil_id = static_cast<int>(std::floor(
    (virtual_ceiling - map_origin_z) * resolution_inv)) - 1;
  ASSERT_EQ(ceil_id, 14);

  const double ceil_voxel_lower =
    map_origin_z + static_cast<double>(ceil_id) * resolution;
  const double ceil_margin = std::min(1e-4, 0.5 * resolution);
  const double strict_seed_upper = std::nextafter(
    ceil_voxel_lower - ceil_margin,
    -std::numeric_limits<double>::infinity());
  EXPECT_LT(strict_seed_upper, ceil_voxel_lower);
  EXPECT_EQ(static_cast<int>(std::floor(
               (strict_seed_upper - map_origin_z) * resolution_inv)), ceil_id - 1);

  double bounded = 0.0;
  ASSERT_TRUE(boundRandomMidpointAltitude(
    1.49, 1.49, 4.70, map_origin_z, virtual_ceiling,
    map_origin_z + resolution, strict_seed_upper, bounded));
  EXPECT_LT(bounded, ceil_voxel_lower);
  EXPECT_EQ(static_cast<int>(std::floor(
               (bounded - map_origin_z) * resolution_inv)), ceil_id - 1);
}

TEST(TrajectorySeedSafety, KeepsValidAltitudeVariationInsideMapCorridor)
{
  double bounded = 0.0;
  ASSERT_TRUE(boundRandomMidpointAltitude(
    1.10, 1.70, 1.35, 0.50, 2.00, 0.60, 1.90, bounded));
  EXPECT_DOUBLE_EQ(bounded, 1.35);

  ASSERT_TRUE(boundRandomMidpointAltitude(
    1.10, 1.70, -2.0, 0.50, 2.00, 0.60, 1.90, bounded));
  EXPECT_DOUBLE_EQ(bounded, 0.60);

  // A disabled ceiling must not erase valid 3-D exploration above the
  // endpoint altitude; only the lower map boundary remains enforced.
  ASSERT_TRUE(boundRandomMidpointAltitude(
    1.10, 1.70, 4.0, 0.50,
    std::numeric_limits<double>::infinity(), 0.60,
    std::numeric_limits<double>::infinity(), bounded));
  EXPECT_DOUBLE_EQ(bounded, 4.0);
}

TEST(TrajectorySeedSafety, AllowsEndpointsInsideSeedMargin)
{
  double bounded = 0.0;
  // Both endpoints are inside the hard corridor but in its contracted seed
  // margin.  They remain valid polynomial endpoints; only the midpoint is
  // clamped to the one-voxel seed corridor.
  ASSERT_TRUE(boundRandomMidpointAltitude(
    0.55, 1.95, 4.0, 0.50, 2.00, 0.60, 1.90, bounded));
  EXPECT_DOUBLE_EQ(bounded, 1.90);

  ASSERT_TRUE(boundRandomMidpointAltitude(
    0.55, 0.55, -4.0, 0.50, 2.00, 0.60, 1.90, bounded));
  EXPECT_DOUBLE_EQ(bounded, 0.60);
}

TEST(TrajectorySeedSafety, RejectsEndpointsOutsideHardBoundary)
{
  double bounded = 0.0;
  EXPECT_FALSE(boundRandomMidpointAltitude(
    0.49, 1.0, 1.5, 0.50, 2.00, 0.60, 1.90, bounded));
  EXPECT_FALSE(boundRandomMidpointAltitude(
    1.0, 2.01, 1.5, 0.50, 2.00, 0.60, 1.90, bounded));
}

TEST(TrajectorySeedSafety, RejectsInvalidInputsAndBounds)
{
  double bounded = 0.0;
  EXPECT_FALSE(boundRandomMidpointAltitude(
    std::numeric_limits<double>::quiet_NaN(), 1.49, 1.5,
    0.50, 2.00, 0.60, 1.90, bounded));
  EXPECT_FALSE(boundRandomMidpointAltitude(
    1.49, 1.49, std::numeric_limits<double>::quiet_NaN(),
    0.50, 2.00, 0.60, 1.90, bounded));
  EXPECT_FALSE(boundRandomMidpointAltitude(
    1.49, 1.49, 1.5, 2.0, 1.0, 1.1, 1.9, bounded));
  EXPECT_FALSE(boundRandomMidpointAltitude(
    1.49, 1.49, 1.5, 0.60,
    std::numeric_limits<double>::infinity() * -1.0,
    0.70, 1.90, bounded));
  EXPECT_FALSE(boundRandomMidpointAltitude(
    1.49, 1.49, 1.5,
    std::numeric_limits<double>::quiet_NaN(), 2.00, 0.60, 1.90, bounded));
  EXPECT_FALSE(boundRandomMidpointAltitude(
    1.49, 1.49, 1.5, 0.50, 2.00, 0.60,
    std::numeric_limits<double>::quiet_NaN(), bounded));
}

}  // namespace
