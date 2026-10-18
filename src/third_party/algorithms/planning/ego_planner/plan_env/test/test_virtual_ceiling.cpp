#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "plan_env/virtual_ceiling.h"

TEST(VirtualCeiling, BoundaryIsHardAndContinuous)
{
  EXPECT_FALSE(plan_env::virtualCeilingBlocks(1.999999, 2.0));
  EXPECT_TRUE(plan_env::virtualCeilingBlocks(2.0, 2.0));
  EXPECT_TRUE(plan_env::virtualCeilingBlocks(2.000001, 2.0));
}

TEST(VirtualCeiling, DisabledSentinelRemainsExplicit)
{
  // The GridMap declaration uses -1.0 when no launch override is supplied;
  // both it and the boundary sentinel must leave ordinary samples usable.
  EXPECT_FALSE(plan_env::virtualCeilingBlocks(1.5, -1.0));
  EXPECT_FALSE(plan_env::virtualCeilingBlocks(100.0, -0.5));
}

TEST(VirtualCeiling, InvalidSampleOrConfigurationFailsClosed)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(plan_env::virtualCeilingBlocks(nan, 2.0));
  EXPECT_TRUE(plan_env::virtualCeilingBlocks(1.5, nan));
  EXPECT_TRUE(plan_env::virtualCeilingBlocks(1.5, inf));
}
