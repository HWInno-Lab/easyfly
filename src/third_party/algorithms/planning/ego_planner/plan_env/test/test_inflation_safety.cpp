#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "plan_env/inflation_safety.h"

namespace
{

constexpr int kMapX = 500;
constexpr int kMapY = 250;
constexpr int kMapZ = 50;

plan_env::InflationKernelConfig compute(double resolution, double inflation)
{
  plan_env::InflationKernelConfig result;
  EXPECT_TRUE(
    plan_env::computeInflationKernel(
      resolution, inflation, kMapX, kMapY, kMapZ, result));
  return result;
}

}  // namespace

TEST(InflationSafety, NormalG8ConfigurationIsBounded)
{
  const auto config = compute(0.10, 0.80);
  EXPECT_EQ(config.step, 8);
  EXPECT_EQ(config.kernel_elements, static_cast<std::size_t>(17 * 17 * 17));
}

TEST(InflationSafety, RejectsInvalidResolution)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double resolution : {nan, inf, -1.0, 0.0}) {
    plan_env::InflationKernelConfig config;
    EXPECT_FALSE(
      plan_env::computeInflationKernel(
        resolution, 0.80, kMapX, kMapY, kMapZ, config));
  }
}

TEST(InflationSafety, RejectsInvalidOrOversizedInflation)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double inflation : {nan, inf, -1.0, 0.0, 1e308,
      plan_env::kMaxInflationMeters + 0.01})
  {
    plan_env::InflationKernelConfig config;
    EXPECT_FALSE(
      plan_env::computeInflationKernel(
        0.10, inflation, kMapX, kMapY, kMapZ, config));
  }
}

TEST(InflationSafety, RejectsKernelThatWouldExceedHardStepOrSizeLimit)
{
  plan_env::InflationKernelConfig config;
  EXPECT_FALSE(
    plan_env::computeInflationKernel(
      1e-3, 0.80, kMapX, kMapY, kMapZ, config));

  EXPECT_FALSE(
    plan_env::computeInflationKernel(
      0.10, 0.80, 4, 4, 4, config));
}

TEST(InflationSafety, KernelSizeHelperChecksIntegerMultiplication)
{
  std::size_t elements = 0;
  EXPECT_TRUE(plan_env::checkedKernelElements(8, elements));
  EXPECT_EQ(elements, static_cast<std::size_t>(17 * 17 * 17));
  EXPECT_FALSE(plan_env::checkedKernelElements(-1, elements));
  EXPECT_FALSE(
    plan_env::checkedKernelElements(
      plan_env::kMaxInflationStep,
      elements));
}

TEST(InflationSafety, TrustedCloudVerticalMarginCoversSparseZReturns)
{
  int step = -1;
  EXPECT_TRUE(
    plan_env::computeTrustedCloudVerticalStep(0.10, 0.40, kMapZ, step));
  EXPECT_EQ(step, 4);
}

TEST(InflationSafety, TrustedCloudVerticalMarginRejectsBadConfiguration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double margin : {nan, inf, -1.0, 1e308,
      plan_env::kMaxTrustedCloudVerticalInflationMeters + 0.01})
  {
    int step = -1;
    EXPECT_FALSE(
      plan_env::computeTrustedCloudVerticalStep(0.10, margin, kMapZ, step));
  }

  int step = -1;
  EXPECT_FALSE(
    plan_env::computeTrustedCloudVerticalStep(1e-3, 0.40, kMapZ, step));
  EXPECT_FALSE(
    plan_env::computeTrustedCloudVerticalStep(0.10, 0.40, 4, step));
  EXPECT_TRUE(
    plan_env::computeTrustedCloudVerticalStep(0.10, 0.0, kMapZ, step));
  EXPECT_EQ(step, 0);
}
