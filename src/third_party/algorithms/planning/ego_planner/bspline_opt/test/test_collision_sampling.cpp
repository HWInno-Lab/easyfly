#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "bspline_opt/collision_sampling.h"
#include "bspline_opt/uniform_bspline.h"

namespace
{

TEST(CollisionSampling, CoversTerminalThirdAndExactEndpoint)
{
  const auto samples = ego_planner::collision_sampling::fullTrajectorySampleTimes(
      0.0, 3.0, 9.0, 0.1);

  ASSERT_FALSE(samples.empty());
  EXPECT_DOUBLE_EQ(samples.front(), 0.0);
  EXPECT_DOUBLE_EQ(samples.back(), 3.0);
  EXPECT_TRUE(std::any_of(samples.begin(), samples.end(), [](const double t) {
    return t > 2.0 && t < 3.0;
  }));
  EXPECT_TRUE(std::is_sorted(samples.begin(), samples.end()));
}

TEST(CollisionSampling, RejectsInvalidInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(ego_planner::collision_sampling::fullTrajectorySampleTimes(
      nan, 1.0, 1.0, 0.1).empty());
  EXPECT_TRUE(ego_planner::collision_sampling::fullTrajectorySampleTimes(
      0.0, inf, 1.0, 0.1).empty());
  EXPECT_TRUE(ego_planner::collision_sampling::fullTrajectorySampleTimes(
      1.0, 0.0, 1.0, 0.1).empty());
  EXPECT_TRUE(ego_planner::collision_sampling::fullTrajectorySampleTimes(
      0.0, 1.0, -1.0, 0.1).empty());
  EXPECT_TRUE(ego_planner::collision_sampling::fullTrajectorySampleTimes(
      0.0, 1.0, 1.0, 0.0).empty());
}

TEST(CollisionSampling, BoundsResourceUseForVeryLongPath)
{
  const auto samples = ego_planner::collision_sampling::fullTrajectorySampleTimes(
      0.0, 10.0, 2000.0, 1.0);
  ASSERT_FALSE(samples.empty());
  EXPECT_EQ(samples.size(), static_cast<std::size_t>(20001));
  EXPECT_DOUBLE_EQ(samples.back(), 10.0);

  const auto over_budget = ego_planner::collision_sampling::fullTrajectorySampleTimes(
      0.0, 10.0, 1e12, 1e-6);
  EXPECT_TRUE(over_budget.empty());
}

TEST(CollisionSampling, UsesDerivativeBoundForZeroChordLoopAndMiddleObstacle)
{
  Eigen::MatrixXd control_points = Eigen::MatrixXd::Zero(3, 7);
  control_points.row(0) << 0.0, 0.0, 2.0, 2.0, 0.0, 0.0, 0.0;
  ego_planner::UniformBspline trajectory(control_points, 3, 0.5);

  double start = 0.0;
  double end = 0.0;
  ASSERT_TRUE(trajectory.getTimeSpan(start, end));
  const double max_speed = trajectory.getMaxVelocityBound();
  ASSERT_TRUE(std::isfinite(max_speed));
  EXPECT_GT(max_speed, 0.0);

  const auto samples = ego_planner::collision_sampling::fullTrajectorySampleTimes(
      start, end, max_speed, 0.10);
  ASSERT_GT(samples.size(), static_cast<std::size_t>(2));
  EXPECT_DOUBLE_EQ(samples.back(), end);

  bool sampled_middle = false;
  for (const double t : samples)
  {
    const auto point = trajectory.evaluateDeBoorT(t - start);
    if (point(0) > 0.5)
    {
      sampled_middle = true;
      break;
    }
  }
  EXPECT_TRUE(sampled_middle);

  const auto terminal = trajectory.evaluateDeBoorT(samples.back() - start);
  EXPECT_NEAR(terminal(0), control_points(0, control_points.cols() - 1), 1e-9);
}

TEST(CollisionSampling, RuntimeBudgetStillIncludesExactWindowEnd)
{
  const auto samples = ego_planner::collision_sampling::boundedTrajectorySampleTimes(
      2.0, 12.0, 10.0, 1.0, 128);
  ASSERT_FALSE(samples.empty());
  EXPECT_LE(samples.size(), static_cast<std::size_t>(129));
  EXPECT_DOUBLE_EQ(samples.front(), 2.0);
  EXPECT_DOUBLE_EQ(samples.back(), 12.0);
}

}  // namespace
