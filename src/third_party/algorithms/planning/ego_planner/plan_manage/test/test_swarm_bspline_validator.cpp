#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <bspline_opt/uniform_bspline.h>
#include <plan_manage/bspline_runtime_validator.h>
#include <plan_manage/swarm_bspline_validator.h>
#include <plan_manage/navigation_session_contract.h>
#include <plan_manage/fixed_endpoint.h>

namespace
{

TEST(ManagedFixedEndpoint, ActualCubicEndpointHasExactPositionVelocityAndZeroAcceleration) {
  const Eigen::Vector3d target(1.1, -1.1, 1.5);
  for (const double interval : {0.1, 0.5, 1.0}) {
    for (const auto &velocity : {Eigen::Vector3d::Zero().eval(), Eigen::Vector3d(0.2, -0.1, 0.0)}) {
      Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 8);
      ASSERT_TRUE(ego_planner::setFixedCubicEndpoint(points, interval, target, velocity));
      ego_planner::UniformBspline trajectory(points, 3, interval);
      auto derivative = trajectory.getDerivative();
      auto acceleration = derivative.getDerivative();
      const double end = trajectory.getTimeSum();
      EXPECT_LT((trajectory.evaluateDeBoorT(end) - target).norm(), 1e-12);
      EXPECT_LT((derivative.evaluateDeBoorT(end) - velocity).norm(), 1e-12);
      EXPECT_LT(acceleration.evaluateDeBoorT(end).norm(), 1e-10);
    }
  }
  Eigen::MatrixXd overlapping = Eigen::MatrixXd::Zero(3, 6);
  EXPECT_FALSE(ego_planner::setFixedCubicEndpoint(overlapping, 0.1, target, Eigen::Vector3d::Zero()));
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 8);
  EXPECT_FALSE(ego_planner::setFixedCubicEndpoint(points, 0.0, target, Eigen::Vector3d::Zero()));
}

TEST(NavigationVelocityLimit, ConservativeAxesBoundActualDiagonalSplineSpeed) {
  for (const double speed : {0.01, 0.5, 2.0}) {
    for (const double tolerance : {0.0, 0.05, 1.0}) {
      const double axis = ego_planner::navigationAxisVelocityLimit(speed, tolerance);
      ASSERT_TRUE(std::isfinite(axis));
      EXPECT_LE(std::sqrt(3.0) * (axis * (1.0 + tolerance) + 1e-4), speed + 1e-12);
      Eigen::MatrixXd points(3, 4);
      for (int i = 0; i < 4; ++i) points.col(i).setConstant(i * axis);
      ego_planner::UniformBspline spline(points, 3, 1.0);
      auto velocity = spline.getDerivative();
      for (int i = 0; i <= 100; ++i)
        EXPECT_LE(velocity.evaluateDeBoorT(i / 100.0).norm(), speed + 1e-12);
    }
  }
  EXPECT_FALSE(std::isfinite(ego_planner::navigationAxisVelocityLimit(0.0, 0.05)));
  EXPECT_FALSE(std::isfinite(ego_planner::navigationAxisVelocityLimit(1.0, -0.01)));
  EXPECT_FALSE(std::isfinite(ego_planner::navigationAxisVelocityLimit(1.0, 1.01)));
  EXPECT_FALSE(std::isfinite(ego_planner::navigationAxisVelocityLimit(1e-5, 0.0)));
  EXPECT_FALSE(std::isfinite(ego_planner::navigationAxisVelocityLimit(
    1.0, std::numeric_limits<double>::quiet_NaN())));
}

struct FakePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct FakeTime
{
  int32_t sec{1};
  uint32_t nanosec{0};
};

struct FakeBspline
{
  int32_t drone_id{0};
  int32_t order{3};
  int64_t traj_id{1};
  FakeTime start_time;
  std::vector<double> knots;
  std::vector<FakePoint> pos_pts;
  std::vector<double> yaw_pts;
  double yaw_dt{0.0};
};

struct FakeMultiBsplines
{
  int32_t drone_id_from{0};
  std::vector<FakeBspline> traj;
};

FakeBspline validTrajectory(const int id = 0)
{
  FakeBspline msg;
  msg.drone_id = id;
  msg.knots = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  msg.pos_pts = {{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0},
                 {2.0, 0.0, 1.0}, {3.0, 0.0, 1.0}};
  return msg;
}

TEST(SwarmBsplineValidator, AcceptsValidTrajectoryAndMultiMessage)
{
  const auto trajectory = validTrajectory(2);
  const auto result = ego_planner::swarm_validation::validateBspline(trajectory);
  ASSERT_TRUE(result.valid());
  EXPECT_DOUBLE_EQ(result.duration, 1.0);

  FakeMultiBsplines multi;
  multi.drone_id_from = 1;
  multi.traj = {validTrajectory(0), validTrajectory(1)};
  EXPECT_TRUE(ego_planner::swarm_validation::validateMultiBsplines(multi).valid());

  auto repeated_knots = validTrajectory();
  repeated_knots.knots = {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  const auto repeated_result =
      ego_planner::swarm_validation::validateBspline(repeated_knots);
  EXPECT_TRUE(repeated_result.valid());
  EXPECT_DOUBLE_EQ(repeated_result.duration, 1.0);
}

TEST(SwarmBsplineValidator, MultiRequiresIndexedTrajectoryIdentity)
{
  FakeMultiBsplines swapped;
  swapped.drone_id_from = 1;
  swapped.traj = {validTrajectory(1), validTrajectory(0)};
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(swapped).error,
            ego_planner::swarm_validation::ValidationError::kMultiDroneId);

  FakeMultiBsplines duplicate;
  duplicate.drone_id_from = 1;
  duplicate.traj = {validTrajectory(0), validTrajectory(0)};
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(duplicate).error,
            ego_planner::swarm_validation::ValidationError::kMultiDroneId);

  FakeMultiBsplines missing;
  missing.drone_id_from = 2;
  missing.traj = {validTrajectory(0), validTrajectory(2), validTrajectory(2)};
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(missing).error,
            ego_planner::swarm_validation::ValidationError::kMultiDroneId);

  FakeMultiBsplines truncated;
  truncated.drone_id_from = 2;
  truncated.traj = {validTrajectory(0), validTrajectory(1)};
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(truncated).error,
            ego_planner::swarm_validation::ValidationError::kMultiDroneId);

  FakeMultiBsplines out_of_range;
  out_of_range.drone_id_from = 1;
  out_of_range.traj = {
    validTrajectory(0),
    validTrajectory(ego_planner::swarm_validation::kMaxSwarmId + 1)};
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(out_of_range).error,
            ego_planner::swarm_validation::ValidationError::kMultiDroneId);
}

TEST(SwarmBsplineValidator, InvalidPlanningHorizonIsRejectedBeforeIngress)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ego_planner::swarm_validation::validPlanningHorizon(nan));
  EXPECT_FALSE(ego_planner::swarm_validation::validPlanningHorizon(0.0));
  EXPECT_FALSE(ego_planner::swarm_validation::validPlanningHorizon(-1.0));
  EXPECT_FALSE(ego_planner::swarm_validation::validPlanningHorizon(
      ego_planner::swarm_validation::kMaxPlanningHorizon + 1.0));
  EXPECT_TRUE(ego_planner::swarm_validation::validPlanningHorizon(5.0));

  // The same fail-closed commit gate used by Broadcast proves that an
  // invalid precondition cannot resize or overwrite a prior valid slot.
  struct Slot
  {
    int value;
  };
  std::vector<Slot> slots{{7}};
  const Slot candidate{42};
  const auto blank = []() { return Slot{0}; };
  EXPECT_FALSE(ego_planner::swarm_validation::commitSwarmCandidate(
      slots, 0, ego_planner::swarm_validation::validPlanningHorizon(0.0),
      candidate, blank));
  ASSERT_EQ(slots.size(), 1U);
  EXPECT_EQ(slots[0].value, 7);
}

TEST(SwarmBsplineValidator, RejectsNegativeAndHugeIdsBeforeIndexing)
{
  auto negative = validTrajectory();
  negative.drone_id = -1;
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(negative).error,
            ego_planner::swarm_validation::ValidationError::kDroneId);

  auto huge = validTrajectory();
  huge.drone_id = ego_planner::swarm_validation::kMaxSwarmId + 1;
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(huge).error,
            ego_planner::swarm_validation::ValidationError::kDroneId);

  auto huge_traj_id = validTrajectory();
  huge_traj_id.traj_id = std::numeric_limits<int64_t>::max();
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(huge_traj_id).error,
            ego_planner::swarm_validation::ValidationError::kTrajectoryId);

  auto zero_time = validTrajectory();
  zero_time.start_time.sec = 0;
  zero_time.start_time.nanosec = 0;
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(zero_time).error,
            ego_planner::swarm_validation::ValidationError::kStartTime);

  FakeMultiBsplines multi;
  multi.drone_id_from = -1;
  multi.traj = {validTrajectory()};
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(multi).error,
            ego_planner::swarm_validation::ValidationError::kMultiDroneId);
}

TEST(SwarmBsplineValidator, RejectsUnsupportedOrderAndOversizedVectors)
{
  auto low_order = validTrajectory();
  low_order.order = ego_planner::swarm_validation::kMinBsplineOrder - 1;
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(low_order).error,
            ego_planner::swarm_validation::ValidationError::kOrder);

  auto high_order = validTrajectory();
  high_order.order = ego_planner::swarm_validation::kMaxBsplineOrder + 1;
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(high_order).error,
            ego_planner::swarm_validation::ValidationError::kOrder);

  auto too_many_points = validTrajectory();
  too_many_points.pos_pts.resize(
      ego_planner::swarm_validation::kMaxControlPoints + 1);
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(too_many_points).error,
            ego_planner::swarm_validation::ValidationError::kControlPointsTooLarge);

  auto too_many_knots = validTrajectory();
  too_many_knots.knots.resize(ego_planner::swarm_validation::kMaxKnots + 1);
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(too_many_knots).error,
            ego_planner::swarm_validation::ValidationError::kKnotsTooLarge);
}

TEST(SwarmBsplineValidator, RejectsNonCubicConsumerInput)
{
  auto quadratic = validTrajectory();
  quadratic.order = 2;
  quadratic.knots = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  EXPECT_EQ(ego_planner::swarm_validation::validateCubicBspline(quadratic).error,
            ego_planner::swarm_validation::ValidationError::kOrder);
  FakeMultiBsplines multi;
  multi.traj = {quadratic};
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(multi).error,
            ego_planner::swarm_validation::ValidationError::kOrder);

  auto too_few_for_cubic = validTrajectory();
  too_few_for_cubic.pos_pts.resize(3);
  too_few_for_cubic.knots = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  EXPECT_EQ(
      ego_planner::swarm_validation::validateCubicBspline(too_few_for_cubic).error,
      ego_planner::swarm_validation::ValidationError::kControlPointsTooShort);
}

TEST(SwarmBsplineValidator, RejectsPhysicalAndDerivativeOverflow)
{
  auto huge_point = validTrajectory();
  huge_point.pos_pts[0].x = 1.0e308;
  EXPECT_EQ(ego_planner::swarm_validation::validateCubicBspline(huge_point).error,
            ego_planner::swarm_validation::ValidationError::kControlPointOutOfRange);

  auto huge_velocity = validTrajectory();
  huge_velocity.pos_pts[1].x = 1000.0;
  huge_velocity.pos_pts[2].x = -1000.0;
  huge_velocity.knots = {0.0, 1.0e-6, 2.0e-6, 3.0e-6, 4.0e-6, 1.0, 2.0, 3.0};
  EXPECT_EQ(ego_planner::swarm_validation::validateCubicBspline(huge_velocity).error,
            ego_planner::swarm_validation::ValidationError::kDerivativeOutOfRange);
}

TEST(SwarmBsplineValidator, RejectsRepeatedKnotThatBreaksSecondDerivative)
{
  auto repeated = validTrajectory();
  repeated.pos_pts.push_back({4.0, 0.0, 1.0});
  repeated.knots = {0.0, 1.0, 2.0, 2.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  EXPECT_EQ(ego_planner::swarm_validation::validateCubicBspline(repeated).error,
            ego_planner::swarm_validation::ValidationError::kDerivativeDenominator);

  Eigen::MatrixXd points(3, 5);
  points << 0.0, 1.0, 2.0, 3.0, 4.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 1.0, 1.0, 1.0, 1.0;
  Eigen::VectorXd knots(9);
  knots << 0.0, 1.0, 2.0, 2.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  ego_planner::UniformBspline position(points, 3, 1.0);
  position.setKnot(knots);
  ego_planner::UniformBspline velocity = position.getDerivative();
  ego_planner::UniformBspline acceleration = velocity.getDerivative();
  EXPECT_FALSE(acceleration.getControlPoint().allFinite());
}

TEST(SwarmBsplineValidator, RuntimeCandidateAndCommitGateAreFailClosed)
{
  const auto msg = validTrajectory();
  Eigen::MatrixXd points(3, 4);
  points << 0.0, 1.0, 2.0, 3.0,
            0.0, 0.0, 0.0, 0.0,
            1.0, 1.0, 1.0, 1.0;
  Eigen::VectorXd knots(8);
  for (int i = 0; i < knots.size(); ++i)
  {
    knots(i) = static_cast<double>(i);
  }
  ego_planner::UniformBspline position(points, 3, 1.0);
  position.setKnot(knots);
  ego_planner::UniformBspline velocity = position.getDerivative();
  ego_planner::UniformBspline acceleration = velocity.getDerivative();
  EXPECT_TRUE(ego_planner::swarm_validation::validateCubicRuntimeTrajectory(
                  position, velocity, acceleration, msg.pos_pts.size(), msg.knots.size(), 1.0)
                  .valid());

  struct Slot
  {
    int value;
  };
  std::vector<Slot> slots{{7}};
  const Slot candidate{42};
  const auto blank = []() { return Slot{0}; };
  EXPECT_FALSE(ego_planner::swarm_validation::commitSwarmCandidate(
      slots, 0, false, candidate, blank));
  ASSERT_EQ(slots.size(), 1U);
  EXPECT_EQ(slots[0].value, 7);
  EXPECT_TRUE(ego_planner::swarm_validation::commitSwarmCandidate(
      slots, 0, true, candidate, blank));
  EXPECT_EQ(slots[0].value, 42);
}

TEST(SwarmBsplineValidator, RejectsEmptyShortAndMismatchedKnotData)
{
  auto empty_points = validTrajectory();
  empty_points.pos_pts.clear();
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(empty_points).error,
            ego_planner::swarm_validation::ValidationError::kControlPointsEmpty);

  auto short_points = validTrajectory();
  short_points.pos_pts.resize(3);
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(short_points).error,
            ego_planner::swarm_validation::ValidationError::kControlPointsTooShort);

  auto empty_knots = validTrajectory();
  empty_knots.knots.clear();
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(empty_knots).error,
            ego_planner::swarm_validation::ValidationError::kKnotsEmpty);

  auto mismatch = validTrajectory();
  mismatch.knots.pop_back();
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(mismatch).error,
            ego_planner::swarm_validation::ValidationError::kKnotCountMismatch);
}

TEST(SwarmBsplineValidator, RejectsNonFiniteNonMonotonicAndBadDuration)
{
  auto nan_point = validTrajectory();
  nan_point.pos_pts[1].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(nan_point).error,
            ego_planner::swarm_validation::ValidationError::kControlPointNonFinite);

  auto nan_knot = validTrajectory();
  nan_knot.knots[2] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(nan_knot).error,
            ego_planner::swarm_validation::ValidationError::kKnotNonFinite);

  auto decreasing = validTrajectory();
  decreasing.knots[3] = 1.5;
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(decreasing).error,
            ego_planner::swarm_validation::ValidationError::kKnotNotStrictlyIncreasing);

  auto zero_duration = validTrajectory();
  zero_duration.knots[4] = zero_duration.knots[3];
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(zero_duration).error,
            ego_planner::swarm_validation::ValidationError::kInvalidDuration);

  auto huge_duration = validTrajectory();
  huge_duration.knots[4] = 304.0;
  huge_duration.knots[5] = 305.0;
  huge_duration.knots[6] = 306.0;
  huge_duration.knots[7] = 307.0;
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(huge_duration).error,
            ego_planner::swarm_validation::ValidationError::kInvalidDuration);
}

TEST(SwarmBsplineValidator, ValidatesEveryMultiEntryAndYawFields)
{
  FakeMultiBsplines multi;
  multi.drone_id_from = 1;
  multi.traj = {validTrajectory(0), validTrajectory(1)};
  multi.traj[1].pos_pts[0].z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(multi).error,
            ego_planner::swarm_validation::ValidationError::kControlPointNonFinite);

  auto yaw_mismatch = validTrajectory();
  yaw_mismatch.yaw_pts = {0.0};
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(yaw_mismatch).error,
            ego_planner::swarm_validation::ValidationError::kYawCountMismatch);

  auto yaw_nan = validTrajectory();
  yaw_nan.yaw_pts = {0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN()};
  yaw_nan.yaw_dt = 0.1;
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(yaw_nan).error,
            ego_planner::swarm_validation::ValidationError::kYawNonFinite);

  auto yaw_dt_nan = validTrajectory();
  yaw_dt_nan.yaw_pts.assign(yaw_dt_nan.pos_pts.size(), 0.0);
  yaw_dt_nan.yaw_dt = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(yaw_dt_nan).error,
            ego_planner::swarm_validation::ValidationError::kYawDt);

  auto yaw_dt_zero = validTrajectory();
  yaw_dt_zero.yaw_pts.assign(yaw_dt_zero.pos_pts.size(), 0.0);
  yaw_dt_zero.yaw_dt = 0.0;
  EXPECT_EQ(ego_planner::swarm_validation::validateBspline(yaw_dt_zero).error,
            ego_planner::swarm_validation::ValidationError::kYawDt);
}

TEST(SwarmBsplineValidator, RejectsEmptyOrOversizedMultiMessage)
{
  FakeMultiBsplines empty;
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(empty).error,
            ego_planner::swarm_validation::ValidationError::kMultiEmpty);

  FakeMultiBsplines oversized;
  oversized.drone_id_from = ego_planner::swarm_validation::kMaxSwarmId;
  oversized.traj.resize(ego_planner::swarm_validation::kMaxSwarmTrajectories + 1);
  EXPECT_EQ(ego_planner::swarm_validation::validateMultiBsplines(oversized).error,
            ego_planner::swarm_validation::ValidationError::kMultiTooLarge);
}

}  // namespace
