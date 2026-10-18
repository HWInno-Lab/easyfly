#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include <plan_manage/collision_safety.h>
#include <plan_manage/waypoint_admission.h>

namespace
{

using ego_planner::collision_safety::emergencyWindowImminent;
using ego_planner::collision_safety::sanitizeEmergencyTime;
using namespace ego_planner::waypoint_admission;

TEST(WaypointAdmission, AcceptsOnlyOdomFrameSpellings)
{
  EXPECT_TRUE(validOdomFrame("odom"));
  EXPECT_TRUE(validOdomFrame("/odom"));
  EXPECT_FALSE(validOdomFrame(""));
  EXPECT_FALSE(validOdomFrame("map"));
  EXPECT_FALSE(validOdomFrame("/map"));
  EXPECT_FALSE(validOdomFrame("odom/base_link"));
}

TEST(WaypointAdmission, RejectsZeroAndMalformedStampFields)
{
  EXPECT_FALSE(validStampFields(0, 0));
  EXPECT_TRUE(validStampFields(0, 1));
  EXPECT_TRUE(validStampFields(1, 0));
  EXPECT_FALSE(validStampFields(-1, 0));
  EXPECT_FALSE(validStampFields(1, 1000000000U));
}

TEST(WaypointAdmission, RejectsStaleFutureAndNonFiniteTimestampWindows)
{
  EXPECT_TRUE(validTimestampWindow(0.1, 1.0, 0.2));
  EXPECT_TRUE(validTimestampWindow(-0.2, 1.0, 0.2));
  EXPECT_FALSE(validTimestampWindow(-0.200001, 1.0, 0.2));
  EXPECT_FALSE(validTimestampWindow(1.000001, 1.0, 0.2));
  EXPECT_FALSE(validTimestampWindow(
    std::numeric_limits<double>::quiet_NaN(), 1.0, 0.2));
  EXPECT_FALSE(validTimestampWindow(0.0, 0.0, 0.2));
}

TEST(WaypointAdmission, RejectsRegressedWaypointStamp)
{
  EXPECT_TRUE(nonDecreasingStamp(10, 0, false));
  EXPECT_TRUE(nonDecreasingStamp(10, 10, true));
  EXPECT_TRUE(nonDecreasingStamp(11, 10, true));
  EXPECT_FALSE(nonDecreasingStamp(9, 10, true));
  EXPECT_FALSE(nonDecreasingStamp(0, 10, true));
}

TEST(WaypointAdmission, ExplicitAndLowAltitudeGoalsHaveSafeSemantics)
{
  double selected = 0.0;
  EXPECT_TRUE(selectGoalAltitude(1.5, 1.49, selected));
  EXPECT_DOUBLE_EQ(selected, 1.5);

  // A 2-D RViz goal carries z=0; retain the current stable hover altitude.
  EXPECT_TRUE(selectGoalAltitude(0.0, 1.49, selected));
  EXPECT_DOUBLE_EQ(selected, 1.49);

  EXPECT_FALSE(selectGoalAltitude(0.0, 0.49, selected));
  EXPECT_FALSE(selectGoalAltitude(
    std::numeric_limits<double>::quiet_NaN(), 1.49, selected));
}

TEST(WaypointAdmission, PositionAndFutureToleranceAreFailClosed)
{
  EXPECT_TRUE(validFinitePosition(1.0, 2.0, 3.0));
  EXPECT_FALSE(validFinitePosition(
    std::numeric_limits<double>::quiet_NaN(), 2.0, 3.0));
  EXPECT_DOUBLE_EQ(boundedFutureTolerance(0.3), 0.3);
  EXPECT_DOUBLE_EQ(boundedFutureTolerance(-1.0), 0.2);
  EXPECT_DOUBLE_EQ(boundedFutureTolerance(6.0), 0.2);
}

TEST(CollisionSafety, EmergencyTimeAndBoundaryAreFailClosed)
{
  EXPECT_DOUBLE_EQ(sanitizeEmergencyTime(0.8), 0.8);
  EXPECT_DOUBLE_EQ(sanitizeEmergencyTime(0.0), 1.0);
  EXPECT_DOUBLE_EQ(sanitizeEmergencyTime(-1.0), 1.0);
  EXPECT_DOUBLE_EQ(sanitizeEmergencyTime(
    std::numeric_limits<double>::quiet_NaN()), 1.0);
  EXPECT_DOUBLE_EQ(sanitizeEmergencyTime(61.0), 1.0);

  EXPECT_FALSE(emergencyWindowImminent(1.01, 1.0));
  EXPECT_TRUE(emergencyWindowImminent(1.0, 1.0));
  EXPECT_TRUE(emergencyWindowImminent(0.1, 1.0));
  EXPECT_TRUE(emergencyWindowImminent(
    std::numeric_limits<double>::quiet_NaN(), 1.0));
  EXPECT_TRUE(emergencyWindowImminent(1.0, 0.0));
}

TEST(CollisionSafety, InvalidTrajectoryDurationIsRejected)
{
  EXPECT_TRUE(ego_planner::collision_safety::validTrajectoryDuration(0.001));
  EXPECT_TRUE(ego_planner::collision_safety::validTrajectoryDuration(300.0));
  EXPECT_FALSE(ego_planner::collision_safety::validTrajectoryDuration(0.0));
  EXPECT_FALSE(ego_planner::collision_safety::validTrajectoryDuration(-1.0));
  EXPECT_FALSE(ego_planner::collision_safety::validTrajectoryDuration(300.001));
  EXPECT_FALSE(ego_planner::collision_safety::validTrajectoryDuration(
    std::numeric_limits<double>::quiet_NaN()));
}

}  // namespace
