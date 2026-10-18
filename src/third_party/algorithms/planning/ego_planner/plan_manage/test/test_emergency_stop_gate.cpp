#include <gtest/gtest.h>
#include <plan_manage/navigation_session_contract.h>

#include <limits>

#include <plan_manage/emergency_stop_gate.h>

namespace ego_planner
{

TEST(NavigationSession, RuntimeCollisionScanRequiresPublishedCurrentSession)
{
  EXPECT_FALSE(runtimeTrajectoryBelongsToSession(true, "old", "new"));
  EXPECT_FALSE(runtimeTrajectoryBelongsToSession(true, "", "new"));
  EXPECT_FALSE(runtimeTrajectoryBelongsToSession(true, "", ""));
  EXPECT_FALSE(runtimeTrajectoryBelongsToSession(true, "old", ""));
  EXPECT_TRUE(runtimeTrajectoryBelongsToSession(true, "new", "new"));
  EXPECT_TRUE(runtimeTrajectoryBelongsToSession(false, "", ""));
}

TEST(EmergencyRecoveryGate, PreviousSessionAckCannotRecoverNewSession) {
  EmergencyRecoveryGate gate;
  gate.beginStop(7);
  ASSERT_TRUE(gate.markStopPublished(true, 1.0, 42));
  const auto deliver = [&](const std::string &session) {
    return matchesNavigationSession(session, "new-session") &&
      gate.acknowledgeStop(42, 11, true, 1.0, 1.0);
  };
  EXPECT_FALSE(deliver("old-session"));
  EXPECT_FALSE(deliver(""));
  EXPECT_FALSE(gate.downstreamAcknowledged());
  EXPECT_FALSE(gate.observe(1.1, true, true, 0.0));
  EXPECT_TRUE(deliver("new-session"));
  EXPECT_TRUE(gate.downstreamAcknowledged());
}

TEST(EmergencyStopGate, ArmsAndPublishesExactlyOnce)
{
  EmergencyStopGate gate;

  EXPECT_EQ(gate.request(false), EmergencyStopGate::RequestResult::kArmed);
  EXPECT_TRUE(gate.pending());

  // A second collision/depth callback while stopped is a no-op.  The first
  // pending stop must remain available to the FSM timer.
  EXPECT_EQ(gate.request(true), EmergencyStopGate::RequestResult::kAlreadyStopped);
  EXPECT_TRUE(gate.pending());
  EXPECT_TRUE(gate.consume(true));
  EXPECT_FALSE(gate.pending());
  EXPECT_FALSE(gate.consume(true));
}

TEST(EmergencyStopGate, DoesNotConsumeOutsideEmergencyState)
{
  EmergencyStopGate gate;
  EXPECT_EQ(gate.request(false), EmergencyStopGate::RequestResult::kArmed);
  EXPECT_FALSE(gate.consume(false));
  EXPECT_TRUE(gate.pending());
  EXPECT_TRUE(gate.consume(true));
}

TEST(EmergencyStopGate, SafetyStateBlocksCollisionPlanningAndExecutionCommit)
{
  EmergencyStopGate gate;
  EXPECT_TRUE(gate.shouldReturnFromCollisionCallback(true));
  EXPECT_FALSE(gate.canCommitExecution(true));
  EXPECT_FALSE(gate.canTransitionToReplan(true));
  EXPECT_FALSE(gate.shouldReturnFromCollisionCallback(false));
  EXPECT_TRUE(gate.canCommitExecution(false));
  EXPECT_TRUE(gate.canTransitionToReplan(false));
}

TEST(EmergencyStopGate, RemoteCollisionBroadcastCannotReplanWhileStopped)
{
  EmergencyStopGate gate;

  // This is the decision made after a remote trajectory has been committed
  // and collision checking reports a conflict.  A previously entered stop
  // must remain latched; receiving the broadcast cannot reopen REPLAN_TRAJ.
  const bool stopped = true;
  EXPECT_FALSE(gate.canTransitionToReplan(stopped));
  EXPECT_TRUE(gate.shouldReturnFromCollisionCallback(stopped));
}

TEST(EmergencyStopGate, ReachedGoalWaitsForFreshManualTarget)
{
  const auto action = EmergencyStopGate::recoveryAction(
    true, true, true, 0.25, 1.0);
  EXPECT_EQ(action, EmergencyStopGate::RecoveryAction::kWaitForTarget);
}

TEST(EmergencyStopGate, UnreachedGoalResumesPlanningAfterStop)
{
  EXPECT_EQ(
    EmergencyStopGate::recoveryAction(true, true, true, 2.0, 1.0),
    EmergencyStopGate::RecoveryAction::kResumePlanning);
}

TEST(EmergencyStopGate, RecoveryRequiresFreshCloudAndFiniteTolerance)
{
  EXPECT_EQ(
    EmergencyStopGate::recoveryAction(true, true, false, 0.1, 1.0),
    EmergencyStopGate::RecoveryAction::kHold);
  EXPECT_EQ(
    EmergencyStopGate::recoveryAction(false, true, true, 0.1, 1.0),
    EmergencyStopGate::RecoveryAction::kHold);
  EXPECT_EQ(
    EmergencyStopGate::recoveryAction(
      true, true, true, 0.1, std::numeric_limits<double>::quiet_NaN()),
    EmergencyStopGate::RecoveryAction::kResumePlanning);
}

TEST(EmergencyStopGate, NewManualGoalCanResumeAfterReachedGoalIsCleared)
{
  // Model the FSM sequence without ROS: the old target is cleared on the
  // first decision, then waypointCallback installs a new, non-terminal goal.
  bool have_target = true;
  EXPECT_EQ(
    EmergencyStopGate::recoveryAction(true, have_target, true, 0.2, 1.0),
    EmergencyStopGate::RecoveryAction::kWaitForTarget);
  have_target = false;
  EXPECT_EQ(
    EmergencyStopGate::recoveryAction(true, have_target, true, 0.0, 1.0),
    EmergencyStopGate::RecoveryAction::kResumePlanning);
  have_target = true;
  EXPECT_EQ(
    EmergencyStopGate::recoveryAction(true, have_target, true, 2.0, 1.0),
    EmergencyStopGate::RecoveryAction::kResumePlanning);
}

TEST(EmergencyRecoveryGate, RequiresPublishedStopConsecutiveSamplesAndHold)
{
  EmergencyRecoveryGate::Params params;
  params.stable_cycles = 3;
  params.hold_seconds = 0.5;
  params.max_speed = 0.1;
  params.max_clock_jump = 1.0;
  params.ack_timeout = 1.0;
  EmergencyRecoveryGate gate(params);
  gate.beginStop(7, 10);

  // Low velocity before successful stop publication cannot count.
  EXPECT_FALSE(gate.observe(0.0, true, true, 0.0));
  EXPECT_FALSE(gate.markStopPublished(false, 0.0));
  EXPECT_FALSE(gate.observe(0.1, true, true, 0.0));

  ASSERT_TRUE(gate.markStopPublished(true, 1.0, 42));
  ASSERT_TRUE(gate.acknowledgeStop(42, 11, true, 1.0, 1.0));
  EXPECT_FALSE(gate.observe(1.1, true, true, 0.0));
  EXPECT_FALSE(gate.observe(1.2, true, true, 0.0));
  EXPECT_FALSE(gate.observe(1.3, true, true, 0.0));
  // Three cycles have elapsed, but the explicit hold time has not.
  EXPECT_FALSE(gate.observe(1.4, true, true, 0.0));
  EXPECT_FALSE(gate.observe(1.5, true, true, 0.0));
  EXPECT_TRUE(gate.observe(1.61, true, true, 0.0));
}

TEST(EmergencyRecoveryGate, FreshnessOrSpeedBreaksConsecutiveEvidence)
{
  EmergencyRecoveryGate::Params params;
  params.stable_cycles = 2;
  params.hold_seconds = 0.1;
  params.max_speed = 0.1;
  EmergencyRecoveryGate gate(params);
  gate.beginStop(1);
  ASSERT_TRUE(gate.markStopPublished(true, 2.0, 7));
  ASSERT_TRUE(gate.acknowledgeStop(7, 1, true, 2.0, 2.0));
  EXPECT_FALSE(gate.observe(2.05, true, true, 0.0));
  EXPECT_FALSE(gate.observe(2.10, false, true, 0.0));
  EXPECT_EQ(gate.stableCyclesSeen(), 0);
  EXPECT_FALSE(gate.observe(2.15, true, true, 0.2));
  EXPECT_EQ(gate.stableCyclesSeen(), 0);
  EXPECT_FALSE(gate.observe(2.20, true, true, 0.0));
  EXPECT_FALSE(gate.observe(2.25, true, true, 0.0));
  EXPECT_TRUE(gate.observe(2.31, true, true, 0.0));
}

TEST(EmergencyRecoveryGate, ClockJumpRequiresStopRepublish)
{
  EmergencyRecoveryGate::Params params;
  params.stable_cycles = 1;
  params.hold_seconds = 0.2;
  params.max_clock_jump = 0.5;
  EmergencyRecoveryGate gate(params);
  gate.beginStop(2);
  ASSERT_TRUE(gate.markStopPublished(true, 10.0, 8));
  ASSERT_TRUE(gate.acknowledgeStop(8, 1, true, 10.0, 10.0));
  EXPECT_FALSE(gate.observe(10.1, true, true, 0.0));
  EXPECT_FALSE(gate.observe(9.0, true, true, 0.0));
  EXPECT_TRUE(gate.needsStopPublication());
  EXPECT_FALSE(gate.observe(9.1, true, true, 0.0));
  ASSERT_TRUE(gate.markStopPublished(true, 9.2, 9));
  ASSERT_TRUE(gate.acknowledgeStop(9, 2, true, 9.2, 9.2));
  EXPECT_FALSE(gate.observe(9.25, true, true, 0.0));
  EXPECT_TRUE(gate.observe(9.46, true, true, 0.0));
}

TEST(EmergencyRecoveryGate, NewGoalGenerationCannotBeClearedAsOldGoal)
{
  EXPECT_TRUE(EmergencyRecoveryGate::shouldClearReachedTarget(
    4, 4, true, 0.2, 1.0));
  EXPECT_FALSE(EmergencyRecoveryGate::shouldClearReachedTarget(
    4, 5, true, 0.2, 1.0));
  EXPECT_FALSE(EmergencyRecoveryGate::shouldClearReachedTarget(
    4, 4, true, 2.0, 1.0));
}

TEST(EmergencyRecoveryGate, RequiresFreshMatchingDownstreamAck)
{
  EmergencyRecoveryGate gate;
  gate.beginStop(3, 20);
  ASSERT_TRUE(gate.markStopPublished(true, 5.0, 17));
  EXPECT_FALSE(gate.acknowledgeStop(16, 21, true, 5.0, 5.0));
  EXPECT_FALSE(gate.acknowledgeStop(17, 20, true, 5.0, 5.0));
  EXPECT_FALSE(gate.acknowledgeStop(17, 21, false, 5.0, 5.0));
  EXPECT_TRUE(gate.acknowledgeStop(17, 21, true, 5.0, 5.0));
  EXPECT_FALSE(gate.acknowledgeStop(17, 22, true, 5.0, 5.0));
  EXPECT_FALSE(gate.observe(5.1, true, true, 0.0));
}

TEST(EmergencyRecoveryGate, DedicatedAckSourceRejectsNoFakeOrMultiplePublishers)
{
  EXPECT_TRUE(EmergencyRecoveryGate::validTrajectoryAckSource(
    true, true, true, true, true));
  // No discovered publisher, more than one publisher, a different endpoint
  // GID/node/type, or a spoofed source field must all keep EMERGENCY_STOP.
  EXPECT_FALSE(EmergencyRecoveryGate::validTrajectoryAckSource(
    false, true, true, true, true));
  EXPECT_FALSE(EmergencyRecoveryGate::validTrajectoryAckSource(
    true, false, true, true, true));
  EXPECT_FALSE(EmergencyRecoveryGate::validTrajectoryAckSource(
    true, true, false, true, true));
  EXPECT_FALSE(EmergencyRecoveryGate::validTrajectoryAckSource(
    true, true, true, false, true));
  EXPECT_FALSE(EmergencyRecoveryGate::validTrajectoryAckSource(
    true, true, true, true, false));
}

TEST(EmergencyRecoveryGate, DedicatedAckSourceRequiresCompleteNodeFqn)
{
  EXPECT_TRUE(EmergencyRecoveryGate::endpointFqnMatches(
    "traj_server", "/drone_0", "/drone_0/traj_server"));
  EXPECT_FALSE(EmergencyRecoveryGate::endpointFqnMatches(
    "traj_server", "/drone_1", "/drone_0/traj_server"));
  EXPECT_FALSE(EmergencyRecoveryGate::endpointFqnMatches(
    "other", "/drone_0", "/drone_0/traj_server"));
  EXPECT_TRUE(EmergencyRecoveryGate::validTrajectoryAckSource(
    true, true, "traj_server", "/drone_0", "/drone_0/traj_server",
    "traj_utils/msg/TrajectoryAck", "/drone_0/traj_server"));
  EXPECT_FALSE(EmergencyRecoveryGate::validTrajectoryAckSource(
    true, true, "traj_server", "/drone_0", "/drone_0/traj_server",
    "traj_utils/msg/TrajectoryAck", "/traj_server"));
  EXPECT_FALSE(EmergencyRecoveryGate::validTrajectoryAckSource(
    true, true, "traj_server", "/drone_1", "/drone_0/traj_server",
    "traj_utils/msg/TrajectoryAck", "/drone_0/traj_server"));
}

TEST(EmergencyRecoveryGate, AckRejectsZeroOldFutureAndRollbackStamps)
{
  EmergencyRecoveryGate gate;
  gate.beginStop(1, 10);
  ASSERT_TRUE(gate.markStopPublished(true, 5.0, 17));
  EXPECT_FALSE(gate.acknowledgeStop(17, 11, true, 0.0, 5.0));
  EXPECT_FALSE(gate.acknowledgeStop(17, 12, true, 4.9, 5.0));
  EXPECT_FALSE(gate.acknowledgeStop(17, 13, true, 5.1, 5.0));
  EXPECT_TRUE(gate.acknowledgeStop(17, 14, true, 5.0, 5.0));
  EXPECT_FALSE(gate.acknowledgeStop(17, 15, true, 4.0, 5.0));
}

TEST(EmergencyRecoveryGate, MissingAckCausesBoundedRetry)
{
  EmergencyRecoveryGate::Params params;
  params.ack_retry_period = 0.5;
  EmergencyRecoveryGate gate(params);
  gate.beginStop(1);
  ASSERT_TRUE(gate.markStopPublished(true, 1.0, 4));
  EXPECT_FALSE(gate.stopPublicationDue(1.49));
  EXPECT_TRUE(gate.stopPublicationDue(1.5));
  ASSERT_TRUE(gate.markStopPublished(true, 1.5, 5));
  EXPECT_FALSE(gate.stopPublicationDue(1.9));
  EXPECT_TRUE(gate.acknowledgeStop(5, 1, true, 1.9, 1.9));
  EXPECT_FALSE(gate.stopPublicationDue(2.2));
}

TEST(EmergencyRecoveryGate, AckIsOneShotAndDoesNotExpireIntoRetry)
{
  EmergencyRecoveryGate::Params params;
  params.stable_cycles = 2;
  params.hold_seconds = 0.1;
  params.ack_timeout = 0.5;
  params.max_clock_jump = 60.0;
  EmergencyRecoveryGate gate(params);
  gate.beginStop(1);
  ASSERT_TRUE(gate.markStopPublished(true, 1.0, 4));
  ASSERT_TRUE(gate.acknowledgeStop(4, 1, true, "gid-a", 1.0, 1.0));

  // The activation receipt remains valid after its freshness window; it is
  // not a heartbeat and must not trigger another stop publication.
  EXPECT_FALSE(gate.stopPublicationDue(71.0));
  // Advance with ordinary FSM-sized samples so the clock-jump guard is not
  // the subject of this one-shot ACK test.
  EXPECT_FALSE(gate.observe(50.0, true, true, 0.0, true));
  EXPECT_TRUE(gate.observe(50.2, true, true, 0.0, true));
  EXPECT_TRUE(gate.observe(70.2, true, true, 0.0, true));
}

TEST(EmergencyRecoveryGate, LostAckPublisherForcesNewStopAttempt)
{
  EmergencyRecoveryGate gate;
  gate.beginStop(1);
  ASSERT_TRUE(gate.markStopPublished(true, 1.0, 4));
  ASSERT_TRUE(gate.acknowledgeStop(4, 1, true, "gid-a", 1.0, 1.0));
  EXPECT_FALSE(gate.observe(1.1, true, true, 0.0, false));
  EXPECT_FALSE(gate.downstreamAcknowledged());
  EXPECT_TRUE(gate.needsStopPublication());
  EXPECT_FALSE(gate.stopPublicationDue(1.1));
  EXPECT_TRUE(gate.stopPublicationDue(1.5));
}

TEST(EmergencyRecoveryGate, StableHoldStartsAtFirstContinuousStableSample)
{
  EmergencyRecoveryGate::Params params;
  params.stable_cycles = 2;
  params.hold_seconds = 0.5;
  params.ack_timeout = 1.0;
  EmergencyRecoveryGate gate(params);
  gate.beginStop(1);
  ASSERT_TRUE(gate.markStopPublished(true, 1.0, 2));
  ASSERT_TRUE(gate.acknowledgeStop(2, 1, true, 1.0, 1.0));
  EXPECT_FALSE(gate.observe(1.1, true, true, 0.0));
  EXPECT_FALSE(gate.observe(1.2, false, true, 0.0));
  EXPECT_FALSE(gate.observe(1.3, true, true, 0.0));
  EXPECT_FALSE(gate.observe(1.7, true, true, 0.0));
  EXPECT_TRUE(gate.observe(1.81, true, true, 0.0));
}

TEST(EmergencyRecoveryGate, InvalidSafetyParametersUseConservativeDefaults)
{
  EmergencyRecoveryGate::Params params;
  params.stable_cycles = 0;
  params.hold_seconds = 1000.0;
  params.max_speed = std::numeric_limits<double>::infinity();
  params.max_clock_jump = -1.0;
  params.ack_timeout = 100.0;
  params.ack_retry_period = 0.0;
  params.ack_retry_max_period = 0.0;
  EmergencyRecoveryGate gate(params);
  const auto effective = gate.params();
  EXPECT_EQ(effective.stable_cycles, 5);
  EXPECT_DOUBLE_EQ(effective.hold_seconds, 0.5);
  EXPECT_DOUBLE_EQ(effective.max_speed, 0.1);
  EXPECT_DOUBLE_EQ(effective.max_clock_jump, 1.0);
  EXPECT_DOUBLE_EQ(effective.ack_timeout, 0.5);
  EXPECT_DOUBLE_EQ(effective.ack_retry_period, 0.5);
  EXPECT_DOUBLE_EQ(effective.ack_retry_max_period, 5.0);
}

TEST(EmergencyRecoveryGate, AckRetryUsesBoundedExponentialBackoff)
{
  EmergencyRecoveryGate::Params params;
  params.ack_retry_period = 0.5;
  params.ack_retry_max_period = 1.0;
  EmergencyRecoveryGate gate(params);
  gate.beginStop(1);
  ASSERT_TRUE(gate.markStopPublished(true, 1.0, 4));
  EXPECT_TRUE(gate.stopPublicationDue(1.5));
  ASSERT_TRUE(gate.markStopPublished(true, 1.5, 5));
  EXPECT_FALSE(gate.stopPublicationDue(2.0));
  EXPECT_TRUE(gate.stopPublicationDue(2.5));
  ASSERT_TRUE(gate.markStopPublished(true, 2.5, 6));
  EXPECT_FALSE(gate.stopPublicationDue(3.4));
  EXPECT_TRUE(gate.stopPublicationDue(3.5));
}

TEST(EmergencyRecoveryGate, ActivationSequenceIsMonotonicPerPublisherAndAttempt)
{
  EmergencyRecoveryGate gate;
  gate.beginStop(1);
  ASSERT_TRUE(gate.markStopPublished(true, 1.0, 4));
  EXPECT_TRUE(gate.acknowledgeStop(4, 7, true, "gid-a", 1.0, 1.0));
  // Same publisher replay, rollback, and duplicate sequence are rejected.
  EXPECT_FALSE(gate.acknowledgeStop(4, 7, true, "gid-a", 1.0, 1.0));
  EXPECT_FALSE(gate.acknowledgeStop(4, 6, true, "gid-a", 1.0, 1.0));
  // A changed endpoint cannot silently replace the publisher within this
  // attempt, even if its sequence starts at one after a restart.
  EXPECT_FALSE(gate.acknowledgeStop(4, 1, true, "gid-b", 1.0, 1.0));

  // A new stop publication starts a new attempt. A new endpoint GID may then
  // bind its reset sequence; the old GID remains globally replay-protected.
  ASSERT_TRUE(gate.markStopPublished(true, 2.0, 5));
  EXPECT_TRUE(gate.acknowledgeStop(5, 1, true, "gid-b", 2.0, 2.0));
  EXPECT_FALSE(gate.acknowledgeStop(5, 1, true, "gid-b", 2.0, 2.0));
  ASSERT_TRUE(gate.markStopPublished(true, 3.0, 6));
  EXPECT_FALSE(gate.acknowledgeStop(6, 7, true, "gid-a", 3.0, 3.0));
}

TEST(EmergencyRecoveryGate, OdomDepthTimeoutNeedsFreshSensorsAndNewGeneration)
{
  EXPECT_FALSE(EmergencyRecoveryGate::canRecoverOdomDepthTimeout(
    true, true, true, true, 4, 4));
  EXPECT_FALSE(EmergencyRecoveryGate::canRecoverOdomDepthTimeout(
    true, false, true, true, 4, 5));
  EXPECT_FALSE(EmergencyRecoveryGate::canRecoverOdomDepthTimeout(
    true, true, true, false, 4, 5));
  EXPECT_TRUE(EmergencyRecoveryGate::canRecoverOdomDepthTimeout(
    true, true, true, true, 4, 5));
  EXPECT_TRUE(EmergencyRecoveryGate::canRecoverOdomDepthTimeout(
    false, false, false, false, 4, 4));
}

}  // namespace ego_planner
