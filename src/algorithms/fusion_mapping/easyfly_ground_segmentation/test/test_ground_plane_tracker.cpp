// Copyright 2026 EasyFly Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cmath>

#include "easyfly_ground_segmentation/ground_plane_tracker.hpp"

namespace easyfly::ground
{
namespace
{

GroundPlaneCandidate horizontalCandidate(const double z, const std::size_t inliers = 800)
{
  GroundPlaneCandidate candidate;
  candidate.model.normal = {0.0, 0.0, 1.0};
  candidate.model.offset = -z;
  candidate.inlier_count = inliers;
  candidate.sample_count = 1000;
  return candidate;
}

GroundVehiclePose vehicle(const double z = 2.5, const bool armed = false)
{
  GroundVehiclePose pose;
  pose.x = 0.0;
  pose.y = 0.0;
  pose.z = z;
  pose.connected = true;
  pose.pose_valid = true;
  pose.armed = armed;
  pose.state_age_s = 0.0;
  return pose;
}

GroundPlaneTracker initializedTracker()
{
  GroundPlaneTracker tracker;
  const auto candidate = horizontalCandidate(-0.67);
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kPending);
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kPending);
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kInitialized);
  EXPECT_TRUE(tracker.initialized());
  return tracker;
}

TEST(GroundPlaneTrackerTest, RequiresConsecutiveConsistentInitialization)
{
  GroundPlaneTracker tracker;
  const auto candidate = horizontalCandidate(-0.67);

  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kPending);
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kPending);
  EXPECT_FALSE(tracker.initialized());
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kInitialized);
  EXPECT_TRUE(tracker.initialized());
  EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), -0.67, 1e-6);
}

TEST(GroundPlaneTrackerTest, RejectsPlaneAtAircraftHeight)
{
  auto tracker = initializedTracker();
  const double trusted_z = tracker.model().zAt(0.0, 0.0);

  // This reproduces the G7-N +2.42m degenerate RANSAC plane while the
  // aircraft is at 2.5m.  A near-body-height candidate must not replace the
  // trusted ground model.
  EXPECT_EQ(tracker.update(horizontalCandidate(2.42), vehicle()), GroundPlaneUpdate::kRejected);
  EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), trusted_z, 1e-6);
}

TEST(GroundPlaneTrackerTest, AcceptsObservedLandedClearanceAboveMinimum)
{
  GroundPlaneTracker tracker;

  // Direct landed traces are 0.053-0.054m.  Use a small margin above the
  // configured 0.03m floor so this boundary check is not float fragile.
  const auto candidate = horizontalCandidate(2.4461);  // 0.0539m clearance
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kPending);
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kPending);
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kInitialized);
  EXPECT_TRUE(tracker.initialized());
}

TEST(GroundPlaneTrackerTest, RejectsClearanceBelowLandedMinimum)
{
  GroundPlaneTracker tracker;

  // This remains just below the 0.03m minimum and covers the near-zero
  // false-plane boundary below the allowed landed gap.
  const auto candidate = horizontalCandidate(2.4701);  // 0.0299m clearance
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kRejected);
    EXPECT_FALSE(tracker.initialized());
  }
}

TEST(GroundPlaneTrackerTest, RejectsZeroClearanceInitialization)
{
  GroundPlaneTracker tracker;
  const auto candidate = horizontalCandidate(2.5);

  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kRejected);
    EXPECT_FALSE(tracker.initialized());
  }
}

TEST(GroundPlaneTrackerTest, SingleInnovationUsesHistoricalModelForClassification)
{
  auto tracker = initializedTracker();
  const double trusted_z = tracker.model().zAt(0.0, 0.0);

  // Physically below the vehicle, but far enough from the trusted model to
  // be an innovation outlier.  It must not supply its own inliers.
  EXPECT_EQ(tracker.update(horizontalCandidate(1.65), vehicle()), GroundPlaneUpdate::kRejected);
  EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), trusted_z, 1e-6);
  EXPECT_TRUE(tracker.isGroundPoint(0.0, 0.0, trusted_z, 0.12));
  EXPECT_FALSE(tracker.isGroundPoint(0.0, 0.0, 0.50, 0.12));
  EXPECT_NEAR(tracker.heightAboveGround(0.0, 0.0, 0.50), 1.17, 1e-6);
}

TEST(GroundPlaneTrackerTest, SmoothsSlowGroundChange)
{
  auto tracker = initializedTracker();
  EXPECT_EQ(tracker.update(horizontalCandidate(-0.55), vehicle()), GroundPlaneUpdate::kAccepted);

  // EMA moves toward the new plane without jumping all the way in one frame.
  const double smoothed_z = tracker.model().zAt(0.0, 0.0);
  EXPECT_GT(smoothed_z, -0.67);
  EXPECT_LT(smoothed_z, -0.55);
}

TEST(GroundPlaneTrackerTest, FreezesTrustedModelWhileArmed)
{
  auto tracker = initializedTracker();
  const double trusted_z = tracker.model().zAt(0.0, 0.0);

  // A plausible but different RANSAC plane must not move the trusted model
  // while the aircraft is armed; this is an expected freeze, not rejection.
  EXPECT_EQ(
    tracker.update(horizontalCandidate(-0.20), vehicle(2.5, true)),
    GroundPlaneUpdate::kFrozen);
  EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), trusted_z, 1e-6);
}

TEST(GroundPlaneTrackerTest, FreezesTrustedModelWhenDisconnected)
{
  auto tracker = initializedTracker();
  const double trusted_z = tracker.model().zAt(0.0, 0.0);
  auto state = vehicle();
  state.connected = false;

  EXPECT_EQ(
    tracker.update(horizontalCandidate(-0.20), state),
    GroundPlaneUpdate::kFrozen);
  EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), trusted_z, 1e-6);
}

TEST(GroundPlaneTrackerTest, FreezesTrustedModelWhenPoseIsInvalid)
{
  auto tracker = initializedTracker();
  const double trusted_z = tracker.model().zAt(0.0, 0.0);
  auto state = vehicle();
  state.pose_valid = false;

  EXPECT_EQ(
    tracker.update(horizontalCandidate(-0.20), state),
    GroundPlaneUpdate::kFrozen);
  EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), trusted_z, 1e-6);
}

TEST(GroundPlaneTrackerTest, FreezesTrustedModelWhenStateIsStale)
{
  auto tracker = initializedTracker();
  const double trusted_z = tracker.model().zAt(0.0, 0.0);
  auto state = vehicle();
  state.state_age_s = 2.0;

  EXPECT_EQ(
    tracker.update(horizontalCandidate(-0.20), state),
    GroundPlaneUpdate::kFrozen);
  EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), trusted_z, 1e-6);
}

TEST(GroundPlaneTrackerTest, FreshDisarmedStateRecoversAfterStaleFreeze)
{
  auto tracker = initializedTracker();
  const double trusted_z = tracker.model().zAt(0.0, 0.0);
  auto stale = vehicle();
  stale.state_age_s = 2.0;

  EXPECT_EQ(
    tracker.update(horizontalCandidate(-0.55), stale),
    GroundPlaneUpdate::kFrozen);
  EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), trusted_z, 1e-6);

  EXPECT_EQ(
    tracker.update(horizontalCandidate(-0.55), vehicle()),
    GroundPlaneUpdate::kAccepted);
  EXPECT_GT(tracker.model().zAt(0.0, 0.0), trusted_z);
}

TEST(GroundPlaneTrackerTest, DisarmedStateCanUpdateAfterArmedFreeze)
{
  auto tracker = initializedTracker();
  const double trusted_z = tracker.model().zAt(0.0, 0.0);

  EXPECT_EQ(
    tracker.update(horizontalCandidate(-0.55), vehicle(2.5, true)),
    GroundPlaneUpdate::kFrozen);
  EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), trusted_z, 1e-6);

  // Once disarmed, the existing innovation/smoothing policy is restored.
  EXPECT_EQ(
    tracker.update(horizontalCandidate(-0.55), vehicle(2.5, false)),
    GroundPlaneUpdate::kAccepted);
  EXPECT_GT(tracker.model().zAt(0.0, 0.0), trusted_z);
  EXPECT_LT(tracker.model().zAt(0.0, 0.0), -0.55);
}

TEST(GroundPlaneTrackerTest, PersistentAircraftHeightPlanesNeverReplaceModel)
{
  auto tracker = initializedTracker();
  const double trusted_z = tracker.model().zAt(0.0, 0.0);

  // Both planes are physically below the vehicle, but there is no independent
  // spatial continuity evidence that either one is real terrain.  They must
  // remain rejected even when repeated for many frames.
  for (const double z : {1.50, 2.00}) {
    for (int i = 0; i < 12; ++i) {
      EXPECT_EQ(tracker.update(horizontalCandidate(z), vehicle()), GroundPlaneUpdate::kRejected);
      EXPECT_NEAR(tracker.model().zAt(0.0, 0.0), trusted_z, 1e-6);
    }
  }
}

TEST(GroundPlaneTrackerTest, CannotInitializeFromBadCandidates)
{
  GroundPlaneTracker tracker;
  const auto aircraft_plane = horizontalCandidate(1.50);

  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(tracker.update(aircraft_plane, vehicle(2.5, true)), GroundPlaneUpdate::kRejected);
    EXPECT_FALSE(tracker.initialized());
  }
}

TEST(GroundPlaneTrackerTest, InAirRestartRejectsPersistentFalsePlanes)
{
  GroundPlaneTracker tracker;

  // No independent landed/ground indication is available after an in-air
  // restart.  Both plausible-looking clearances must remain fail-closed.
  for (const double z : {1.50, 2.00}) {
    for (int i = 0; i < 12; ++i) {
      EXPECT_EQ(
        tracker.update(horizontalCandidate(z), vehicle(2.5, true)),
        GroundPlaneUpdate::kRejected);
      EXPECT_FALSE(tracker.initialized());
    }
  }
}

TEST(GroundPlaneTrackerTest, ArmedStateRejectsFreshInitialization)
{
  GroundPlaneTracker tracker;
  const auto candidate = horizontalCandidate(-0.67);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(tracker.update(candidate, vehicle(2.5, true)), GroundPlaneUpdate::kRejected);
  }
  EXPECT_FALSE(tracker.initialized());
}

TEST(GroundPlaneTrackerTest, DisarmedStateAllowsFreshInitializationWithoutGps)
{
  GroundPlaneTracker tracker;
  const auto candidate = horizontalCandidate(-0.67);
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kPending);
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kPending);
  EXPECT_EQ(tracker.update(candidate, vehicle()), GroundPlaneUpdate::kInitialized);
  EXPECT_TRUE(tracker.initialized());
}

}  // namespace
}  // namespace easyfly::ground
