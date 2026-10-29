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
#include "easyfly_precision_landing/policy.hpp"

namespace pl = easyfly_precision_landing;

pl::Config config()
{
  pl::Config c;
  c.horizontal_speed = .3;
  c.descent_speed = .2;
  c.align_tolerance = .1;
  c.handoff_height = .3;
  c.descent_floor_height = .2;
  c.max_height = 2;
  c.max_radius = 2;
  c.plane_z = 0;
  return c;
}

TEST(LandingPolicy, MissingBoundariesCannotActivate)
{
  EXPECT_FALSE(pl::Config{}.valid());
  auto c = config();
  EXPECT_TRUE(c.valid());
  c.descent_speed = 2;
  EXPECT_FALSE(c.valid());
}

TEST(LandingPolicy, OnlyDistinctObservationsAuthorizeDescent)
{
  pl::Policy policy(config());
  policy.start({0, 0, 1}, 10);
  pl::Observation o{true, 1, 0, 0, 0, 0, 0, -1};
  for (int i = 0; i < 20; ++i) {
    const auto d = policy.step({0, 0, 1}, o, 10 + i * .01, .01);
    EXPECT_EQ("align", d.phase);
    EXPECT_DOUBLE_EQ(1, d.z);
  }
  for (int i = 0; i < 4; ++i) {
    ++o.stamp_ns;
    const auto d = policy.step({0, 0, 1}, o, 10.3 + i * .01, .01);
    if (i == 3) {EXPECT_EQ("descend", d.phase); EXPECT_LT(d.z, 1);}
  }
}

TEST(LandingPolicy, LossStopsDescentAndRecoveryMustRealign)
{
  pl::Policy policy(config());
  policy.start({0, 0, 1}, 10);
  pl::Observation o{true, 1, 0, 0, 0, 0, 0, 1};
  for (int i = 0; i < 5; ++i) {
    ++o.stamp_ns;
    policy.step({0, 0, 1}, o, 10 + i * .05, .05);
  }
  auto d = policy.step({0, 0, .9}, {}, 10.3, .05);
  EXPECT_EQ("lost_hold", d.phase);
  EXPECT_DOUBLE_EQ(.9, d.z);
  ++o.stamp_ns;
  d = policy.step({0, 0, .9}, o, 10.4, .05);
  EXPECT_EQ("reacquire_hold", d.phase);
  EXPECT_DOUBLE_EQ(.9, d.z);
  policy.step({0, 0, .9}, {}, 11, .05);
  d = policy.step({0, 0, .9}, {}, 13.01, .05);
  EXPECT_TRUE(d.failed);
  EXPECT_FALSE(d.handoff);
}

TEST(LandingPolicy, HandoffNeedsAlignmentAndMeasuredLowHeight)
{
  pl::Policy policy(config());
  policy.start({0, 0, .29}, 10);
  pl::Observation o{true, 1, .8, 0, 0, 0, 0, 1};
  for (int i = 0; i < 10; ++i) {
    ++o.stamp_ns;
    EXPECT_FALSE(policy.step({0, 0, .29}, o, 10 + i * .01, .01).handoff);
  }
  policy.start({0, 0, .29}, 11);
  o.x = 0;
  for (int i = 0; i < 5; ++i) {
    ++o.stamp_ns;
    const auto d = policy.step({0, 0, .29}, o, 11 + i * .01, .01);
    EXPECT_EQ(i == 4, d.handoff);
  }
}

TEST(LandingPolicy, StaticBoardCannotAccumulateSmallJumps)
{
  pl::Policy policy(config());
  policy.start({0, 0, 1}, 10);
  pl::Observation o{true, 1, 0, 0, 0, 0, 0, 1};
  policy.step({0, 0, 1}, o, 10, .05);
  o.x = .2;
  ++o.stamp_ns;
  EXPECT_TRUE(policy.step({0, 0, 1}, o, 10.05, .05).target_valid);
  o.x = .4;
  ++o.stamp_ns;
  EXPECT_FALSE(policy.step({0, 0, 1}, o, 10.1, .05).target_valid);
}

TEST(LandingPolicy, PlaneAndPositionBoundariesHoldWithoutBlindDescent)
{
  pl::Policy policy(config());
  policy.start({0, 0, 1}, 10);
  pl::Observation o{true, 1, 0, 0, .5, 0, 0, 1};
  EXPECT_FALSE(policy.step({0, 0, 1}, o, 10, .05).target_valid);
  o.z = 0;
  o.normal_z = .5;
  EXPECT_FALSE(policy.step({0, 0, 1}, o, 10.1, .05).target_valid);
  EXPECT_TRUE(policy.step({3, 0, 1}, {}, 10.2, .05).failed);
}

TEST(LandingPolicy, AlignmentDoesNotChaseVerticalMeasurementDrift)
{
  pl::Policy policy(config());
  policy.start({0, 0, 1}, 10);
  pl::Observation o{true, 1, .2, 0, 0, 0, 0, 1};
  for (int i = 0; i < 10; ++i) {
    ++o.stamp_ns;
    const auto d = policy.step({0, 0, 1 - i * .01}, o, 10 + i * .05, .05);
    EXPECT_EQ("align", d.phase);
    EXPECT_DOUBLE_EQ(1, d.z);
  }
  o.x = 0;
  for (int i = 0; i < 5; ++i) {
    ++o.stamp_ns;
    policy.step({0, 0, .9}, o, 11 + i * .05, .05);
  }
  o.x = .2;
  ++o.stamp_ns;
  EXPECT_DOUBLE_EQ(.88, policy.step({0, 0, .88}, o, 11.3, .05).z);
  ++o.stamp_ns;
  EXPECT_DOUBLE_EQ(.88, policy.step({0, 0, .87}, o, 11.35, .05).z);
}

TEST(LandingPolicy, ExplicitApproachBandRejectsMissingOrInvertedFloor)
{
  auto c = config();
  c.descent_floor_height = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(c.valid());
  c.descent_floor_height = .3;
  EXPECT_FALSE(c.valid());
  c.descent_floor_height = .4;
  EXPECT_FALSE(c.valid());
  c.descent_floor_height = .1;
  EXPECT_FALSE(c.valid());
  c.descent_floor_height = .2;
  EXPECT_TRUE(c.valid());
}

TEST(LandingPolicy, ApproachCommandsRespectFloorAndHandoffNeedsMeasuredBand)
{
  pl::Policy policy(config());
  policy.start({0, 0, .35}, 10);
  pl::Observation o{true, 1, 0, 0, 0, 0, 0, 1};
  pl::Decision d;
  for (int i = 0; i < 30; ++i) {
    ++o.stamp_ns;
    d = policy.step({0, 0, .35}, o, 10 + i * .05, .05);
    EXPECT_FALSE(d.handoff);
    EXPECT_GE(d.z, .2);
  }
  EXPECT_DOUBLE_EQ(.2, d.z);
  EXPECT_LT(d.z, .3);
  ++o.stamp_ns;
  d = policy.step({0, 0, .29}, o, 11.6, .05);
  EXPECT_TRUE(d.handoff);
  EXPECT_DOUBLE_EQ(.29, d.z);
  policy.start({0, 0, .35}, 12);
  for (int i = 0; i < 10; ++i) {
    ++o.stamp_ns;
    policy.step({0, 0, .35}, o, 12 + i * .05, .05);
  }
  d = policy.step({0, 0, .32}, {}, 12.6, .05);
  EXPECT_EQ("lost_hold", d.phase);
  EXPECT_DOUBLE_EQ(.32, d.z);
  EXPECT_FALSE(d.handoff);
  ++o.stamp_ns;
  d = policy.step({0, 0, .29}, o, 12.65, .05);
  EXPECT_FALSE(d.handoff);
  EXPECT_EQ("reacquire_hold", d.phase);
}

TEST(LandingPolicy, IntermittentRecoveryDoesNotMoveHoldPointOrRenewLossDeadline)
{
  pl::Policy policy(config());
  policy.start({0, 0, 1}, 10);
  pl::Observation o{true, 1, 0, 0, 0, 0, 0, 1};
  policy.step({0, 0, 1}, o, 10, .05);
  policy.step({.1, .05, .9}, {}, 10.1, .05);
  for (int i = 0; i < 8; ++i) {
    ++o.stamp_ns;
    auto d = policy.step({.2, .1, .8}, o, 10.2 + i * .2, .05);
    EXPECT_EQ("reacquire_hold", d.phase);
    EXPECT_DOUBLE_EQ(.1, d.x);
    EXPECT_DOUBLE_EQ(.05, d.y);
    EXPECT_DOUBLE_EQ(.9, d.z);
    d = policy.step({.3, .15, .7}, {}, 10.3 + i * .2, .05);
    EXPECT_DOUBLE_EQ(.1, d.x);
    EXPECT_DOUBLE_EQ(.05, d.y);
    EXPECT_DOUBLE_EQ(.9, d.z);
    EXPECT_FALSE(d.handoff);
  }
  ++o.stamp_ns;
  const auto d = policy.step({.3, .15, .7}, o, 12.11, .05);
  EXPECT_TRUE(d.failed);
  EXPECT_EQ("landing_target_reacquisition_timeout", d.reason);
}

TEST(LandingPolicy, DuplicateRecoveryFramesCannotConfirmReacquisition)
{
  pl::Policy policy(config());
  policy.start({0, 0, 1}, 10);
  pl::Observation o{true, 1, 0, 0, 0, 0, 0, 1};
  policy.step({0, 0, 1}, o, 10, .05);
  policy.step({0, 0, .9}, {}, 10.1, .05);
  ++o.stamp_ns;
  for (int i = 0; i < 20; ++i) {
    const auto d = policy.step({0, 0, .8}, o, 10.2 + i * .05, .05);
    EXPECT_EQ("reacquire_hold", d.phase);
    EXPECT_DOUBLE_EQ(.9, d.z);
    EXPECT_FALSE(d.handoff);
  }
}

TEST(LandingPolicy, StableRecoveryStillRequiresFreshAlignmentBeforeDescent)
{
  pl::Policy policy(config());
  policy.start({0, 0, 1}, 10);
  pl::Observation o{true, 1, 0, 0, 0, 0, 0, 1};
  policy.step({0, 0, 1}, o, 10, .05);
  policy.step({.3, 0, .9}, {}, 10.1, .05);
  for (int i = 0; i < 5; ++i) {
    ++o.stamp_ns;
    const auto d = policy.step({.3, 0, .9}, o, 10.2 + i * .05, .05);
    EXPECT_EQ(i < 4 ? "reacquire_hold" : "align", d.phase);
    EXPECT_DOUBLE_EQ(.9, d.z);
    if (i < 4) {EXPECT_DOUBLE_EQ(.3, d.x);}
  }
  for (int i = 0; i < 5; ++i) {
    ++o.stamp_ns;
    const auto d = policy.step({0, 0, .9}, o, 10.5 + i * .05, .05);
    EXPECT_EQ(i < 4 ? "align" : "descend", d.phase);
  }
}

TEST(LandingPolicy, MovingBoardUsesOriginalMotionWindowAndKeepsStaticDefault)
{
  auto c = config();
  c.moving_target = true;
  c.align_tolerance = .08;
  pl::Policy policy(c);
  policy.start({0, 0, 1}, 10);
  pl::Decision decision;
  for (int i = 0; i < 400; ++i) {
    const double t = i * .05;
    pl::Observation o{true, 1000000000LL + i * 50000000LL, .02 * t, 0, 0, 0, 0, 1};
    decision = policy.step({.02 * t, 0, 1}, o, 10 + t, .05);
    EXPECT_FALSE(decision.failed);
    if (t < .7) {EXPECT_EQ(decision.phase, "align");}
  }
  EXPECT_EQ(decision.phase, "descend");
  // Travel exceeded the static board's 0.25m anchored boundary.
  EXPECT_GT(decision.x, .25);
  auto unsafe = c;
  unsafe.moving_usable_radius = .3;
  EXPECT_FALSE(unsafe.valid());
}

TEST(LandingPolicy, FastOrNonlinearMovingBoardCannotAuthorizeLanding)
{
  for (const double speed : {.2, -.2}) {
    auto c = config(); c.moving_target = true; c.align_tolerance = .08;
    pl::Policy policy(c); policy.start({0, 0, .29}, 10);
    pl::Decision decision;
    for (int i = 0; i < 30; ++i) {
      const double t = i * .05;
      decision = policy.step(
        {speed * t, 0, .29},
        {true, 1000000000LL + i * 50000000LL, speed * t, 0, 0, 0, 0, 1}, 10 + t, .05);
      EXPECT_FALSE(decision.handoff);
    }
    EXPECT_TRUE(decision.failed);
    EXPECT_EQ(decision.reason, "moving_target_exceeds_declared_motion_profile");
  }
}
