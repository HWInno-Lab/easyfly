// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "easyfly_orbslam3_integration/pose_continuity.hpp"
using easyfly_orbslam3::PoseContinuity;
TEST(PoseContinuity, MeasuredF030JumpLatchesAndCannotResume)
{
  PoseContinuity gate;
  ASSERT_TRUE(gate.accept(94201000000, {.823277235, 8.122971535, -.450313926}, {0,0,0,1}));
  EXPECT_FALSE(gate.accept(94234000000, {4.365235329, 17.58599472, -.085003078}, {0,0,0,1}));
  EXPECT_FALSE(gate.accept(94267000000, {.83, 8.12, -.45}, {0,0,0,1}));
}
TEST(PoseContinuity, NormalMotionAndQuaternionSignRemainValid)
{
  PoseContinuity gate;
  for (int i=0; i<100; ++i)
    ASSERT_TRUE(gate.accept(1000000000LL+i*33000000LL, {.099*i,0,0}, {0,0,0,i%2? -1.:1.}));
}
TEST(PoseContinuity, RotationJumpAndMissingSourceIntervalReject)
{
  PoseContinuity turn, gap, duplicate;
  for (auto * gate : {&turn, &gap, &duplicate})
    ASSERT_TRUE(gate->accept(1000000000, {0,0,0}, {0,0,0,1}));
  EXPECT_FALSE(turn.accept(1033000000, {0,0,0}, {0,0,1,0}));
  EXPECT_FALSE(gap.accept(1600000000, {0,0,0}, {0,0,0,1}));
  EXPECT_FALSE(duplicate.accept(1000000000, {0,0,0}, {0,0,0,1}));
}
TEST(PoseContinuity, NonfiniteAndZeroQuaternionReject)
{
  PoseContinuity values, rotation;
  EXPECT_FALSE(values.accept(1000000000, {std::numeric_limits<double>::quiet_NaN(),0,0}, {0,0,0,1}));
  EXPECT_FALSE(rotation.accept(1000000000, {0,0,0}, {0,0,0,0}));
}
