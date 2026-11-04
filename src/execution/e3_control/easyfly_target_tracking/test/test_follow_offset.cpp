// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_target_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#include <gtest/gtest.h>
#include <cmath>
#include "easyfly_target_tracking/follow_offset.hpp"

namespace easyfly_target_tracking
{
TEST(FollowOffset, LateralMotionExceedsUnchangedOneMetreReissueThreshold)
{
  // A target can move four metres without the old current-bearing waypoint
  // ever moving one metre: the complete V09 flight exposed this dead zone.
  const auto offset = followOffset(8, 0, 0, 0, 0, 4);
  const double initial_x = 8 + offset[0];
  for (double y : {-2.0, 2.0}) {
    const double length = std::hypot(8 - initial_x, y);
    const double old_x = 8 - (8 - initial_x) * 4 / length;
    const double old_y = y - y * 4 / length;
    EXPECT_LT(std::hypot(old_x - initial_x, old_y), 1.0);
    EXPECT_GT(std::hypot(8 + offset[0] - initial_x, y + offset[1]), 1.0);
    EXPECT_DOUBLE_EQ(std::hypot(offset[0], offset[1]), 4.0);
  }
}

TEST(FollowOffset, PreservesInitialApproachAndArbitraryOdomBearing)
{
  const auto offset = followOffset(7, 9, 1, 1, 0, 4);
  EXPECT_DOUBLE_EQ(offset[0], -2.4);
  EXPECT_DOUBLE_EQ(offset[1], -3.2);
  EXPECT_DOUBLE_EQ(std::hypot(offset[0], offset[1]), 4.0);
  // Moving the target translates the waypoint by exactly the same amount.
  EXPECT_DOUBLE_EQ((9 + offset[0]) - (7 + offset[0]), 2.0);
}

TEST(FollowOffset, CoincidentTargetUsesCurrentHeadingWithoutDivisionByZero)
{
  const auto offset = followOffset(3, 2, 3, 2, std::acos(-1.0) / 2, 4);
  EXPECT_NEAR(offset[0], 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(offset[1], -4.0);
}
}  // namespace easyfly_target_tracking
