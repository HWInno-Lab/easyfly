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
