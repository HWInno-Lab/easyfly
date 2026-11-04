// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_target_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#include <gtest/gtest.h>
#include "easyfly_target_tracking/target_frame.hpp"
namespace easyfly_target_tracking
{
namespace
{
TEST(TargetFrameTest, RejectsEmptyStartupSnapshot)
{
  easyfly_msgs::msg::Target3DArray message;
  EXPECT_FALSE(targetArrayFrameValid(message));
}
TEST(TargetFrameTest, AcceptsOdomFrame)
{
  easyfly_msgs::msg::Target3DArray message;
  message.header.frame_id = "odom";
  EXPECT_TRUE(targetArrayFrameValid(message));
}
TEST(TargetFrameTest, RejectsOtherFrame)
{
  easyfly_msgs::msg::Target3DArray message;
  message.header.frame_id = "map";
  EXPECT_FALSE(targetArrayFrameValid(message));
}
}  // namespace
}  // namespace easyfly_target_tracking
