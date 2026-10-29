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
