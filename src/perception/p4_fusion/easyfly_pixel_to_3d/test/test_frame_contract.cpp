// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_pixel_to_3d/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#include <gtest/gtest.h>
#include "easyfly_pixel_to_3d/frame_contract.hpp"
namespace
{
std_msgs::msg::Header sourceHeader()
{
  std_msgs::msg::Header source;
  source.stamp.sec = 123;
  source.stamp.nanosec = 456789012;
  source.frame_id = "camera_color_optical_frame";
  return source;
}
}  // namespace
TEST(FrameContract, ChangesFrameWithoutChangingTimestamp)
{
  const auto source = sourceHeader();
  const auto output = easyfly_pixel_to_3d::targetFrameHeader(source, "odom");
  EXPECT_EQ(output.frame_id, "odom");
  EXPECT_EQ(output.stamp.sec, source.stamp.sec);
  EXPECT_EQ(output.stamp.nanosec, source.stamp.nanosec);
  EXPECT_EQ(source.frame_id, "camera_color_optical_frame");
}
TEST(FrameContract, AppliesSameFrameAndTimestampToArrayAndTargets)
{
  const auto source = sourceHeader();
  easyfly_msgs::msg::Target3DArray output;
  output.header = source;
  output.targets.resize(2);
  output.targets[0].header = source;
  output.targets[1].header = source;
  easyfly_pixel_to_3d::applyTargetFrameContract(output, source, "odom");
  EXPECT_EQ(output.header.frame_id, "odom");
  EXPECT_EQ(output.header.stamp.sec, source.stamp.sec);
  EXPECT_EQ(output.header.stamp.nanosec, source.stamp.nanosec);
  ASSERT_EQ(output.targets.size(), 2u);
  for (const auto & target : output.targets) {
    EXPECT_EQ(target.header.frame_id, "odom");
    EXPECT_EQ(target.header.stamp.sec, source.stamp.sec);
    EXPECT_EQ(target.header.stamp.nanosec, source.stamp.nanosec);
  }
}
