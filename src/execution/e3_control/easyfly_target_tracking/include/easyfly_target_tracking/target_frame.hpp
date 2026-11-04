// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_target_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#pragma once
#include <string>
#include <easyfly_msgs/msg/target3_d_array.hpp>
namespace easyfly_target_tracking
{
inline constexpr char kTargetWorldFrame[] = "odom";
// The default-constructed message is not a received sample.  Callers must
// gate this check on their received/generation flag, then reject every
// received empty or non-odom frame before using target coordinates.
inline bool targetArrayFrameValid(const easyfly_msgs::msg::Target3DArray & message)
{
  return message.header.frame_id == kTargetWorldFrame;
}
}  // namespace easyfly_target_tracking
