// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_pixel_to_3d/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#pragma once
#include <string>
#include <easyfly_msgs/msg/target3_d_array.hpp>
#include <std_msgs/msg/header.hpp>
namespace easyfly_pixel_to_3d
{
// Target3D values are expressed in target_frame after the optical-frame TF
// transform. Keep the source timestamp for temporal association, but publish
// the frame in which those values are actually expressed.
inline std_msgs::msg::Header targetFrameHeader(
  const std_msgs::msg::Header & source,
  const std::string & target_frame)
{
  auto output = source;
  output.frame_id = target_frame;
  return output;
}
// Apply the output-frame contract to both levels of Target3DArray. Keeping
// this in one helper prevents the array and per-target headers from drifting
// apart as the message construction evolves.
inline void applyTargetFrameContract(
  easyfly_msgs::msg::Target3DArray & output,
  const std_msgs::msg::Header & source,
  const std::string & target_frame)
{
  output.header = targetFrameHeader(source, target_frame);
  for (auto & target : output.targets) {
    target.header = targetFrameHeader(source, target_frame);
  }
}
}  // namespace easyfly_pixel_to_3d
