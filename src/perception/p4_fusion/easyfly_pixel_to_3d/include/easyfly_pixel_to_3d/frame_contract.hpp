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
