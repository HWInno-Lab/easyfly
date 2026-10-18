// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <fast_lio/msg/pose6_d.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
int main()
{
  fast_lio::msg::Pose6D original;
  original.offset_time = .123;
  original.acc = {1., 2., 3.}; original.gyr = {.1, .2, .3};
  original.vel = {4., 5., 6.}; original.pos = {7., 8., 9.};
  original.rot = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
  rclcpp::Serialization<fast_lio::msg::Pose6D> codec;
  rclcpp::SerializedMessage bytes;
  codec.serialize_message(&original, &bytes);
  fast_lio::msg::Pose6D restored;
  codec.deserialize_message(&bytes, &restored);
  return restored == original ? 0 : 1;
}
