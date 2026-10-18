// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <class_loader/class_loader.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <algorithm>
#include <iostream>
#include <string>

template<typename Message>
bool roundtrip(const Message & original)
{
  rclcpp::Serialization<Message> codec;
  rclcpp::SerializedMessage bytes;
  codec.serialize_message(&original, &bytes);
  Message restored;
  codec.deserialize_message(&bytes, &restored);
  return restored == original;
}

int main(int argc, char ** argv)
{
  if (argc != 3) {return 1;}
  livox_ros_driver2::msg::CustomMsg points;
  points.header.frame_id = "livox_fixture";
  points.timebase = 123456; points.point_num = 1; points.lidar_id = 2;
  points.points.resize(1);
  points.points[0].x = 1.5f; points.points[0].y = -.5f; points.points[0].z = 4;
  points.points[0].reflectivity = 42;
  realsense2_camera_msgs::msg::RGBD rgbd;
  rgbd.header.frame_id = "camera_fixture";
  rgbd.depth.width = rgbd.depth.height = 2;
  rgbd.depth.encoding = "16UC1"; rgbd.depth.step = 4;
  rgbd.depth.data = {1, 0, 2, 0, 3, 0, 4, 0};
  if (!roundtrip(points) || !roundtrip(rgbd)) {return 2;}
  const char * expected[] = {"livox_ros::DriverNode", "realsense2_camera::RealSenseNodeFactory"};
  for (int i = 0; i < 2; ++i) {
    class_loader::ClassLoader loader(argv[i+1]);
    const auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
    if (std::none_of(classes.begin(), classes.end(), [&](const std::string & name) {
        return name.find(expected[i]) != std::string::npos;})) {return 3;}
    std::cout << "component " << expected[i] << " loaded from " << argv[i+1] << '\n';
    // Registration/loading only: do not instantiate NodeFactory or any device node.
  }
  std::cout << "message roundtrips: Livox CustomMsg, RealSense RGBD\n";
}
