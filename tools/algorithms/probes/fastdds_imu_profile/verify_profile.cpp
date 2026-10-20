// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Actual native writer policy inspection; no sensor messages or flight commands.
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>

#include <fastdds/dds/publisher/DataWriter.hpp>
#include <rcl/publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw_fastrtps_shared_cpp/custom_publisher_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fastdds_imu_policy_probe");
  int result = 0;
  auto verify = [&result](const auto & pub, int64_t expected, int depth) {
      const auto name = pub->get_topic_name();
      auto rmw = rcl_publisher_get_rmw_handle(pub->get_publisher_handle().get());
      if (!rmw || std::strcmp(rmw->implementation_identifier, "rmw_fastrtps_cpp") != 0) {
        std::cerr << "This native diagnostic requires rmw_fastrtps_cpp" << std::endl;
        result = 2;
        return;
      }
      auto info = static_cast<CustomPublisherInfo *>(rmw->data);
      const auto qos = info->data_writer_->get_qos();
      const auto heartbeat = qos.reliable_writer_qos().times.heartbeatPeriod.to_ns();
      std::cout << name << " heartbeat_ns=" << heartbeat << " depth=" << qos.history().depth <<
        std::endl;
      if (heartbeat != expected || qos.history().depth != depth) {
        result = 1;
      }
    };
  verify(
    node->create_publisher<sensor_msgs::msg::Imu>(
      "/livox/imu", rclcpp::QoS(1000).reliable()), 50000000LL, 1000);
  verify(
    node->create_publisher<sensor_msgs::msg::Imu>(
      "/unrelated/imu", rclcpp::QoS(1000).reliable()), 3000000000LL, 1000);
  if (node->declare_parameter<bool>("extended_inputs", false)) {
    verify(
      node->create_publisher<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", rclcpp::QoS(32).reliable()), 50000000LL, 32);
    verify(
      node->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", rclcpp::QoS(32).reliable()), 50000000LL, 32);
  }
  node.reset();
  rclcpp::shutdown();
  return result;
}
