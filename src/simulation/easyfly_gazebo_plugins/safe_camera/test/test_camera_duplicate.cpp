// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0.
// You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
// Distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <sdf/sdf.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "easyfly_gazebo_plugins/camera_measurement_stamps.hpp"
#include "easyfly_gazebo_plugins/safe_camera.hpp"

namespace
{
class ExposedCamera : public gazebo_plugins::EasyFlyGazeboRosCamera
{
public:
  using gazebo_plugins::EasyFlyGazeboRosCamera::Load;
  using gazebo_plugins::EasyFlyGazeboRosCamera::GetRosNode;
};

TEST(CameraDuplicate, KeepsFrameGroupsIndependentAndPreservesClockRollback)
{
  gazebo_plugins::detail::CameraMeasurementStamps stamps;
  std::vector<int> info_stamps, image_stamps;
  auto publish = [&](int camera, int second) {
      const gazebo::common::Time stamp(second, 0);
      if (stamps.duplicate(camera, stamp)) {return false;}
      info_stamps.push_back(second); image_stamps.push_back(second);
      stamps.published(camera, stamp); return true;
    };
  EXPECT_TRUE(publish(0, 0));  // An initial zero stamp retains upstream behavior.
  EXPECT_FALSE(publish(0, 0));
  EXPECT_TRUE(publish(0, 10));
  EXPECT_FALSE(publish(0, 10));  // A new render payload is not a new measurement.
  EXPECT_TRUE(publish(0, 11));
  EXPECT_TRUE(publish(0, 2));  // Do not hide a real simulation-time rollback.
  EXPECT_FALSE(publish(0, 2));
  EXPECT_TRUE(publish(0, 3));
  EXPECT_TRUE(publish(1, 10));  // Stereo eyes may share an acquisition timestamp.
  EXPECT_FALSE(publish(1, 10));
  EXPECT_TRUE(publish(0, 10));
  EXPECT_EQ(info_stamps, image_stamps);
  EXPECT_EQ(image_stamps, (std::vector<int>{0, 10, 11, 2, 3, 10, 10}));
}

TEST(CameraDuplicate, SerializesDuplicatePublicationGroupsUnderImageMutex)
{
  gazebo_plugins::detail::CameraMeasurementStamps stamps;
  std::mutex image_mutex;
  unsigned info_count = 0, image_count = 0;
  std::vector<std::thread> callbacks;
  for (int i = 0; i < 12; ++i) {
    callbacks.emplace_back(
      [&] {
        std::lock_guard<std::mutex> lock(image_mutex);
        const gazebo::common::Time stamp(10, 123);
        if (stamps.duplicate(0, stamp)) {return;}
        ++info_count; ++image_count; stamps.published(0, stamp);
      });
  }
  for (auto & callback : callbacks) {
    callback.join();
  }
  EXPECT_EQ(info_count, 1u);
  EXPECT_EQ(image_count, 1u);
}

TEST(CameraDuplicate, RejectsNullRosNodeBeforeAccessingSensorOrQos)
{
  rclcpp::init(0, nullptr);
  auto sdf = std::make_shared<sdf::SDF>();
  sdf::init(sdf);
  ASSERT_TRUE(
    sdf::readString(
      "<sdf version='1.6'><model name='fixture'><plugin name='camera_guard' "
      "filename='unused.so'><ros><namespace>/duplicate_test</namespace></ros>"
      "</plugin></model></sdf>", sdf));
  auto plugin = sdf->Root()->GetElement("model")->GetElement("plugin");
  auto original = gazebo_ros::Node::Get(plugin);
  ASSERT_NE(original, nullptr);
  {
    ExposedCamera camera;
    // An invalid sensor must never be inspected after duplicate Node::Get fails.
    camera.Load(nullptr, plugin);
    EXPECT_EQ(camera.GetRosNode(), nullptr);
    EXPECT_EQ(original->get_fully_qualified_name(), std::string("/duplicate_test/camera_guard"));
  }
  original.reset();
  // Registry removal on teardown must allow a later independent owner.
  auto replacement = gazebo_ros::Node::Get(plugin);
  EXPECT_NE(replacement, nullptr);
  replacement.reset();
  rclcpp::shutdown();
}
}  // namespace
