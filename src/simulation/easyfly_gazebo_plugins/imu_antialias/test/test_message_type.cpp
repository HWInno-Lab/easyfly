// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <memory>
#include <gazebo/msgs/msgs.hh>
#include "easyfly_imu/message_type.hpp"
TEST(ImuMessageType, GeneratedClassIdentitySurvivesCloneAndParsing)
{
  auto descriptor = gazebo::msgs::Vector3d::descriptor();
  auto prototype = easyfly_imu::NativePrototype(descriptor);
  ASSERT_NE(prototype,nullptr);
  std::unique_ptr<google::protobuf::Message> clone(prototype->New());
  ASSERT_NE(dynamic_cast<gazebo::msgs::Vector3d *>(clone.get()),nullptr);
  gazebo::msgs::Vector3d source;
  source.set_x(1.);source.set_y(2.);source.set_z(3.);
  ASSERT_TRUE(clone->ParseFromString(source.SerializeAsString()));
  EXPECT_EQ(dynamic_cast<gazebo::msgs::Vector3d *>(clone.get())->z(),3.);
  EXPECT_EQ(easyfly_imu::NativePrototype(nullptr),nullptr);
}
