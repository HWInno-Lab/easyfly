// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Contract fixture only; synthetic pixels are not a localization accuracy test.
#include <gtest/gtest.h>
#include "LIVMapper.h"

class ImageOwnership : public ::testing::TestWithParam<std::string>
{
public:
  static void SetUpTestSuite() {int argc = 0; rclcpp::init(argc, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_P(ImageOwnership, QueuedPixelsSurviveMessageMutationAndRelease)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("extrin_calib.extrinsic_T", std::vector<double>{0., 0., 0.}),
    rclcpp::Parameter("extrin_calib.extrinsic_R", std::vector<double>{1.,0.,0.,0.,1.,0.,0.,0.,1.}),
    rclcpp::Parameter("extrin_calib.Pcl", std::vector<double>{0., 0., 0.}),
    rclcpp::Parameter("extrin_calib.Rcl", std::vector<double>{1.,0.,0.,0.,1.,0.,0.,0.,1.})});
  auto node = std::make_shared<rclcpp::Node>("image_ownership_contract", options);
  LIVMapper mapper(node, "unused");
  auto message = std::make_shared<sensor_msgs::msg::Image>();
  message->width = 640; message->height = 480; message->step = 1920;
  message->encoding = GetParam(); message->data.resize(480 * 1920);
  message->data[0] = GetParam() == "bgr8" ? 1 : 3;
  message->data[1] = 2;
  message->data[2] = GetParam() == "bgr8" ? 3 : 1;
  auto image = mapper.getImageFromMsg(message);
  ASSERT_EQ(image.rows, 480); ASSERT_EQ(image.cols, 640);
  ASSERT_NE(image.data, message->data.data());
  std::fill(message->data.begin(), message->data.end(), 77);
  message.reset();
  const auto pixel = image.at<cv::Vec3b>(0, 0);
  EXPECT_EQ(pixel[0], 1); EXPECT_EQ(pixel[1], 2); EXPECT_EQ(pixel[2], 3);
}

INSTANTIATE_TEST_SUITE_P(RealSupportedEncodings, ImageOwnership,
  ::testing::Values(std::string("bgr8"), std::string("rgb8")));

TEST(AtlasOwnership, SurvivesNextImuPassAndCannotRepublishWrongEpoch)
{
  AtlasScanSnapshot snapshot;
  PointCloudXYZI original;
  original.resize(101);
  for (size_t i = 0; i < original.size(); ++i) {
    original[i].x = float(i) * .01f;
    original[i].y = -.7f; original[i].z = 1.2f;
    original[i].intensity = float(i);
  }
  snapshot.remember(original, 12.345);
  original.clear();  // The actual next UndistortPcl(VIO) does this.
  auto saved = snapshot.take(12.345);
  ASSERT_EQ(saved.size(), 101U);
  EXPECT_FLOAT_EQ(saved[100].x, 1.f);
  EXPECT_FLOAT_EQ(saved[100].intensity, 100.f);
  EXPECT_TRUE(snapshot.take(12.345).empty());
  snapshot.remember(saved, 13.);
  EXPECT_TRUE(snapshot.take(13.1).empty());
  EXPECT_TRUE(snapshot.take(13.).empty());
  snapshot.remember(saved, 14.);
  snapshot.remember(original, 14.1);  // A newer empty LIO scan invalidates the old one.
  EXPECT_TRUE(snapshot.take(14.).empty());
  EXPECT_TRUE(snapshot.take(14.1).empty());
}
