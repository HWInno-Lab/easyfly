// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_yolo_onnx/letterbox.hpp"
#include "easyfly_yolo_onnx/nms.hpp"
#include "easyfly_yolo_onnx/backend_factory.hpp"
TEST(YoloPostprocess, LetterboxRoundTripAndPadding)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(1, 2, 3)), out;
  const auto info = easyfly::yolo_onnx::letterbox(image, out, 640);
  EXPECT_EQ(info.pad_x, 0); EXPECT_EQ(info.pad_y, 80); EXPECT_FLOAT_EQ(info.scale, 1);
  EXPECT_EQ(out.at<cv::Vec3b>(0, 0), cv::Vec3b(114, 114, 114));
  float x1 = 10, y1 = 100, x2 = 20, y2 = 120;
  easyfly::yolo_onnx::unletterbox(x1, y1, x2, y2, info);
  EXPECT_FLOAT_EQ(x1, 10); EXPECT_FLOAT_EQ(y1, 20); EXPECT_FLOAT_EQ(y2, 40);
}
TEST(YoloPostprocess, NmsIsClassAwareAndBackendsDoNotFallbackSilently)
{
  std::vector<easyfly::detection::Detection> input = {
    {0, 0, 10, 10, .9f, 0, "a"}, {1, 1, 11, 11, .8f, 0, "a"},
    {0, 0, 10, 10, .7f, 1, "b"}};
  const auto output = easyfly::yolo_onnx::nms_by_class(input, .45);
  ASSERT_EQ(output.size(), 2u); EXPECT_EQ(output[0].class_id, 0); EXPECT_EQ(output[1].class_id, 1);
  easyfly::detection::DetectorParams params; params.backend = "tensorrt";
  EXPECT_THROW(easyfly::yolo_onnx::createBackend(params), std::runtime_error);
}
