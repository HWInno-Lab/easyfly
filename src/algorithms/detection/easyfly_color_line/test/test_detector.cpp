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
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <easyfly_color_line/detector.hpp>
#include <opencv2/imgproc.hpp>

namespace line = easyfly::color_line;

TEST(ColorLine, CenterAngleWidthAndOffset)
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(image, {210, 20}, {230, 220}, {0, 0, 255}, -1);
  const auto before = image.clone();
  const auto result = line::detect(image);
  ASSERT_EQ(result.size(), 1u);
  const auto & o = result.front();
  EXPECT_NEAR(o.center.x, 220.0, 0.5);
  EXPECT_NEAR(o.center.y, 120.0, 0.5);
  EXPECT_NEAR(o.length_px, 200.0, 1.0);
  EXPECT_NEAR(o.width_px, 20.0, 1.0);
  EXPECT_NEAR(o.angle_rad, -CV_PI / 2.0, 0.01);
  EXPECT_NEAR(o.offset_x_normalized, (220.0 - 159.5) / 159.5, 0.001);
  EXPECT_NEAR(o.score, 1.0, 0.01);
  EXPECT_NEAR(cv::norm(o.end - o.start), o.length_px, 0.1);
  EXPECT_EQ(cv::norm(image, before, cv::NORM_INF), 0);
}

TEST(ColorLine, RotatedTargetsAndNoncontiguousInput)
{
  for (float angle : {-70.0f, -25.0f, 0.0f, 40.0f, 85.0f}) {
    cv::Mat canvas(300, 420, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat image = canvas(cv::Rect(10, 10, 400, 280));
    cv::Point2f vertices[4];
    cv::RotatedRect({200, 140}, {180, 16}, angle).points(vertices);
    std::vector<cv::Point> points;
    for (const auto & point : vertices) {
      points.emplace_back(cvRound(point.x), cvRound(point.y));
    }
    cv::fillConvexPoly(image, points, {0, 0, 255});
    const auto result = line::detect(image);
    ASSERT_EQ(result.size(), 1u) << angle;
    EXPECT_NEAR(std::remainder(result[0].angle_rad - angle * CV_PI / 180.0, CV_PI), 0, 0.03);
    EXPECT_NEAR(result[0].length_px, 180, 3);
  }
}

TEST(ColorLine, WrappedRedAndConfigurableGreen)
{
  for (int hue : {0, 5, 175, 179}) {
    cv::Mat hsv(100, 400, CV_8UC3, cv::Scalar(hue, 255, 255)), image;
    cv::cvtColor(hsv, image, cv::COLOR_HSV2BGR);
    EXPECT_EQ(line::detect(image).size(), 1u) << hue;
  }
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(image, {140, 10}, {160, 230}, {0, 255, 0}, -1);
  EXPECT_TRUE(line::detect(image).empty());
  line::Parameters parameters;
  parameters.hue_min = 50;
  parameters.hue_max = 70;
  EXPECT_EQ(line::detect(image, parameters).size(), 1u);
}

TEST(ColorLine, RejectsBlobsWrongColorNoiseAndHonorsOutputCap)
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  EXPECT_TRUE(line::detect(image).empty());
  cv::circle(image, {100, 100}, 30, {0, 0, 255}, -1);
  cv::rectangle(image, {230, 20}, {250, 220}, {255, 0, 0}, -1);
  EXPECT_TRUE(line::detect(image).empty());
  image.setTo(cv::Scalar(0, 0, 0));
  for (int x : {40, 140, 240}) {
    cv::rectangle(image, {x, 20}, {x + 10, 220}, {0, 0, 255}, -1);
  }
  image.at<cv::Vec3b>(5, 5) = {0, 0, 255};
  EXPECT_EQ(line::detect(image).size(), 3u);
  line::Parameters parameters;
  parameters.max_detections = 2;
  EXPECT_EQ(line::detect(image, parameters).size(), 2u);
}

TEST(ColorLine, InvalidParametersTypesAndBounds)
{
  EXPECT_TRUE(line::detect(cv::Mat()).empty());
  EXPECT_THROW(line::detect(cv::Mat(20, 20, CV_8UC1)), std::invalid_argument);
  EXPECT_THROW(line::detect(cv::Mat(1500, 1500, CV_8UC3)), std::invalid_argument);
  line::Parameters parameters;
  parameters.hue_max = 180;
  EXPECT_THROW(line::validate(parameters), std::invalid_argument);
  parameters = {};
  parameters.morphology_kernel = 2;
  EXPECT_THROW(line::validate(parameters), std::invalid_argument);
  parameters = {};
  parameters.min_area_px = std::numeric_limits<double>::infinity();
  EXPECT_THROW(line::validate(parameters), std::invalid_argument);
}
