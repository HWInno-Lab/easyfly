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

#include <easyfly_ellipse/detector.hpp>
#include <opencv2/imgproc.hpp>

namespace ellipse = easyfly::ellipse;

TEST(Ellipse, RotationsScaleAndSingleEdgeDeduplication)
{
  for (double angle : {0.0, 30.0, 89.0, 130.0}) {
    cv::Mat image(300, 400, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::ellipse(image, {210, 145}, {75, 35}, angle, 0, 360, {255, 255, 255}, -1);
    const auto before = image.clone();
    const auto observations = ellipse::detect(image);
    ASSERT_EQ(observations.size(), 1u) << angle;
    const auto & o = observations.front();
    EXPECT_NEAR(o.center.x, 210.0, 1.0);
    EXPECT_NEAR(o.center.y, 145.0, 1.0);
    EXPECT_NEAR(o.major_axis_px, 150.0, 3.0);
    EXPECT_NEAR(o.minor_axis_px, 70.0, 3.0);
    const double difference = std::remainder(o.angle_rad - angle * CV_PI / 180.0, CV_PI);
    EXPECT_NEAR(difference, 0.0, 0.03);
    EXPECT_GE(o.score, 0.5);
    EXPECT_LE(o.score, 1.0);
    EXPECT_EQ(cv::norm(image, before, cv::NORM_INF), 0);
  }
}

TEST(Ellipse, MultipleTargetsGrayNoncontiguousAndLimit)
{
  cv::Mat canvas(350, 500, CV_8UC1, cv::Scalar(0));
  cv::Mat image = canvas(cv::Rect(10, 10, 450, 300));
  cv::ellipse(image, {100, 110}, {50, 25}, 15, 0, 360, {255}, -1);
  cv::ellipse(image, {320, 170}, {60, 30}, 70, 0, 360, {255}, -1);
  ASSERT_FALSE(image.isContinuous());
  EXPECT_EQ(ellipse::detect(image).size(), 2u);
  ellipse::Parameters parameters;
  parameters.max_detections = 1;
  EXPECT_EQ(ellipse::detect(image, parameters).size(), 1u);
}

TEST(Ellipse, RejectsRectanglePartialArcSmallAndClippedTargets)
{
  cv::Mat image(300, 400, CV_8UC1, cv::Scalar(0));
  EXPECT_TRUE(ellipse::detect(image).empty());
  cv::rectangle(image, {80, 70}, {300, 220}, {255}, -1);
  EXPECT_TRUE(ellipse::detect(image).empty());
  image.setTo(0);
  cv::ellipse(image, {200, 150}, {70, 35}, 0, 0, 180, {255}, 2);
  EXPECT_TRUE(ellipse::detect(image).empty());
  image.setTo(0);
  cv::ellipse(image, {5, 150}, {70, 35}, 0, 0, 360, {255}, -1);
  EXPECT_TRUE(ellipse::detect(image).empty());
  image.setTo(0);
  cv::circle(image, {200, 150}, 3, {255}, -1);
  EXPECT_TRUE(ellipse::detect(image).empty());
}

TEST(Ellipse, RingCircleAndDeterministicNoise)
{
  cv::Mat image(300, 400, CV_8UC1, cv::Scalar(0));
  cv::circle(image, {200, 150}, 50, {255}, -1);
  const auto circle = ellipse::detect(image);
  ASSERT_EQ(circle.size(), 1u);
  EXPECT_NEAR(circle[0].major_axis_px, circle[0].minor_axis_px, 1.0);
  image.setTo(0);
  cv::ellipse(image, {200, 150}, {80, 40}, 25, 0, 360, {255}, 3);
  EXPECT_FALSE(ellipse::detect(image).empty());
  image.setTo(0);
  cv::RNG random(42);
  random.fill(image, cv::RNG::UNIFORM, 0, 30);
  EXPECT_TRUE(ellipse::detect(image).empty());
}

TEST(Ellipse, InvalidParametersTypesAndBounds)
{
  EXPECT_TRUE(ellipse::detect(cv::Mat()).empty());
  EXPECT_THROW(ellipse::detect(cv::Mat(20, 20, CV_32FC1)), std::invalid_argument);
  EXPECT_THROW(ellipse::detect(cv::Mat(1500, 1500, CV_8UC1)), std::invalid_argument);
  ellipse::Parameters parameters;
  parameters.max_radial_error = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ellipse::validate(parameters), std::invalid_argument);
  parameters = {};
  parameters.canny_high = parameters.canny_low;
  EXPECT_THROW(ellipse::validate(parameters), std::invalid_argument);
  parameters = {};
  parameters.max_detections = 0;
  EXPECT_THROW(ellipse::validate(parameters), std::invalid_argument);
}
