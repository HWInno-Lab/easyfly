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
#include "easyfly_tag_pose/tag_core.hpp"
namespace ef_tag = easyfly::tag_pose;
TEST(TagCore, KnownTiltPoseAndUnitQuaternion) {
  cv::Mat K = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);

  std::vector<cv::Point3f> object{{-.1, .1, 0}, {.1, .1, 0}, {.1, -.1, 0}, {-.1, -.1, 0}};

  std::vector<cv::Point2f> corners;

  cv::projectPoints(object, cv::Vec3d(2.7, .2, .1), cv::Vec3d(.1, .05, 1), K, cv::Mat(), corners);

  auto pose = ef_tag::estimate(corners, K, cv::Mat(), .2);

  ASSERT_TRUE(pose.valid) << pose.reason;
  EXPECT_NEAR(pose.t[2], 1, 1e-4);
  EXPECT_NEAR(
    cv::norm(
      pose.q), 1, 1e-9);

  EXPECT_FALSE(ef_tag::estimate(corners, K, cv::Mat(), 0).valid);

  corners[0] = corners[1];
  corners[2] = corners[1];

  EXPECT_FALSE(ef_tag::estimate(corners, K, cv::Mat(), .2).valid);
}
TEST(TagCore, ActualDictionaryDetector) {
  cv::Mat marker;

  cv::aruco::drawMarker(
    cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_APRILTAG_36h11), 7, 160, marker);

  cv::Mat image(240, 240, CV_8U, cv::Scalar(255));
  marker.copyTo(image(cv::Rect(40, 40, 160, 160)));

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;

  cv::aruco::detectMarkers(
    image, cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_APRILTAG_36h11), corners, ids, ef_tag::detector_parameters());

  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], 7);
}
TEST(TagCore, WarpedMarkerActualDetectorAndPose) {
  cv::Mat marker;

  cv::aruco::drawMarker(
    cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_APRILTAG_36h11), 7, 160, marker);

  cv::Mat K = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);

  std::vector<cv::Point3f> object{{-.1, .1, 0}, {.1, .1, 0}, {.1, -.1, 0}, {-.1, -.1, 0}};

  std::vector<cv::Point2f> corners;

  cv::projectPoints(object, cv::Vec3d(2.7, .2, .1), cv::Vec3d(0, 0, 1), K, cv::Mat(), corners);

  std::vector<cv::Point2f> source{{0, 0}, {159, 0}, {159, 159}, {0, 159}};
  cv::Mat image;

  cv::warpPerspective(
    marker, image, cv::getPerspectiveTransform(source, corners), cv::Size(
      640,
      480), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(
      255));

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> detected;

  cv::aruco::detectMarkers(
    image, cv::aruco::getPredefinedDictionary(
      cv::aruco::DICT_APRILTAG_36h11), detected, ids, ef_tag::detector_parameters());

  ASSERT_EQ(ids.size(), 1u);
  auto pose = ef_tag::estimate(detected[0], K, cv::Mat(), .2);

  ASSERT_TRUE(pose.valid) << pose.reason;
  EXPECT_NEAR(pose.t[2], 1, .06);
}

TEST(TagCore, SubpixelRefinementOfKnownTiltCorners) {
  cv::Mat marker;
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  cv::aruco::drawMarker(dictionary, 7, 160, marker);
  cv::Mat K = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
  std::vector<cv::Point3f> object{{-.1, .1, 0}, {.1, .1, 0}, {.1, -.1, 0}, {-.1, -.1, 0}};
  std::vector<cv::Point2f> truth;
  cv::projectPoints(
    object, cv::Vec3d(3.0, .03, .01), cv::Vec3d(.12, .06, 1), K,
    cv::Mat(), truth);
  // Geometry describes pixel boundaries, not centers of the boundary pixels.
  std::vector<cv::Point2f> source{{-.5, -.5}, {159.5, -.5}, {159.5, 159.5}, {-.5, 159.5}};
  cv::Mat image;
  cv::warpPerspective(
    marker, image, cv::getPerspectiveTransform(source, truth),
    cv::Size(640, 480), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
  std::vector<int> ids, refined_ids;
  std::vector<std::vector<cv::Point2f>> raw, refined;
  cv::aruco::detectMarkers(image, dictionary, raw, ids);
  cv::aruco::detectMarkers(image, dictionary, refined, refined_ids, ef_tag::detector_parameters());
  ASSERT_EQ(ids, refined_ids);
  ASSERT_EQ(ids.size(), 1u);
  double raw_error = 0, refined_error = 0;
  for (size_t i = 0; i < truth.size(); ++i) {
    raw_error += cv::norm(raw[0][i] - truth[i]);
    refined_error += cv::norm(refined[0][i] - truth[i]);
  }
  EXPECT_LT(refined_error, raw_error);
  auto pose = ef_tag::estimate(refined[0], K, cv::Mat(), .2);
  ASSERT_TRUE(pose.valid) << pose.reason;
  EXPECT_NEAR(pose.t[2], 1, .02);
  auto parameters = ef_tag::detector_parameters();
  EXPECT_EQ(parameters->cornerRefinementMethod, cv::aruco::CORNER_REFINE_SUBPIX);
  EXPECT_EQ(parameters->cornerRefinementWinSize, 3);
}

TEST(TagCore, ActualWrapperRejectsAmbiguityAndReprojectionError)
{
  cv::Mat K = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
  std::vector<cv::Point3f> object{{-.1, .1, 0}, {.1, .1, 0}, {.1, -.1, 0}, {-.1, -.1, 0}};
  std::vector<cv::Point2f> corners;
  cv::projectPoints(object, cv::Vec3d(3.0, .03, .01), cv::Vec3d(.12, .06, 1), K, cv::Mat(), corners);
  const auto ambiguous = ef_tag::estimate(corners, K, cv::Mat(), .2, 20, 100);
  EXPECT_FALSE(ambiguous.valid); EXPECT_EQ(ambiguous.reason, "pose_ambiguous");
  corners[0].x += 5;
  const auto noisy = ef_tag::estimate(corners, K, cv::Mat(), .2, .001, .000001);
  EXPECT_FALSE(noisy.valid); EXPECT_EQ(noisy.reason, "reprojection_error");
}
