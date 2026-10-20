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
#include <fstream>
#include <sstream>
#include <limits>
#include "easyfly_camera_calib/camera_contract.hpp"
using easyfly_camera_calib::cameraMismatch;
using easyfly_camera_calib::loadCalibration;
using easyfly_camera_calib::parseCalibration;
static std::string config()
{
  std::ifstream input(CALIBRATION_FILE); std::ostringstream text; text << input.rdbuf();
  return text.str();
}
static std::string replace(std::string text, const std::string & from, const std::string & to)
{
  const auto at = text.find(from); EXPECT_NE(at, std::string::npos);
  if (at != std::string::npos) {text.replace(at, from.size(), to);} return text;
}
TEST(Calibration, RealConfiguredTreeIsValidated) {
  const auto c = loadCalibration(CALIBRATION_FILE); ASSERT_EQ(c.mounts.size(), 5u);
  EXPECT_TRUE(c.has_camera); EXPECT_EQ(c.camera_frame, "camera_color_optical_frame");
  EXPECT_DOUBLE_EQ(c.camera.fx, 337.36);
  const auto & imu = c.mounts.back();
  EXPECT_EQ(imu.parent, "livox_frame"); EXPECT_EQ(imu.child, "livox_imu_frame");
  EXPECT_DOUBLE_EQ(imu.translation[2], -0.00588);
}
TEST(Calibration, LegacyExtrinsicWithoutChildRemainsValidationOnly) {
  const auto c = parseCalibration(replace(config(), "      child: livox_imu_frame\n", ""));
  EXPECT_EQ(c.mounts.size(), 4u);
}
TEST(Calibration, RejectMalformedIntrinsicExtrinsicAndGraphBeforePublication) {
  const auto text = config();
  for (const auto & bad : std::vector<std::pair<std::string, std::string>>{
    {"fx: 337.36", "fx: .nan"}, {"fy: 337.36", "fy: 0"}, {"width: 640", "width: 0"},
    {"parent: base_link", "parent: camera_link"}, {"child: camera_link", "child: livox_frame"},
    {"[0.25, 0.0, 0.22]", "[.inf, 0, 0]"}, {"[0.25, 0.0, 0.22]", "[0, 0]"},
    {"distortion_model: plumb_bob", "distortion_model: unsupported"},
    {"fx: 337.36", "fx: 337.36\n      fx: 300"}, {"child: livox_frame", "child: invalid frame"},
    {"parent: base_link", "parent: camera_color_optical_frame"},
    {"fx: 337.36", "unknown_field: 42\n      fx: 337.36"},
    {"child: livox_imu_frame", "child: livox_frame"},
    {"child: livox_imu_frame", "child: camera_link"},
    {"child: livox_imu_frame", "child: bad imu"}})
  {
    EXPECT_THROW(parseCalibration(replace(text, bad.first, bad.second)), std::exception);
  }
  EXPECT_THROW(parseCalibration(text + "\n---\nsensors: {}"), std::exception);
  EXPECT_THROW(parseCalibration(std::string(65537, ' ')), std::exception);
}
TEST(Calibration, EveryCameraInfoCheckedNotJustFirstValid) {
  const auto c = loadCalibration(CALIBRATION_FILE); sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = c.camera_frame; info.width = c.camera.width; info.height = c.camera.height;
  info.k = {c.camera.fx, 0, c.camera.cx, 0, c.camera.fy, c.camera.cy, 0, 0, 1};
  info.distortion_model = c.camera.distortion_model; info.d = c.camera.d;
  EXPECT_EQ(cameraMismatch(info, c, 1), "valid");
  info.k[0] += 10; EXPECT_EQ(cameraMismatch(info, c, 1), "intrinsics_mismatch");
  info.k[0] = std::numeric_limits<double>::quiet_NaN(); EXPECT_EQ(
    cameraMismatch(
      info, c,
      1), "invalid_K");
  info.k[0] = c.camera.fx; info.d[0] = .1; EXPECT_EQ(
    cameraMismatch(
      info, c,
      1), "distortion_mismatch");
  info.d = c.camera.d; info.header.frame_id = "wrong"; EXPECT_EQ(
    cameraMismatch(
      info, c,
      1),
    "camera_frame_mismatch");
}
