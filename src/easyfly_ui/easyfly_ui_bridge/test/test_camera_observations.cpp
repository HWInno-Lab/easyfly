// Copyright 2026 EasyFly Project
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
#include "easyfly_ui_bridge/camera_observations.hpp"
namespace b = easyfly_ui_bridge;
namespace
{
b::CameraPerceptionBinding binding()
{return {"/image", "/detections", "/tracks_raw", "/target_positions", "odom"};}
sensor_msgs::msg::Image::SharedPtr image(int sec = 10)
{
  auto message = std::make_shared<sensor_msgs::msg::Image>();
  message->header.stamp.sec = sec; message->header.frame_id = "camera";
  message->width = 640; message->height = 480; message->encoding = "rgb8";
  message->step = 1920; message->data.assign(921600, 1); return message;
}
easyfly_msgs::msg::DetectionInfoArray::SharedPtr detections(int sec = 10)
{
  auto value = std::make_shared<easyfly_msgs::msg::DetectionInfoArray>();
  value->header = image(sec)->header;
  easyfly_msgs::msg::DetectionInfo box; box.header = value->header;
  box.track_id = -1; box.class_name = "person"; box.confidence = .8;
  box.bbox_center_x = box.bbox_center_y = .5; box.bbox_width = box.bbox_height = .2;
  value->detections.push_back(box); return value;
}
b::ClockState observed_clock(int sec = 10)
{
  return {b::Quality::ValidAtObservation, sec *1000000000LL, 0};
}
}  // namespace
TEST(CameraObservations, ExactHeaderMatchAndNoCameraFrameGuess)
{
  b::CameraObservationBuffer buffer(binding()); buffer.image(image(), 1, 0);
  buffer.detections(detections(), 1.1, 0);
  auto result = buffer.snapshot(observed_clock(), 1.2);
  ASSERT_TRUE(result.detection_image.value);
  EXPECT_EQ(result.detection_image.value->header, result.detections.value->header);
  auto changed = detections(11); changed->header.frame_id = "different_camera";
  changed->detections[0].header = changed->header;
  buffer.image(image(11), 1.3, 0); buffer.detections(changed, 1.4, 0);
  EXPECT_FALSE(buffer.snapshot(observed_clock(11), 1.5).detection_image.value);
}
TEST(CameraObservations, CountBytesAndSteadyLifetimeBounded)
{
  b::CameraObservationBuffer buffer(binding());
  for (int i = 0; i < 30; ++i) {
    auto next = image(); next->header.stamp.nanosec = static_cast<unsigned>(i) * 1000000U;
    buffer.image(next, 1 + i * .001, 0);
  }
  auto result = buffer.snapshot(observed_clock(), 1.1);
  EXPECT_EQ(result.cached_images, 8U);
  EXPECT_LE(result.cached_bytes, b::CameraObservationBuffer::max_bytes);
  EXPECT_EQ(buffer.snapshot(observed_clock(), 2.1).cached_images, 0U);
}
TEST(CameraObservations, DuplicateConflictAndOlderInputsDoNotRenew)
{
  b::CameraObservationBuffer buffer(binding()); auto input = image();
  buffer.image(input, 1, 0); buffer.detections(detections(), 1, 0);
  buffer.image(image(), 1.8, 0); buffer.detections(detections(), 1.8, 0);
  EXPECT_FALSE(buffer.snapshot(observed_clock(), 2.01).detection_image.value);
  EXPECT_EQ(buffer.snapshot(observed_clock(), 2.01).detections.evidence.quality, b::Quality::Stale);
  buffer.clear(); buffer.image(image(), 3, 0); buffer.detections(detections(), 3, 0);
  auto conflict = image(); conflict->data[0] = 2; buffer.image(conflict, 3.1, 0);
  EXPECT_FALSE(buffer.snapshot(observed_clock(), 3.2).detection_image.value);
  buffer.detections(detections(9), 3.3, 0);
  EXPECT_EQ(
    buffer.snapshot(observed_clock(), 3.4).detections.evidence.quality,
    b::Quality::Invalid);
}
TEST(CameraObservations, ByteBudgetIsIndependentOfFrameCount)
{
  b::CameraObservationBuffer buffer(binding());
  for (int i = 0; i < 3; ++i) {
    auto next = image(10 + i); next->width = 1920; next->height = 1080; next->step = 5760;
    next->data.assign(6220800, 1); buffer.image(next, 1 + i * .1, 0);
  }
  const auto value = buffer.snapshot(observed_clock(12), 1.3);
  EXPECT_EQ(value.cached_images, 1U);
  EXPECT_EQ(value.cached_bytes, 6220800U);
}
TEST(CameraObservations, InvalidOversizeAndValidEmptyStayDistinct)
{
  b::CameraObservationBuffer buffer(binding()); buffer.image(image(), 1, 0);
  auto invalid = detections(); invalid->detections[0].bbox_width = -1;
  buffer.detections(invalid, 1, 0);
  EXPECT_EQ(
    buffer.snapshot(observed_clock(), 1.1).detections.evidence.quality,
    b::Quality::Invalid);
  auto large = detections(11); large->detections.resize(101);
  buffer.detections(large, 1.2, 0);
  EXPECT_FALSE(buffer.snapshot(observed_clock(11), 1.3).detections.value);
  auto empty = detections(12); empty->detections.clear(); buffer.image(image(12), 1.4, 0);
  buffer.detections(empty, 1.4, 0); auto result = buffer.snapshot(observed_clock(12), 1.5);
  ASSERT_TRUE(result.detection_image.value); ASSERT_TRUE(result.detections.value);
  EXPECT_TRUE(result.detections.value->detections.empty());
}
TEST(CameraObservations, PauseResetFutureAndClearFailClosed)
{
  b::CameraObservationBuffer buffer(binding()); buffer.image(image(), 1, 0);
  buffer.detections(detections(), 1, 0);
  auto paused = observed_clock(); paused.quality = b::Quality::ClockPaused;
  EXPECT_FALSE(buffer.snapshot(paused, 1.1).detection_image.value);
  auto reset = observed_clock(); reset.epoch = 1;
  EXPECT_FALSE(buffer.snapshot(reset, 1.1).detection_image.value);
  EXPECT_FALSE(buffer.snapshot(observed_clock(1), 1.1).detection_image.value);
  buffer.clear(); EXPECT_EQ(buffer.snapshot(observed_clock(), 1.1).cached_images, 0U);
  EXPECT_FALSE(buffer.snapshot(observed_clock(), 1.1).detections.value);
}

TEST(CameraObservations, ClassicalGeometryMatchesOnlyOriginalImageAndExpires)
{
  auto bnd = binding(); bnd.ellipse_topic = "/ellipse_observations";
  b::CameraObservationBuffer buffer(bnd);
  auto message = std::make_shared<easyfly_msgs::msg::EllipseObservationArray>();
  message->header = image()->header; message->image_width = 640; message->image_height = 480;
  message->valid = true; buffer.image(image(), 1, 0); buffer.ellipses(message, 1, 0);
  auto result = buffer.snapshot(observed_clock(), 1.1);
  ASSERT_TRUE(result.ellipse_image.value); ASSERT_TRUE(result.ellipses.value);
  EXPECT_TRUE(result.ellipses.value->observations.empty());  // valid no-target image
  buffer.ellipses(message, 1.4, 0);  // duplicate cannot refresh the receipt lifetime
  EXPECT_FALSE(buffer.snapshot(observed_clock(), 1.6).ellipse_image.value);
  buffer.clear(); buffer.image(image(), 2, 0); buffer.ellipses(message, 2, 0);
  auto invalidation = std::make_shared<easyfly_msgs::msg::EllipseObservationArray>(*message);
  invalidation->valid = false; buffer.ellipses(invalidation, 2.1, 0);
  EXPECT_FALSE(buffer.snapshot(observed_clock(), 2.2).ellipse_image.value);
  buffer.clear(); auto malformed = std::make_shared<easyfly_msgs::msg::ColorLineObservationArray>();
  malformed->header = image()->header; malformed->valid = true;
  malformed->image_width = 640; malformed->image_height = 480;
  malformed->observations.resize(33); buffer.color_lines(malformed, 3, 0);
  EXPECT_EQ(
    buffer.snapshot(observed_clock(), 3.1).color_lines.evidence.quality,
    b::Quality::Invalid);
}

namespace
{
template<typename Message, typename Receive, typename Select>
void verified_geometry_expiry(Receive receive, Select select)
{
  auto bnd = binding(); bnd.ellipse_topic = "/ellipse_observations";
  bnd.color_line_topic = "/color_line_observations";
  b::CameraObservationBuffer buffer(bnd);
  auto valid = std::make_shared<Message>(); valid->header = image()->header;
  valid->image_width = 640; valid->image_height = 480; valid->valid = true;
  (buffer.*receive)(valid, 1., 0);
  auto expired = std::make_shared<Message>(*valid); expired->valid = false;
  expired->reason = "expired_or_clock_invalid";
  (buffer.*receive)(expired, 1.6, 0);
  const auto result = buffer.snapshot(observed_clock(), 1.7).*select;
  ASSERT_TRUE(result.value); EXPECT_FALSE(result.value->valid);
  EXPECT_TRUE(result.value->observations.empty());
  EXPECT_EQ(result.evidence.quality, b::Quality::Invalid);
  EXPECT_EQ(result.evidence.reason, b::kValidatedGeometryExpiry);
  EXPECT_DOUBLE_EQ(result.evidence.first_source_steady, 1.);
  EXPECT_DOUBLE_EQ(result.evidence.received_steady, 1.);
  EXPECT_NEAR(result.evidence.receipt_age, .7, 1e-10);
  // Duplicate expiry remains terminal even if the clock/source looks fresh again.
  (buffer.*receive)(std::make_shared<Message>(*expired), 1.8, 0);
  const auto repeat = buffer.snapshot(observed_clock(), 1.9).*select;
  ASSERT_TRUE(repeat.value); EXPECT_EQ(repeat.evidence.quality, b::Quality::Invalid);
  EXPECT_EQ(repeat.evidence.reason, b::kValidatedGeometryExpiry);
  EXPECT_DOUBLE_EQ(repeat.evidence.first_source_steady, 1.);
  EXPECT_DOUBLE_EQ(repeat.evidence.received_steady, 1.);
  EXPECT_NEAR(repeat.evidence.receipt_age, .9, 1e-10);
  auto next_epoch = observed_clock(); ++next_epoch.epoch;
  const auto reset = buffer.snapshot(next_epoch, 1.9).*select;
  EXPECT_EQ(reset.evidence.quality, b::Quality::ClockReset);
  EXPECT_NE(reset.evidence.reason, b::kValidatedGeometryExpiry);
  // The immutable snapshot keeps its original invalid classification.
  EXPECT_EQ(result.evidence.reason, b::kValidatedGeometryExpiry);
  (buffer.*receive)(std::make_shared<Message>(*valid), 1.95, 0);
  EXPECT_FALSE((buffer.snapshot(observed_clock(), 1.95).*select).value);
  EXPECT_EQ(
    (buffer.snapshot(observed_clock(), 1.95).*select).evidence.quality,
    b::Quality::Invalid);
  auto newer = std::make_shared<Message>(*valid); newer->header.stamp.nanosec = 100000000;
  (buffer.*receive)(newer, 2., 0);
  EXPECT_EQ(
    (buffer.snapshot(observed_clock(), 2.).*select).evidence.quality,
    b::Quality::ValidAtObservation);
  buffer.clear(); EXPECT_FALSE((buffer.snapshot(observed_clock(), 2.).*select).value);
}
template<typename Message, typename Receive, typename Select>
void rejected_geometry_expiry(Receive receive, Select select)
{
  for (int scenario = 0; scenario < 12; ++scenario) {
    SCOPED_TRACE(scenario);
    auto bnd = binding(); bnd.ellipse_topic = "/ellipse_observations";
    bnd.color_line_topic = "/color_line_observations";
    b::CameraObservationBuffer buffer(bnd);
    auto valid = std::make_shared<Message>(); valid->header = image()->header;
    valid->image_width = 640; valid->image_height = 480; valid->valid = true;
    if (scenario != 0) {(buffer.*receive)(valid, 1., 0);}
    auto invalid = std::make_shared<Message>(*valid); invalid->valid = false;
    invalid->reason = "expired_or_clock_invalid";
    uint64_t epoch = 0;
    switch (scenario) {
      case 0: break;  // No prior valid same-frame observation.
      case 1: invalid->image_width = 641; break;
      case 2: invalid->header.frame_id = "other_camera"; break;
      case 3: epoch = 1; break;
      case 4: invalid->reason = "processing_expired"; break;
      case 5: invalid->reason = "unknown"; break;
      case 6: invalid->header.stamp.sec = 9; break;
      case 7: invalid->header.stamp.sec = 11; break;
      case 8: invalid->observations.resize(1); break;
      case 9: invalid->image_width = invalid->image_height = 0; break;
      case 10: invalid->header.stamp.sec = 0; break;
      case 11: {
          // A hard failure destroys the prior valid basis for a later claimed expiry.
          auto malformed = std::make_shared<Message>(*invalid);
          malformed->reason = "algorithm_error"; (buffer.*receive)(malformed, 1.1, 0);
          break;
        }
    }
    (buffer.*receive)(invalid, 1.6, epoch);
    auto clock = observed_clock(); clock.epoch = epoch;
    const auto result = buffer.snapshot(clock, 1.7).*select;
    EXPECT_FALSE(result.value); EXPECT_EQ(result.evidence.quality, b::Quality::Invalid);
    EXPECT_NE(result.evidence.reason, b::kValidatedGeometryExpiry);
    EXPECT_FALSE((buffer.snapshot(clock, 1.8).*select).value);
  }
}
}  // namespace
TEST(CameraObservations, VerifiedClassicalExpiryPreservesOriginalReceiptAndCannotRevive)
{
  verified_geometry_expiry<easyfly_msgs::msg::EllipseObservationArray>(
    &b::CameraObservationBuffer::ellipses, &b::CameraObservations::ellipses);
  verified_geometry_expiry<easyfly_msgs::msg::ColorLineObservationArray>(
    &b::CameraObservationBuffer::color_lines, &b::CameraObservations::color_lines);
}
TEST(CameraObservations, ExpiryClaimsRequirePreviousValidExactEnvelopeAndEpoch)
{
  rejected_geometry_expiry<easyfly_msgs::msg::EllipseObservationArray>(
    &b::CameraObservationBuffer::ellipses, &b::CameraObservations::ellipses);
  rejected_geometry_expiry<easyfly_msgs::msg::ColorLineObservationArray>(
    &b::CameraObservationBuffer::color_lines, &b::CameraObservations::color_lines);
}
