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
#include "../src/camera_stream.hpp"
namespace b = easyfly_ui_bridge;
namespace
{
b::ClockState clock_state(uint64_t epoch = 0)
{return {b::Quality::ValidAtObservation, 10200000000LL, epoch};}
sensor_msgs::msg::Image::SharedPtr image(int ns = 0, bool large = false)
{
  auto m = std::make_shared<sensor_msgs::msg::Image>();
  m->header.frame_id = "camera"; m->header.stamp.sec = 10; m->header.stamp.nanosec = ns;
  m->width = large ? 1920 : 640; m->height = large ? 1080 : 480;
  m->encoding = "rgb8"; m->step = m->width * 3; m->data.assign(m->step * m->height, 10);
  return m;
}
template<typename Message>
std::shared_ptr<Message> metadata(int ns = 0)
{
  auto m = std::make_shared<Message>(); m->header = image(ns)->header; return m;
}
b::CameraPerceptionBinding binding()
{return {"/raw", "/detections", "/tracks", "/targets", "odom", "/ellipses", "/lines"};}
}  // namespace
TEST(CameraStream, CapturesCompletedPairBeforeNewMetadataAndPreservesOriginalLifetime)
{
  b::detail::CameraStream stream(binding()); const auto clock = clock_state();
  stream.image(image(), 1., clock);
  stream.detections(metadata<easyfly_msgs::msg::DetectionInfoArray>(), 1.01, clock);
  stream.detections(metadata<easyfly_msgs::msg::DetectionInfoArray>(100000000), 1.02, clock);
  auto s = stream.snapshot(clock, 1.03);
  ASSERT_TRUE(s.detection_image.value); ASSERT_TRUE(s.detections.value);
  EXPECT_EQ(s.detections.value->header, s.detection_image.value->header);
  EXPECT_EQ(s.detections.value->header.stamp.nanosec, 0U);
  EXPECT_DOUBLE_EQ(s.detections.evidence.first_source_steady, 1.01);
  EXPECT_DOUBLE_EQ(s.detection_image.evidence.first_source_steady, 1.);
  // Further new metadata must not renew the stored pair's original image receipt.
  stream.detections(metadata<easyfly_msgs::msg::DetectionInfoArray>(150000000), 1.9, clock);
  s = stream.snapshot(clock, 2.01); EXPECT_FALSE(s.detection_image.value);
  EXPECT_EQ(s.detections.value->header.stamp.nanosec, 150000000U);
  stream.image(image(150000000), 2.02, clock);
  s = stream.snapshot(clock, 2.03); ASSERT_TRUE(s.detection_image.value);
  EXPECT_EQ(s.detections.value->header, s.detection_image.value->header);
  EXPECT_TRUE(s.detections.value->detections.empty());  // A valid empty pair replaces the old one.
}
TEST(CameraStream, TrackingFallbackKeepsOnlyTheOriginalMatchingDepthTransaction)
{
  b::detail::CameraStream stream(binding()); const auto clock = clock_state();
  stream.image(image(), 1., clock);
  auto track = metadata<easyfly_msgs::msg::TrackInfoArray>(); stream.tracks(track, 1.01, clock);
  auto target = metadata<easyfly_msgs::msg::Target3DArray>(); target->header.frame_id = "odom";
  stream.targets(target, 1.02, clock);
  stream.tracks(metadata<easyfly_msgs::msg::TrackInfoArray>(100000000), 1.03, clock);
  auto newer = metadata<easyfly_msgs::msg::Target3DArray>(100000000);
  newer->header.frame_id = "odom"; stream.targets(newer, 1.04, clock);
  const auto s = stream.snapshot(clock, 1.05);
  ASSERT_TRUE(s.track_image.value); ASSERT_TRUE(s.tracks.value); ASSERT_TRUE(s.targets.value);
  EXPECT_EQ(s.tracks.value->header.stamp, target->header.stamp);
  EXPECT_EQ(s.targets.value->header.stamp, target->header.stamp);
  EXPECT_NE(s.targets.value->header.stamp, newer->header.stamp);
}
TEST(CameraStream, InvalidExpiryRawConflictAndOriginChangesRevokeStoredPairs)
{
  const auto clock = clock_state();
  for (int failure = 0; failure < 5; ++failure) {
    SCOPED_TRACE(failure); b::detail::CameraStream stream(binding());
    stream.source_identity("old"); stream.image(image(), 1., clock);
    auto m = metadata<easyfly_msgs::msg::EllipseObservationArray>();
    m->valid = true; m->image_width = 640; m->image_height = 480;
    stream.ellipses(m, 1.01, clock);
    ASSERT_TRUE(stream.snapshot(clock, 1.02).ellipse_image.value);
    auto next = std::make_shared<easyfly_msgs::msg::EllipseObservationArray>(*m);
    if (failure == 0) {
      next->valid = false; next->reason = "expired_or_clock_invalid";
      stream.ellipses(next, 1.1, clock);
    } else if (failure == 1) {
      next->image_width = 0; stream.ellipses(next, 1.1, clock);
    } else if (failure == 2) {
      auto conflict = image(); conflict->data[0] = 20; stream.image(conflict, 1.1, clock);
    } else if (failure == 3) {stream.source_identity("new");} else {
      EXPECT_FALSE(stream.snapshot(clock_state(1), 1.1).ellipse_image.value); continue;
    }
    EXPECT_FALSE(stream.snapshot(clock, 1.12).ellipse_image.value);
    if (failure == 2) {
      EXPECT_EQ(stream.snapshot(clock, 1.12).ellipses.evidence.quality, b::Quality::Invalid);
      stream.image(image(50000000), 1.13, clock);
      auto newer = std::make_shared<easyfly_msgs::msg::EllipseObservationArray>(*m);
      newer->header.stamp.nanosec = 100000000; stream.ellipses(newer, 1.14, clock);
      EXPECT_EQ(stream.snapshot(clock, 1.15).ellipses.evidence.quality, b::Quality::Invalid);
      EXPECT_FALSE(stream.snapshot(clock, 1.15).ellipse_image.value);
      stream.image(image(100000000), 1.16, clock);
      const auto recovered = stream.snapshot(clock, 1.17);
      ASSERT_TRUE(recovered.ellipse_image.value);
      EXPECT_EQ(recovered.ellipses.evidence.quality, b::Quality::ValidAtObservation);
      EXPECT_EQ(recovered.ellipses.value->header, recovered.ellipse_image.value->header);
      continue;
    }
    EXPECT_EQ(stream.pair_bytes(), 0U);
  }
}
TEST(CameraStream, ExplicitDepthFailureCannotBeOverwrittenByHistoricalTrackDepth)
{
  b::detail::CameraStream stream(binding()); const auto clock = clock_state();
  stream.image(image(), 1., clock);
  auto tracks = metadata<easyfly_msgs::msg::TrackInfoArray>();
  easyfly_msgs::msg::TrackInfo track; track.header = tracks->header;
  track.track_id = 7; track.class_name = "person"; track.confidence = .9;
  track.bbox_center_x = track.bbox_center_y = .5; track.bbox_width = track.bbox_height = .2;
  tracks->tracks.push_back(track); stream.tracks(tracks, 1.01, clock);
  auto target = metadata<easyfly_msgs::msg::Target3DArray>(); target->header.frame_id = "odom";
  easyfly_msgs::msg::Target3D position; position.header = target->header;
  position.track_id = 7; position.class_name = "person"; position.position_valid = true;
  position.confidence = .9; position.distance = position.position_z = 2;
  position.pixel_u = 320; position.pixel_v = 240; target->targets.push_back(position);
  stream.targets(target, 1.02, clock);
  auto next_tracks = metadata<easyfly_msgs::msg::TrackInfoArray>(100000000);
  stream.tracks(next_tracks, 1.03, clock);
  auto invalid = std::make_shared<easyfly_msgs::msg::Target3DArray>(*target);
  invalid->targets[0].distance = 3; stream.targets(invalid, 1.04, clock);  // Same-stamp conflict.
  auto s = stream.snapshot(clock, 1.05);
  ASSERT_TRUE(s.track_image.value); ASSERT_TRUE(s.tracks.value);
  EXPECT_EQ(s.tracks.value->header.stamp.nanosec, 0U);
  EXPECT_FALSE(s.targets.value); EXPECT_EQ(s.targets.evidence.quality, b::Quality::Invalid);
  stream.tracks(next_tracks, 1.055, clock);
  EXPECT_EQ(stream.snapshot(clock, 1.056).targets.evidence.quality, b::Quality::Invalid);
  auto newer = std::make_shared<easyfly_msgs::msg::Target3DArray>(*target);
  newer->header.stamp.nanosec = 100000000; newer->targets[0].header = newer->header;
  stream.targets(newer, 1.06, clock);
  s = stream.snapshot(clock, 1.07); EXPECT_FALSE(s.targets.value);
  stream.image(image(100000000), 1.08, clock);
  s = stream.snapshot(clock, 1.09); ASSERT_TRUE(s.targets.value);
  EXPECT_EQ(s.targets.value->header.stamp, s.tracks.value->header.stamp);
}
TEST(CameraStream, ClassicalFallbackHasHalfSecondLifetimeAndRawAfterMetadataCanCompletePair)
{
  b::detail::CameraStream stream(binding()); const auto clock = clock_state();
  auto m = metadata<easyfly_msgs::msg::ColorLineObservationArray>();
  m->valid = true; m->image_width = 640; m->image_height = 480;
  stream.color_lines(m, 1., clock); stream.image(image(), 1.01, clock);
  auto next = std::make_shared<easyfly_msgs::msg::ColorLineObservationArray>(*m);
  next->header.stamp.nanosec = 100000000; stream.color_lines(next, 1.4, clock);
  ASSERT_TRUE(stream.snapshot(clock, 1.45).color_line_image.value);
  EXPECT_FALSE(stream.snapshot(clock, 1.51).color_line_image.value);
}
TEST(CameraStream, ExtraImagesAreDeduplicatedAndIndependentlyByteBounded)
{
  b::detail::CameraStream stream(binding()); const auto clock = clock_state();
  const auto large = image(0, true); stream.image(large, 1., clock);
  stream.detections(metadata<easyfly_msgs::msg::DetectionInfoArray>(), 1.01, clock);
  stream.tracks(metadata<easyfly_msgs::msg::TrackInfoArray>(), 1.02, clock);
  EXPECT_EQ(stream.pair_bytes(), large->data.size());
  stream.image(image(100000000, true), 1.1, clock);
  stream.tracks(metadata<easyfly_msgs::msg::TrackInfoArray>(100000000), 1.11, clock);
  EXPECT_LE(stream.pair_bytes(), b::CameraObservationBuffer::max_bytes);
  stream.detections(metadata<easyfly_msgs::msg::DetectionInfoArray>(150000000), 1.12, clock);
  const auto s = stream.snapshot(clock, 1.13);
  EXPECT_FALSE(s.detection_image.value); ASSERT_TRUE(s.track_image.value);
  EXPECT_LE(s.cached_bytes, b::CameraObservationBuffer::max_bytes);
  auto replay = image(0, true); replay->data[0] = 20; stream.image(replay, 1.14, clock);
  EXPECT_EQ(stream.snapshot(clock, 1.15).detections.evidence.quality, b::Quality::Invalid);
}
