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
#include <limits>
#include "easyfly_desktop/camera_overlay.hpp"
#include "support.hpp"
namespace ui = easyfly_desktop;
namespace b = easyfly_ui_bridge;
namespace
{
struct Sample
{
  std::shared_ptr<b::UiSnapshot> snapshot{desktop_test::sample()};
  ui::ConvertedFrame frame;
  std::shared_ptr<easyfly_msgs::msg::TrackInfoArray> tracks{
    std::make_shared<easyfly_msgs::msg::TrackInfoArray>()};
  std::shared_ptr<easyfly_msgs::msg::Target3DArray> targets{
    std::make_shared<easyfly_msgs::msg::Target3DArray>()};
  double now{b::detail::steady_seconds()};
  Sample()
  {
    frame.binding = {101, 0, 0, "/camera/color/image_raw"}; frame.stamp_ns = 10000000000LL;
    frame.first_received = now; frame.source_frame = "camera";
    frame.image = QImage(640, 480, QImage::Format_RGB888); frame.image.fill(Qt::blue);
    tracks->header.frame_id = "camera"; tracks->header.stamp.sec = 10;
    easyfly_msgs::msg::TrackInfo box; box.header = tracks->header; box.track_id = 7;
    box.class_name = "person"; box.confidence = .8; box.bbox_center_x = box.bbox_center_y = .5;
    box.bbox_width = .2; box.bbox_height = .4; tracks->tracks.push_back(box);
    targets->header = tracks->header; targets->header.frame_id = "odom";
    easyfly_msgs::msg::Target3D target; target.header = targets->header; target.track_id = 7;
    target.class_name = "person"; target.position_valid = true; target.distance = 5.5;
    target.pixel_u = 320; target.pixel_v = 240; target.position_x = 5;
    targets->targets.push_back(target);
    auto & observations = snapshot->camera_perception[0];
    observations.binding =
    {"/camera/color/image_raw", "/detections", "/tracks_raw", "/target_positions", "odom"};
    observations.tracks = {tracks, desktop_test::evidence()};
    observations.targets = {targets, desktop_test::evidence()};
  }
  ui::CameraOverlay render()
  {return ui::camera_overlay(*snapshot, frame, 0, ui::CameraOverlayMode::Tracks, now);}
};
}  // namespace
TEST(CameraOverlay, UniqueSameStampRawIdClassAndPixelMatch)
{
  Sample sample; const auto value = sample.render();
  ASSERT_TRUE(value.matched); ASSERT_EQ(value.boxes.size(), 1U);
  EXPECT_EQ(value.boxes[0].id, 7); ASSERT_TRUE(value.boxes[0].axial_depth);
  EXPECT_DOUBLE_EQ(*value.boxes[0].axial_depth, 5.5);
  EXPECT_TRUE(value.boxes[0].label.contains("SORT #7"));
  EXPECT_NEAR(value.boxes[0].normalized.width(), .2, 1e-6);
}
TEST(CameraOverlay, UnknownInvalidAndAmbiguousDepthNeverBecomesZero)
{
  Sample sample; sample.targets->targets[0].position_valid = false;
  EXPECT_FALSE(sample.render().boxes[0].axial_depth);
  sample.targets->targets[0].position_valid = true; sample.targets->targets[0].distance = 0;
  EXPECT_FALSE(sample.render().boxes[0].axial_depth);
  sample.targets->targets[0].distance = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(sample.render().boxes[0].axial_depth);
  sample.targets->targets[0].distance = 5; sample.targets->targets[0].pixel_u = 0;
  EXPECT_FALSE(sample.render().boxes[0].axial_depth);
  sample.targets->targets[0].pixel_u = 320;
  sample.targets->targets.push_back(sample.targets->targets[0]);
  EXPECT_FALSE(sample.render().boxes[0].axial_depth);
  sample.targets->targets.pop_back(); sample.targets->targets[0].class_name = "truck";
  EXPECT_FALSE(sample.render().boxes[0].axial_depth);
}
TEST(CameraOverlay, StampFrameCameraSessionAndEpochAreNotInterchangeable)
{
  Sample sample; sample.frame.stamp_ns += 1; EXPECT_FALSE(sample.render().matched);
  sample.frame.stamp_ns -= 1; sample.frame.source_frame = "other"; EXPECT_FALSE(
    sample.render().matched);
  sample.frame.source_frame = "camera"; sample.frame.binding.camera = 1;
  EXPECT_FALSE(sample.render().matched); sample.frame.binding.camera = 0;
  sample.frame.binding.session = 102; EXPECT_FALSE(sample.render().matched);
  sample.frame.binding.session = 101; sample.frame.binding.epoch = 1;
  EXPECT_FALSE(sample.render().matched); sample.frame.binding.epoch = 0;
  sample.frame.binding.topic = "/other"; EXPECT_FALSE(sample.render().matched);
}
TEST(CameraOverlay, PausedStaleSourceAndSteadyReceiptsClear)
{
  Sample sample; sample.snapshot->clock.quality = b::Quality::ClockPaused;
  EXPECT_FALSE(sample.render().matched);
  sample.snapshot->clock.quality = b::Quality::ValidAtObservation;
  sample.now += 1.1; EXPECT_FALSE(sample.render().matched);
  sample.now -= 1.1;
  sample.snapshot->clock.epoch = 1; EXPECT_FALSE(sample.render().matched);
  sample.snapshot->clock.epoch = 0; sample.snapshot->clock.now_ns += 1100000000LL;
  EXPECT_FALSE(sample.render().matched);
}
TEST(CameraOverlay, EmptyDuplicateIdsAndInvalidBoxesAreDistinct)
{
  Sample sample; sample.tracks->tracks.clear();
  EXPECT_TRUE(sample.render().matched); EXPECT_TRUE(sample.render().boxes.empty());
  Sample another; another.tracks->tracks.push_back(another.tracks->tracks[0]);
  EXPECT_FALSE(another.render().matched);
  another.tracks->tracks.pop_back(); another.tracks->tracks[0].bbox_width = -1;
  EXPECT_FALSE(another.render().matched);
}
TEST(CameraOverlay, LetterboxReservesCaptionAndClipsNormalizedBoxes)
{
  EXPECT_EQ(ui::camera_image_rect(QSize(800, 648), QSize(640, 480)), QRect(0, 0, 800, 600));
  EXPECT_EQ(ui::camera_image_rect(QSize(1000, 648), QSize(640, 480)), QRect(100, 0, 800, 600));
  EXPECT_TRUE(ui::camera_image_rect(QSize(640, 40), QSize(640, 480)).isEmpty());
  Sample sample; sample.tracks->tracks[0].bbox_center_x = 0;
  sample.targets->targets[0].pixel_u = 0;
  auto result = sample.render(); ASSERT_TRUE(result.matched);
  EXPECT_NEAR(result.boxes[0].normalized.left(), 0, 1e-6);
  EXPECT_NEAR(result.boxes[0].normalized.width(), .1, 1e-6);
  EXPECT_TRUE(result.boxes[0].axial_depth);
}
TEST(CameraOverlay, DenseLabelsDoNotOverlapOrLeaveImage)
{
  const QRect image(100, 50, 500, 300), bounds(200, 220, 100, 100);
  std::vector<QRect> occupied;
  for (int i = 0; i < 8; ++i) {
    const auto label = ui::camera_label_rect(bounds, image, QSize(180, 24), occupied);
    ASSERT_FALSE(label.isEmpty()); EXPECT_TRUE(image.contains(label));
    for (const auto & previous : occupied) {
      EXPECT_FALSE(label.intersects(previous));
    }
    occupied.push_back(label);
  }
  EXPECT_TRUE(ui::camera_label_rect(bounds, QRect{}, QSize(180, 24), occupied).isEmpty());
  const auto clipped = ui::camera_label_rect(bounds, QRect(0, 0, 50, 30), QSize(180, 24), {});
  EXPECT_TRUE(QRect(0, 0, 50, 30).contains(clipped));
}
