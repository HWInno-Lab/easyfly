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
#include <QComboBox>
#include <QElapsedTimer>
#include <QTableView>
#include <memory>
#include <string>
#include "easyfly_desktop/camera_overlay.hpp"
#include "easyfly_ui_bridge/camera_observations.hpp"
#include "support.hpp"
namespace ui = easyfly_desktop;
namespace b = easyfly_ui_bridge;
namespace
{
class ProcessedCamera : public ::testing::TestWithParam<int>
{
protected:
  std::shared_ptr<desktop_test::FakeState> state;
  std::unique_ptr<ui::MainWindow> window;
  QWidget * canvas{nullptr};
  QComboBox * mode{nullptr};
  QTableView * table{nullptr};
  int control_constructions{0};

  void SetUp() override
  {desktop_test::application(); state = std::make_shared<desktop_test::FakeState>();}
  void TearDown() override
  {
    if (window) {
      window->close(); EXPECT_TRUE(desktop_test::until([&] {return window->cleanup_complete();}));
    }
    EXPECT_EQ(control_constructions, 0); EXPECT_EQ(state->list_calls, 0);
    EXPECT_EQ(state->validate_calls, 0); EXPECT_EQ(state->query_calls, 0);
  }
  b::Observation<sensor_msgs::msg::Image> & selected_image()
  {
    auto & o = state->data->camera_perception[0];
    if (GetParam() == 1) {return o.detection_image;}
    if (GetParam() == 2) {return o.track_image;}
    if (GetParam() == 3) {return o.ellipse_image;}
    return o.color_line_image;
  }
  b::Evidence & selected_evidence()
  {
    auto & o = state->data->camera_perception[0];
    if (GetParam() == 1) {return o.detections.evidence;}
    if (GetParam() == 2) {return o.tracks.evidence;}
    if (GetParam() == 3) {return o.ellipses.evidence;}
    return o.color_lines.evidence;
  }
  void negative(const std::string & reason, const std::string & evidence_reason)
  {
    auto & observations = state->data->camera_perception[0];
    if (GetParam() == 3) {
      auto message = std::make_shared<easyfly_msgs::msg::EllipseObservationArray>(
        *observations.ellipses.value);
      message->valid = false; message->observations.clear(); message->reason = reason;
      observations.ellipses.value = message;
    } else {
      auto message = std::make_shared<easyfly_msgs::msg::ColorLineObservationArray>(
        *observations.color_lines.value);
      message->valid = false; message->observations.clear(); message->reason = reason;
      observations.color_lines.value = message;
    }
    selected_evidence().quality = b::Quality::Invalid;
    selected_evidence().reason = evidence_reason; selected_image() = {};
  }
  void publish(int64_t stamp, bool empty = false, bool matched_image = true, int pixel = 120)
  {
    const double now = b::detail::steady_seconds();
    const auto epoch = state->data->time_epoch;
    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->header.stamp.sec = stamp / 1000000000LL;
    image->header.stamp.nanosec = stamp % 1000000000LL;
    image->header.frame_id = "camera"; image->encoding = "rgb8";
    image->width = 64; image->height = 48; image->step = 192; image->data.assign(9216, pixel);
    auto raw = std::make_shared<sensor_msgs::msg::Image>(*image);
    const int64_t raw_stamp = stamp + 50000000;
    raw->header.stamp.sec = raw_stamp / 1000000000LL;
    raw->header.stamp.nanosec = raw_stamp % 1000000000LL; raw->data.assign(9216, 80);
    auto evidence = desktop_test::evidence(); evidence.time_epoch = epoch;
    state->data->images[0] = {raw, evidence};
    state->data->clock = {b::Quality::ValidAtObservation, raw_stamp, epoch};
    b::CameraPerceptionBinding binding;
    binding.image_topic = "/camera/color/image_raw"; binding.detections_topic = "/detections";
    binding.tracks_topic = "/tracks_raw"; binding.targets_topic = "/target_positions";
    binding.ellipse_topic = "/ellipse_observations";
    binding.color_line_topic = "/color_line_observations";
    b::CameraObservationBuffer buffer(binding);
    if (matched_image) {buffer.image(image, now, epoch);}
    if (GetParam() == 1) {
      auto message = std::make_shared<easyfly_msgs::msg::DetectionInfoArray>();
      message->header = image->header;
      if (!empty) {
        easyfly_msgs::msg::DetectionInfo box; box.header = message->header; box.track_id = -1;
        box.class_name = "person"; box.confidence = .9; box.bbox_center_x = box.bbox_center_y = .5;
        box.bbox_width = box.bbox_height = .2; message->detections.push_back(box);
      }
      buffer.detections(message, now, epoch);
    } else if (GetParam() == 2) {
      auto message = std::make_shared<easyfly_msgs::msg::TrackInfoArray>();
      message->header = image->header;
      if (!empty) {
        easyfly_msgs::msg::TrackInfo box; box.header = message->header; box.track_id = 7;
        box.class_name = "person"; box.confidence = .9; box.bbox_center_x = box.bbox_center_y = .5;
        box.bbox_width = box.bbox_height = .2; message->tracks.push_back(box);
      }
      buffer.tracks(message, now, epoch);
    } else if (GetParam() == 3) {
      auto message = std::make_shared<easyfly_msgs::msg::EllipseObservationArray>();
      message->header = image->header; message->valid = true;
      message->image_width = 64; message->image_height = 48;
      if (!empty) {
        easyfly_msgs::msg::EllipseObservation shape; shape.center_x = 32; shape.center_y = 24;
        shape.major_axis_px = 20; shape.minor_axis_px = 10; shape.angle_rad = .5;
        shape.score = .9; shape.radial_error = .2; message->observations.push_back(shape);
      }
      buffer.ellipses(message, now, epoch);
    } else {
      auto message = std::make_shared<easyfly_msgs::msg::ColorLineObservationArray>();
      message->header = image->header; message->valid = true;
      message->image_width = 64; message->image_height = 48;
      if (!empty) {
        easyfly_msgs::msg::ColorLineObservation shape;
        shape.center_x = 32; shape.center_y = 24; shape.start_x = 16; shape.start_y = 24;
        shape.end_x = 48; shape.end_y = 24; shape.length_px = 32;
        shape.width_px = 4; shape.area_px = 128; shape.score = .9;
        message->observations.push_back(shape);
      }
      buffer.color_lines(message, now, epoch);
    }
    state->data->camera_perception[0] = buffer.snapshot(state->data->clock, now);
  }
  void open()
  {
    ui::ControlFactory controls = [this](const ui::control::ControlConfig &) {
        ++control_constructions; return std::unique_ptr<ui::ControlPort>{};
      };
    window = std::make_unique<ui::MainWindow>(
      b::BridgeConfig{}, desktop_test::factory(state),
      nullptr, ui::default_tool_options(), ui::MissionOptions{}, controls);
    window->show(); ASSERT_TRUE(window->show_page("perception"));
    canvas = window->findChild<QWidget *>("cameraCanvas");
    mode = window->findChild<QComboBox *>("cameraOverlayMode");
    table = window->findChild<QTableView *>("cameraOverlayTable");
    ASSERT_NE(canvas, nullptr); ASSERT_NE(mode, nullptr); ASSERT_NE(table, nullptr);
    mode->setCurrentIndex(GetParam());
  }
  bool ready(int64_t stamp, int count)
  {
    return desktop_test::until(
      [&] {
        return canvas->property("has_frame").toBool() &&
        canvas->property("display_stamp_ns").toLongLong() == stamp &&
        canvas->property("overlay_matched").toBool() &&
        canvas->property("overlay_count").toInt() == count &&
        !canvas->property("frame_frozen").toBool();
      });
  }
  void expect_pixels(int pixel)
  {
    const auto rendered = canvas->grab().toImage();
    const auto bounds = ui::camera_image_rect(
      canvas->size(), QSize(64, 48),
      canvas->property("caption_height").toInt());
    ASSERT_FALSE(bounds.isEmpty());
    const auto color = rendered.pixelColor(
      bounds.x() + bounds.width() / 8,
      bounds.y() + bounds.height() * 7 / 8);
    EXPECT_EQ(color, QColor(pixel, pixel, pixel));
  }
};
TEST_P(ProcessedCamera, MissingMatchKeepsCompletedPairAndFreshEmptyResultReplacesItAtomically)
{
  constexpr int64_t first = 10000000000LL;
  open(); publish(first); ASSERT_TRUE(ready(first, 1)); expect_pixels(120);
  publish(first + 100000000, false, false);
  ASSERT_EQ(selected_image().evidence.quality, b::Quality::Unavailable);
  // Exercise multiple GUI refreshes while newer metadata has no exact source image.
  QElapsedTimer gap; gap.start();
  while (gap.elapsed() < 180) {
    QCoreApplication::processEvents();
    EXPECT_TRUE(canvas->property("has_frame").toBool());
    EXPECT_EQ(canvas->property("display_stamp_ns").toLongLong(), first);
    EXPECT_EQ(canvas->property("overlay_count").toInt(), 1);
    QTest::qWait(10);
  }
  expect_pixels(120);
  constexpr int64_t replacement = first + 200000000;
  publish(replacement, true, true, 150); ASSERT_TRUE(ready(replacement, 0)); expect_pixels(150);
  EXPECT_EQ(table->model()->rowCount(), 0);
  // Use a new nonempty result so expiry must actively remove prior annotations.
  constexpr int64_t annotated = replacement + 100000000;
  publish(annotated, false, true, 170); ASSERT_TRUE(ready(annotated, 1));
  state->data->clock.now_ns = annotated + (GetParam() >= 3 ? 600000000 : 1100000000);
  ASSERT_TRUE(desktop_test::until([&] {return canvas->property("frame_frozen").toBool();}));
  EXPECT_TRUE(canvas->property("has_frame").toBool());
  EXPECT_FALSE(canvas->property("display_stamp_ns").isValid());
  EXPECT_FALSE(canvas->property("overlay_matched").toBool());
  EXPECT_EQ(canvas->property("overlay_count").toInt(), 0);
  EXPECT_EQ(table->model()->rowCount(), 0);
  EXPECT_TRUE(canvas->accessibleDescription().contains(QStringLiteral("已过期")));
  // The real bridge omits matched_image when its metadata becomes stale.
  selected_evidence().quality = b::Quality::Stale; selected_image() = {};
  ASSERT_EQ(selected_image().evidence.quality, b::Quality::NotReceived);
  QElapsedTimer stale_gap; stale_gap.start();
  while (stale_gap.elapsed() < 180) {
    QCoreApplication::processEvents();
    EXPECT_TRUE(canvas->property("has_frame").toBool());
    EXPECT_TRUE(canvas->property("frame_frozen").toBool());
    EXPECT_FALSE(canvas->property("overlay_matched").toBool());
    EXPECT_EQ(canvas->property("overlay_count").toInt(), 0);
    EXPECT_FALSE(canvas->property("display_stamp_ns").isValid());
    EXPECT_EQ(table->model()->rowCount(), 0);
    QTest::qWait(10);
  }
  expect_pixels(170);
  state->data->clock.now_ns = annotated + 5100000000LL;
  ASSERT_TRUE(desktop_test::until([&] {return !canvas->property("has_frame").toBool();}));
  EXPECT_FALSE(canvas->property("frame_frozen").toBool());
}
TEST_P(ProcessedCamera, IdentityChangesAndExplicitInvalidityNeverRetainThePreviousPair)
{
  constexpr int64_t first = 20000000000LL;
  open();
  for (int change = 0; change < 9; ++change) {
    SCOPED_TRACE(change);
    window->findChild<QComboBox *>("cameraSelector")->setCurrentIndex(0);
    mode->setCurrentIndex(GetParam());
    const int64_t stamp = first + change * 200000000LL;
    publish(stamp); ASSERT_TRUE(ready(stamp, 1));
    if (change == 0) {
      mode->setCurrentIndex(GetParam() == 1 ? 2 : 1);
    } else if (change == 1) {
      window->findChild<QComboBox *>("cameraSelector")->setCurrentIndex(1);
    } else if (change == 6) {
      selected_evidence().quality = b::Quality::Invalid;
    } else if (change == 7) {
      state->data->clock.quality = b::Quality::ClockPaused;
    } else if (change == 8) {
      selected_image().evidence.quality = b::Quality::Invalid;
    } else {
      selected_image() = {}; selected_image().evidence.quality = b::Quality::Unavailable;
      if (change == 2) {++state->data->observation_session_id;}
      if (change == 3) {++state->data->time_epoch; ++state->data->clock.epoch;}
      if (change == 4) {state->data->camera_perception[0].binding.tracks_topic += "_changed";}
      if (change == 5) {state->data->camera_perception[0].binding.image_topic = "/other_camera";}
    }
    ASSERT_TRUE(desktop_test::until([&] {return !canvas->property("has_frame").toBool();}));
    EXPECT_FALSE(canvas->property("frame_frozen").toBool());
    EXPECT_FALSE(canvas->property("display_stamp_ns").isValid());
    EXPECT_EQ(canvas->property("overlay_count").toInt(), 0);
    EXPECT_EQ(table->model()->rowCount(), 0);
  }
}
TEST_P(ProcessedCamera, ValidatedClassicalExpiryFreezesPixelsWithoutRenewingOriginalLifetime)
{
  if (GetParam() < 3) {GTEST_SKIP() << "Only classical geometry has validated negative expiry";}
  constexpr int64_t stamp = 30000000000LL;
  open(); publish(stamp, false, true, 170); ASSERT_TRUE(ready(stamp, 1));
  negative("expired_or_clock_invalid", b::kValidatedGeometryExpiry);
  // The old positive pair is still within its original 0.5-second source TTL.
  ASSERT_LT(state->data->clock.now_ns - stamp, 500000000LL);
  ASSERT_TRUE(desktop_test::until([&] {return canvas->property("frame_frozen").toBool();}));
  EXPECT_TRUE(canvas->property("has_frame").toBool()); expect_pixels(170);
  EXPECT_FALSE(canvas->property("overlay_matched").toBool());
  EXPECT_EQ(canvas->property("overlay_count").toInt(), 0);
  EXPECT_FALSE(canvas->property("display_stamp_ns").isValid());
  EXPECT_EQ(table->model()->rowCount(), 0);
  // Repeated expiry reports cannot extend the original displayed image's lifetime.
  state->data->clock.now_ns = stamp + 4900000000LL;
  QTest::qWait(120);
  EXPECT_TRUE(canvas->property("has_frame").toBool());
  EXPECT_TRUE(canvas->property("frame_frozen").toBool()); expect_pixels(170);
  selected_evidence().reason = b::kValidatedGeometryExpiry;
  state->data->clock.now_ns = stamp + 5100000000LL;
  ASSERT_TRUE(desktop_test::until([&] {return !canvas->property("has_frame").toBool();}));
  EXPECT_FALSE(canvas->property("frame_frozen").toBool());
}
TEST_P(ProcessedCamera, UnverifiedClassicalNegativeCannotReuseValidatedExpiryHistory)
{
  if (GetParam() < 3) {GTEST_SKIP() << "Only classical geometry carries negative result reasons";}
  open();
  for (int variant = 0; variant < 2; ++variant) {
    SCOPED_TRACE(variant);
    const int64_t stamp = 40000000000LL + variant * 200000000LL;
    publish(stamp, false, true, 170); ASSERT_TRUE(ready(stamp, 1));
    if (variant == 0) {
      negative("expired_or_clock_invalid", std::string(b::kValidatedGeometryExpiry) + " unknown");
    } else {negative("processing_expired", "processing_expired");}
    ASSERT_TRUE(desktop_test::until([&] {return !canvas->property("has_frame").toBool();}));
    EXPECT_FALSE(canvas->property("frame_frozen").toBool());
    EXPECT_FALSE(canvas->property("display_stamp_ns").isValid());
    EXPECT_EQ(canvas->property("overlay_count").toInt(), 0);
    EXPECT_EQ(table->model()->rowCount(), 0);
  }
}
std::string mode_name(const ::testing::TestParamInfo<int> & info)
{
  const char * names[] = {"Raw", "Detections", "Tracks", "Ellipses", "ColorLines"};
  return names[info.param];
}
INSTANTIATE_TEST_SUITE_P(
  AllProcessedModes, ProcessedCamera, ::testing::Values(1, 2, 3, 4), mode_name);
}  // namespace
