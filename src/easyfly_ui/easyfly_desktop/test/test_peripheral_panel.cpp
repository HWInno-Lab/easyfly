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
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QTest>
#include "easyfly_desktop/peripheral_panel.hpp"
#include "easyfly_desktop/camera_overlay.hpp"
#include "support.hpp"
namespace ui = easyfly_desktop;
namespace c = easyfly_ui_control;
namespace b = easyfly_ui_bridge;
namespace
{
struct Data
{
  c::PeripheralSnapshot state;
  int creates{0}, singles{0}, angles{0}, trackings{0}; bool inhibited{true};
  easyfly_msgs::srv::ManageSingleTrack::Request roi;
  easyfly_msgs::srv::SetGimbalTracking::Request tracking;
  Data()
  {
    state.running = true; state.single_ready = state.angle_ready = state.tracking_ready = true;
  }
};
class Fake : public ui::PeripheralPort
{
  std::shared_ptr<Data> data;

public:
  explicit Fake(std::shared_ptr<Data> d)
  : data(std::move(d)) {++data->creates;}
  std::shared_ptr<const c::PeripheralSnapshot> snapshot() const override
  {
    data->state.produced_steady = b::detail::steady_seconds();
    return std::make_shared<const c::PeripheralSnapshot>(data->state);
  }
  bool single(const easyfly_msgs::srv::ManageSingleTrack::Request & r) override
  {
    if (data->inhibited) {
      return false;
    }
    ++data->singles; data->roi = r; data->state.pending = true; return true;
  }
  bool angle(const easyfly_msgs::srv::SetGimbalAngle::Request &) override
  {if (data->inhibited) {return false;} ++data->angles; return true;}
  bool tracking(const easyfly_msgs::srv::SetGimbalTracking::Request & r) override
  {if (data->inhibited) {return false;} ++data->trackings; data->tracking = r; return true;}
  void inhibit(bool value) override {data->inhibited = value;}
  void stop() override {data->state.stopped = true;}
};
ui::PeripheralFactory factory(std::shared_ptr<Data> data)
{return [data](const auto &) {return std::make_unique<Fake>(data);};}
std::shared_ptr<ui::ConvertedFrame> frame()
{
  auto f = std::make_shared<ui::ConvertedFrame>();
  f->binding = {101, 0, 0, "/camera/color/image_raw"};
  f->stamp_ns = 10000000000LL; f->source_frame = "camera_color_optical_frame";
  f->first_received = b::detail::steady_seconds();
  f->image = QImage(640, 480, QImage::Format_RGB888);
  f->image.fill(QColor("#dae8f8")); return f;
}
void capture(QWidget & w, const QString & name)
{
  const auto root = qEnvironmentVariable("EASYFLY_UI_CAPTURE_ROOT"); if (root.isEmpty()) {return;}
  QDir d(root); ASSERT_TRUE(d.mkpath(".")); QTest::qWait(50);
  EXPECT_TRUE(w.grab().save(d.filePath(name)));
}
void select(ui::RoiCanvas * canvas)
{
  const auto rect = ui::camera_image_rect(canvas->size(), QSize(640, 480));
  const auto start = rect.topLeft() + QPoint(rect.width() / 4, rect.height() / 4);
  const auto end = rect.topLeft() + QPoint(rect.width() / 2, rect.height() / 2);
  QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier, start);
  QTest::mouseMove(canvas, end); QTest::mouseRelease(canvas, Qt::LeftButton, Qt::NoModifier, end);
}
}  // namespace
TEST(PeripheralPanel, RoiUsesPaintedHeaderRejectsStaleAndNeverRetriesPending)
{
  auto d = std::make_shared<Data>(); ui::PeripheralPanel panel(true, nullptr, factory(d));
  panel.resize(1100, 560); panel.show();
  auto s = desktop_test::sample(); auto f = frame(); panel.observe(s, f, true, true);
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return panel.findChild<QWidget *>("singleRoiCanvas")->isVisible();
      }));
  QTest::qWait(60);
  EXPECT_EQ(d->creates, 0); panel.findChild<QPushButton *>("openPeripherals")->click(); EXPECT_EQ(
    d->creates, 1);
  panel.findChild<QCheckBox *>("peripheralPermit")->setChecked(true);
  auto * canvas = dynamic_cast<ui::RoiCanvas *>(panel.findChild<QWidget *>("singleRoiCanvas"));
  ASSERT_NE(canvas, nullptr); select(canvas);
  panel.observe(s, f, true, true); auto * initialize = panel.findChild<QPushButton *>(
    "initializeSingleRoi");
  ASSERT_TRUE(initialize->isEnabled()); capture(panel, "single-roi-review.png");
  initialize->click(); EXPECT_EQ(d->singles, 1); EXPECT_EQ(d->roi.image_header.stamp.sec, 10);
  EXPECT_EQ(d->roi.image_header.frame_id, "camera_color_optical_frame");
  EXPECT_NEAR(d->roi.roi_x, .25, .01); EXPECT_NEAR(d->roi.roi_width, .25, .01);
  EXPECT_FALSE(initialize->isEnabled()); initialize->click(); EXPECT_EQ(d->singles, 1);
  d->state.pending = false; d->state.uncertain = true; panel.observe(s, f, true, true);
  EXPECT_FALSE(initialize->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("resetSingleRoi")->isEnabled());
  d->state.uncertain = false; f->first_received -= 1; panel.observe(s, f, true, true);
  EXPECT_FALSE(initialize->isEnabled()); EXPECT_EQ(d->singles, 1);
  f = frame(); f->binding.camera = 1; panel.observe(s, f, true, true); EXPECT_FALSE(
    initialize->isEnabled());
  panel.stop(); EXPECT_TRUE(panel.stopped());
}
TEST(PeripheralPanel, PaintedFrameDoesNotChangeUntilPaintAndLetterboxCannotSelect)
{
  ui::RoiCanvas canvas; canvas.resize(700, 450); canvas.show(); auto f = frame(); canvas.frame(f);
  ASSERT_TRUE(desktop_test::until([&] {return static_cast<bool>(canvas.displayed());}));
  ASSERT_TRUE(canvas.displayed()); const auto original = canvas.displayed();
  auto next = std::make_shared<ui::ConvertedFrame>(*f); next->stamp_ns += 100000000;
  canvas.frame(next); EXPECT_EQ(canvas.displayed(), original); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return canvas.displayed() == next;
      }));
  EXPECT_EQ(canvas.displayed(), next);
  QTest::mousePress(&canvas, Qt::LeftButton, Qt::NoModifier, QPoint(1, 1));
  QTest::mouseMove(&canvas, QPoint(200, 200)); QTest::mouseRelease(
    &canvas, Qt::LeftButton,
    Qt::NoModifier,
    QPoint(200, 200));
  EXPECT_TRUE(canvas.selection().isEmpty()); canvas.frame({}); EXPECT_FALSE(canvas.displayed());
}
TEST(PeripheralPanel, GimbalShowsActualStateAndNeverSwitchesSelectedTargetSilently)
{
  auto d = std::make_shared<Data>(); ui::PeripheralPanel panel(true, nullptr, factory(d));
  panel.resize(1100, 600); panel.show();
  auto s = desktop_test::sample(); auto f = frame(); panel.observe(s, f, true, true);
  panel.findChild<QPushButton *>("openPeripherals")->click();
  panel.findChild<QCheckBox *>("peripheralPermit")->setChecked(true);
  auto g = std::make_shared<easyfly_msgs::msg::GimbalState>(); g->is_connected = true;
  g->gimbal_model = "SIMULATION_ONLY_rate_limited"; g->header.frame_id = "base_link";
  g->pitch_angle = 5; d->state.gimbal = g; d->state.gimbal_fresh = true;
  auto a = std::make_shared<easyfly_msgs::msg::Target3DArray>(); easyfly_msgs::msg::Target3D t;
  t.header.frame_id = "odom"; t.header.stamp.sec = 10; t.track_id = 7; t.position_valid = true;
  a->targets.push_back(t); s->camera_perception[0].targets = {a, desktop_test::evidence()};
  panel.observe(s, f, true, true); auto * target = panel.findChild<QComboBox *>("gimbalTargetId");
  auto * enable = panel.findChild<QPushButton *>("enableGimbalTracking");
  EXPECT_EQ(target->currentIndex(), -1); EXPECT_FALSE(enable->isEnabled());
  target->setCurrentIndex(0); panel.observe(s, f, true, true); ASSERT_TRUE(enable->isEnabled());
  enable->click(); EXPECT_EQ(d->trackings, 1); EXPECT_EQ(d->tracking.target_id, 7);
  a = std::make_shared<easyfly_msgs::msg::Target3DArray>(*a); a->targets[0].track_id = 8;
  s->camera_perception[0].targets.value = a; panel.observe(s, f, true, true);
  EXPECT_EQ(target->currentIndex(), -1); EXPECT_FALSE(enable->isEnabled()); EXPECT_EQ(
    d->trackings,
    1);
  auto * actual = panel.findChild<QLabel *>("gimbalActualState"); EXPECT_TRUE(
    actual->text().contains("pitch=5.00"));
  panel.findChild<QTabWidget *>("peripheralTabs")->setCurrentIndex(1); capture(
    panel,
    "gimbal-actual-observation.png");
  d->state.gimbal_fresh = false; panel.observe(s, f, true, true);
  EXPECT_FALSE(panel.findChild<QPushButton *>("setGimbalAngle")->isEnabled());
  EXPECT_FALSE(actual->text().contains("pitch=5.00"));
  panel.observe(s, f, false, true); EXPECT_FALSE(
    panel.findChild<QCheckBox *>(
      "peripheralPermit")->isChecked());
}
