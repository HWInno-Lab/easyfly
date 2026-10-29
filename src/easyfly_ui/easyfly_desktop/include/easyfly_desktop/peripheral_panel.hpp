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
#ifndef EASYFLY_DESKTOP__PERIPHERAL_PANEL_HPP_
#define EASYFLY_DESKTOP__PERIPHERAL_PANEL_HPP_
#include <QWidget>
#include <QRectF>
#include <functional>
#include <memory>
#include <optional>
#include "easyfly_desktop/image_pipeline.hpp"
#include <easyfly_ui_control/peripherals.hpp>
namespace easyfly_desktop
{
class RoiCanvas : public QWidget
{
public:
  explicit RoiCanvas(QWidget * parent = nullptr);
  void frame(std::shared_ptr<const ConvertedFrame> frame);
  std::shared_ptr<const ConvertedFrame> displayed() const;
  QRectF selection() const;
  void tracked(QRectF bounds);

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *) override;
  void mouseMoveEvent(QMouseEvent *) override;
  void mouseReleaseEvent(QMouseEvent *) override;

private:
  std::shared_ptr<const ConvertedFrame> frame_, painted_;
  QRectF roi_, tracked_;
  QPointF begin_;
  bool dragging_{false};
  std::optional<QPointF> normalized(const QPoint &) const;
};
class PeripheralPort
{
public:
  virtual ~PeripheralPort() = default;
  virtual std::shared_ptr<const easyfly_ui_control::PeripheralSnapshot> snapshot() const = 0;
  virtual bool single(const easyfly_msgs::srv::ManageSingleTrack::Request &) = 0;
  virtual bool angle(const easyfly_msgs::srv::SetGimbalAngle::Request &) = 0;
  virtual bool tracking(const easyfly_msgs::srv::SetGimbalTracking::Request &) = 0;
  virtual void inhibit(bool) = 0;
  virtual void stop() = 0;
};
using PeripheralFactory = std::function<std::unique_ptr<PeripheralPort>(
      const easyfly_ui_control::PeripheralConfig &)>;
PeripheralFactory peripheral_factory();
class PeripheralPanel : public QWidget
{
public:
  explicit PeripheralPanel(
    bool commands_enabled, QWidget * parent = nullptr,
    PeripheralFactory factory = peripheral_factory());
  ~PeripheralPanel() override;
  void observe(
    std::shared_ptr<const bridge::UiSnapshot> snapshot,
    std::shared_ptr<const ConvertedFrame> frame, bool available, bool use_sim_time);
  bool unresolved() const;
  bool stopped() const;
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__PERIPHERAL_PANEL_HPP_
