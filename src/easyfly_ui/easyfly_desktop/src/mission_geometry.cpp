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

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include "easyfly_desktop/mission_geometry.hpp"
namespace easyfly_desktop
{
DraftGeometry draft_geometry(const easyfly_mission_fsm::MissionValidation & plan)
{
  DraftGeometry result;
  if (!plan.valid() || plan.steps.size() > 256) {result.error = "无有效有界任务"; return result;}
  const auto bounded = [](double v) {return std::isfinite(v) && std::abs(v) <= 1000000;};
  std::optional<size_t> previous;
  for (size_t i = 0; i < plan.steps.size(); ++i) {
    const auto & s = plan.steps[i];
    const bool unlocated = s.type == "hover" || s.type == "takeoff" || s.type == "track" ||
      s.type == "land" || s.type == "precision_land";
    if (s.type == "waypoint" || s.type == "navigate") {
      if (!bounded(s.x) || !bounded(s.y) || !bounded(s.z)) {
        result = {}; result.error = "几何包含无效坐标"; return result;
      }
      const size_t current = result.points.size();
      result.points.push_back({i, s.x, s.y, s.z, s.type});
      if (previous) {result.links.push_back({*previous, current});} previous = current;
    } else if (s.type == "search") {
      if (!bounded(s.center_x) || !bounded(s.center_y) || !bounded(s.altitude) ||
        !bounded(s.area_width) || !bounded(s.area_height) || s.area_width <= 0 ||
        s.area_height <= 0)
      {
        result = {}; result.error = "几何包含无效区域"; return result;
      }
      result.areas.push_back(
        {i, s.center_x - s.area_width / 2, s.center_y - s.area_height / 2,
          s.area_width, s.area_height, s.altitude}); previous.reset();
    } else if (unlocated) {
      result.unlocated.push_back(i);
      if (s.type != "hover") {previous.reset();}
    } else {result = {}; result.error = "几何含未知步骤类型"; return result;}
  }
  result.accepted = true; return result;
}
MissionGeometryCanvas::MissionGeometryCanvas(QWidget * parent)
: QWidget(parent)
{
  setObjectName("draftGeometry"); setMinimumSize(220, 180);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setAccessibleName(QStringLiteral("只读 odom XY 草稿几何示意"));
  setToolTip(QStringLiteral("滚轮缩放，中键平移，单击图形选择步骤；不写任务或发导航"));
}
QSize MissionGeometryCanvas::sizeHint() const {return {380, 260};}
const DraftGeometry & MissionGeometryCanvas::geometry() const {return data_;}
void MissionGeometryCanvas::plan(const easyfly_mission_fsm::MissionValidation & p)
{
  data_ = draft_geometry(p); selected_ = -1; reason_ = QString::fromStdString(data_.error); fit();
  setProperty("selected_step", -1);
  panning_ = false; unsetCursor();
  setProperty("geometry_ready", data_.accepted); update();
}
void MissionGeometryCanvas::invalidate(const QString & reason)
{
  data_ = {}; selected_ = -1; reason_ = reason; panning_ = false; unsetCursor();
  setProperty("selected_step", -1);
  setProperty("geometry_ready", false); update();
}
void MissionGeometryCanvas::select_step(int step)
{selected_ = step; setProperty("selected_step", step); update();}
void MissionGeometryCanvas::selection_handler(std::function<void(size_t)> h)
{selected_handler_ = std::move(h);}
QRectF MissionGeometryCanvas::plot() const
{
  return {46, 18, static_cast<double>(std::max(1, width() - 62)),
    static_cast<double>(std::max(1, height() - 62))};
}
double MissionGeometryCanvas::scale() const
{return std::min(plot().width() / span_x_, plot().height() / span_y_) * zoom_;}
QPointF MissionGeometryCanvas::world_to_screen(double x, double y) const
{return plot().center() + QPointF((x - center_x_) * scale(), -(y - center_y_) * scale());}
QPointF MissionGeometryCanvas::screen_to_world(const QPointF & p) const
{
  return {center_x_ + (p.x() - plot().center().x()) / scale(),
    center_y_ - (p.y() - plot().center().y()) / scale()};
}
void MissionGeometryCanvas::fit()
{
  double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
  double max_x = -min_x, max_y = -min_y;
  const auto add = [&](double x, double y) {
      min_x = std::min(min_x, x); max_x = std::max(max_x, x);
      min_y = std::min(min_y, y); max_y = std::max(max_y, y);
    };
  for (const auto & p : data_.points) {
    add(p.x, p.y);
  }
  for (const auto & a : data_.areas) {
    add(a.x, a.y); add(a.x + a.width, a.y + a.height);
  }
  if (!std::isfinite(min_x)) {min_x = min_y = -1; max_x = max_y = 1;}
  center_x_ = (min_x + max_x) / 2; center_y_ = (min_y + max_y) / 2;
  span_x_ = std::max(2.0, (max_x - min_x) * 1.3);
  span_y_ = std::max(2.0, (max_y - min_y) * 1.3); zoom_ = 1; update();
}
void MissionGeometryCanvas::zoom(double factor)
{
  if (!data_.accepted || !std::isfinite(factor) || factor <= 0) {return;}
  zoom_ = std::clamp(zoom_ * factor, .25, 16.0); update();
}
double MissionGeometryCanvas::zoom_level() const {return zoom_;}
void MissionGeometryCanvas::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this); painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), QColor("#ffffff")); const auto box = plot();
  painter.setPen(QColor("#cbd5e1")); painter.drawRect(box);
  if (!data_.accepted || (data_.points.empty() && data_.areas.empty())) {
    painter.setPen(QColor("#526176"));
    painter.drawText(
      box.adjusted(10, 10, -10, -10), Qt::AlignCenter | Qt::TextWordWrap,
      data_.accepted ? QStringLiteral(
        "此草稿没有明确 XY 目标或搜索区域。\n不假设飞机起点或动态目标位置。") : reason_);
    return;
  }
  const auto low = screen_to_world(box.bottomLeft()), high = screen_to_world(box.topRight());
  const double raw_tick = std::max(high.x() - low.x(), high.y() - low.y()) / 5;
  const double base = std::pow(10.0, std::floor(std::log10(raw_tick)));
  const double tick = base * (raw_tick / base > 5 ? 10 : raw_tick / base > 2 ? 5 : 2);
  painter.save(); painter.setClipRect(box);
  const auto label = [&](const QPointF & center, const QString & text) {
      if (!box.contains(center)) {return;}
      const auto metrics = painter.fontMetrics();
      const auto fitted =
        metrics.elidedText(text, Qt::ElideRight, static_cast<int>(box.width() - 6));
      const double right = box.right() - metrics.horizontalAdvance(fitted) - 3;
      const double x = std::clamp(center.x() + 9, box.left() + 3, std::max(box.left() + 3, right));
      const double y = std::clamp(center.y() - 9, box.top() + metrics.height(), box.bottom() - 3);
      painter.drawText(QPointF(x, y), fitted);
    };
  painter.setPen(QPen(QColor("#e5eaf0"), 1));
  for (double x = std::ceil(low.x() / tick) * tick; x <= high.x(); x += tick) {
    const double px = world_to_screen(x, 0).x();
    painter.drawLine(QPointF(px, box.top()), QPointF(px, box.bottom()));
  }
  for (double y = std::ceil(low.y() / tick) * tick; y <= high.y(); y += tick) {
    const double py = world_to_screen(0, y).y();
    painter.drawLine(QPointF(box.left(), py), QPointF(box.right(), py));
  }
  painter.setPen(QPen(QColor("#94a3b8"), 1, Qt::DashDotLine));
  const auto origin = world_to_screen(0, 0);
  painter.drawLine(QPointF(origin.x(), box.top()), QPointF(origin.x(), box.bottom()));
  painter.drawLine(QPointF(box.left(), origin.y()), QPointF(box.right(), origin.y()));
  for (const auto & a : data_.areas) {
    const bool selected = static_cast<int>(a.step) == selected_;
    const QRectF area(world_to_screen(a.x, a.y + a.height), world_to_screen(a.x + a.width, a.y));
    painter.setPen(QPen(QColor(selected ? "#d97706" : "#0d9488"), selected ? 3 : 1.5));
    painter.setBrush(QColor(13, 148, 136, 28)); painter.drawRect(area);
    if (data_.areas.size() <= 24 || selected) {
      if (area.width() >= 68 && area.height() >= 36) {
        painter.drawText(
          area.adjusted(3, 3, -3, -3), Qt::AlignCenter,
          QString("#%1 search\nz=%2m").arg(a.step + 1).arg(a.z, 0, 'g', 6));
      } else {label(area.center(), QString("#%1 z=%2m").arg(a.step + 1).arg(a.z, 0, 'g', 6));}
    }
  }
  painter.setBrush(Qt::NoBrush);
  for (const auto & link : data_.links) {
    const auto & a = data_.points[link.from]; const auto & b = data_.points[link.to];
    const auto from = world_to_screen(a.x, a.y), to = world_to_screen(b.x, b.y);
    const bool selected = static_cast<int>(b.step) == selected_;
    painter.setPen(
      QPen(
        QColor(selected ? "#d97706" : "#64748b"), selected ? 2 : 1.5,
        Qt::DashLine));
    painter.drawLine(from, to);
    const auto difference = to - from; const double distance = std::hypot(
      difference.x(), difference.y());
    if (distance > 24) {
      const auto d = difference / distance, normal = QPointF(-d.y(), d.x());
      const auto tip = from + difference * .6;
      painter.drawLine(tip, tip - d * 8 + normal * 4); painter.drawLine(
        tip,
        tip - d * 8 - normal * 4);
    }
  }
  for (const auto & p : data_.points) {
    const auto center = world_to_screen(p.x, p.y);
    const bool selected = static_cast<int>(p.step) == selected_;
    painter.setPen(QPen(QColor(selected ? "#d97706" : "#1769df"), selected ? 3 : 1.5));
    painter.setBrush(QColor("#ffffff")); painter.drawEllipse(
      center, selected ? 7 : 5,
      selected ? 7 : 5);
    if (data_.points.size() <= 24 || selected) {
      label(center, QString("#%1 z=%2m").arg(p.step + 1).arg(p.z, 0, 'g', 6));
    }
  }
  painter.restore(); painter.setPen(QColor("#526176"));
  for (double x = std::ceil(low.x() / tick) * tick; x <= high.x(); x += tick) {
    painter.drawText(
      QRectF(world_to_screen(x, 0).x() - 28, box.bottom() + 3, 56, 18),
      Qt::AlignCenter, QString::number(x, 'g', 5));
  }
  for (double y = std::ceil(low.y() / tick) * tick; y <= high.y(); y += tick) {
    painter.drawText(
      QRectF(0, world_to_screen(0, y).y() - 9, 42, 18),
      Qt::AlignRight | Qt::AlignVCenter, QString::number(y, 'g', 5));
  }
  painter.drawText(
    QRectF(4, height() - 21, width() - 8, 18),
    Qt::AlignCenter, QStringLiteral("odom · X+ → / Y+ ↑ · m · 虚线仅步骤顺序"));
}
void MissionGeometryCanvas::wheelEvent(QWheelEvent * e)
{
  if (e->angleDelta().y()) {zoom(e->angleDelta().y() > 0 ? 1.25 : .8);} e->accept();
}
void MissionGeometryCanvas::mousePressEvent(QMouseEvent * e)
{
  if (!data_.accepted || !plot().contains(e->localPos())) {return;}
  if (e->button() == Qt::MiddleButton) {
    panning_ = true; last_mouse_ = e->localPos(); setCursor(Qt::ClosedHandCursor); e->accept();
    return;
  }
  if (e->button() != Qt::LeftButton || !selected_handler_) {return;}
  std::optional<size_t> picked; double best = 12;
  for (const auto & p : data_.points) {
    const auto delta = world_to_screen(p.x, p.y) - e->localPos();
    const double distance = std::hypot(delta.x(), delta.y());
    if (distance < best) {best = distance; picked = p.step;}
  }
  if (!picked) {
    const auto world = screen_to_world(e->localPos());
    for (const auto & a : data_.areas) {
      if (world.x() >= a.x && world.x() <= a.x + a.width && world.y() >= a.y &&
        world.y() <= a.y + a.height)
      {
        picked = a.step; break;
      }
    }
  }
  if (picked) {selected_handler_(*picked);} e->accept();
}
void MissionGeometryCanvas::mouseMoveEvent(QMouseEvent * e)
{
  if (!panning_ || !data_.accepted) {return;}
  const auto delta = e->localPos() - last_mouse_; last_mouse_ = e->localPos();
  center_x_ = std::clamp(center_x_ - delta.x() / scale(), -2000000.0, 2000000.0);
  center_y_ = std::clamp(center_y_ + delta.y() / scale(), -2000000.0, 2000000.0); update();
  e->accept();
}
void MissionGeometryCanvas::mouseReleaseEvent(QMouseEvent * e)
{
  if (e->button() == Qt::MiddleButton) {panning_ = false; unsetCursor(); e->accept();}
}
}  // namespace easyfly_desktop
