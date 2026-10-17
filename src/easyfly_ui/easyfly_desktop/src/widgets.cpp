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

#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "easyfly_desktop/presentation.hpp"
#include "widgets.hpp"
namespace easyfly_desktop
{
QIcon dashboard_icon(const QString & name)
{
  QPixmap image(32, 32); image.fill(Qt::transparent);
  QPainter p(&image); p.setRenderHint(QPainter::Antialiasing);
  p.setPen(QPen(QColor("#1769df"), 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  if (name == "link") {
    p.drawEllipse(QRectF(4, 10, 13, 12)); p.drawEllipse(QRectF(15, 10, 13, 12));
    p.drawLine(12, 16, 20, 16);
  } else if (name == "battery") {
    p.drawRoundedRect(QRectF(4, 9, 22, 14), 2, 2); p.drawLine(29, 13, 29, 19);
    p.fillRect(QRectF(8, 13, 13, 6), QColor("#1769df"));
  } else if (name == "camera") {
    p.drawRoundedRect(QRectF(3, 9, 26, 19), 3, 3); p.drawEllipse(QPointF(16, 18), 5, 5);
    p.drawLine(9, 9, 12, 5); p.drawLine(12, 5, 20, 5); p.drawLine(20, 5, 23, 9);
  } else if (name == "location" || name == "map") {
    p.drawEllipse(QPointF(16, 12), 7, 7); p.drawEllipse(QPointF(16, 12), 2, 2);
    p.drawLine(10, 17, 16, 28); p.drawLine(22, 17, 16, 28);
  } else if (name == "speed") {
    p.drawArc(QRectF(4, 5, 24, 24), 0, 180 * 16); p.drawLine(16, 17, 24, 10);
    p.drawLine(5, 27, 27, 27); p.drawEllipse(QPointF(16, 17), 2, 2);
  } else if (name == "mode" || name == "overview") {
    for (const auto & center : {QPointF(7, 7), QPointF(25, 7), QPointF(7, 25), QPointF(25, 25)}) {
      p.drawEllipse(center, 4, 4); p.drawLine(QPointF(16, 16), center);
    }
    p.drawEllipse(QPointF(16, 16), 3, 3);
  } else if (name == "tools" || name == "diagnostics") {
    for (int i = 0; i < 3; ++i) {
      const int x = 7 + i * 9; p.drawLine(x, 4, x, 28);
      p.drawEllipse(QPointF(x, 10 + i * 5), 3, 3);
    }
  } else {
    p.drawRoundedRect(QRectF(6, 3, 20, 26), 2, 2);
    p.drawLine(11, 10, 21, 10); p.drawLine(11, 16, 21, 16); p.drawLine(11, 22, 18, 22);
  }
  return QIcon(image);
}
FlightInstrumentCanvas::FlightInstrumentCanvas(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(220, 190); setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  setAccessibleName(QStringLiteral("飞机姿态与局部 ENU 航向，只读观测"));
}
void FlightInstrumentCanvas::snapshot(const bridge::UiSnapshot & s, double now)
{
  instruments_ = flight_instruments(s, now);
  setProperty("attitude_valid", instruments_.attitude_valid);
  setProperty(
    "heading_degrees", instruments_.attitude_valid ?
    QVariant(instruments_.heading_degrees) : QVariant{});
  const auto caption = instruments_.attitude_valid ?
    QStringLiteral("横滚 %1° · 俯仰 %2° · 局部航向 %3°")
    .arg(
    number(instruments_.roll_degrees, 1), number(instruments_.pitch_degrees, 1),
    number(instruments_.heading_degrees, 1)) : QStringLiteral("姿态/航向不可用；不绘制指针");
  setAccessibleDescription(caption);
  setToolTip(
    caption + QStringLiteral(
      "\n航向从局部 +Y 起顺时针；不是磁罗盘或地理真北。"
      "\n姿态使用 ROS ENU/FLU 约定，正俯仰为低头。"));
  update();
}
void FlightInstrumentCanvas::paintEvent(QPaintEvent * event)
{
  static_cast<void>(event);
  QPainter p(this); p.setRenderHint(QPainter::Antialiasing); p.fillRect(rect(), Qt::white);
  const double radius = std::max(30.0, std::min(width() / 4.0 - 14, (height() - 68) / 2.0));
  const QPointF attitude(width() * .25, radius + 29), compass(width() * .75, radius + 29);
  p.setPen(QColor("#526176"));
  p.drawText(QRectF(0, 0, width() / 2.0, 22), Qt::AlignCenter, QStringLiteral("姿态"));
  p.drawText(
    QRectF(width() / 2.0, 0, width() / 2.0, 22), Qt::AlignCenter,
    QStringLiteral("局部航向"));
  if (instruments_.attitude_valid) {
    p.save(); QPainterPath clip; clip.addEllipse(attitude, radius, radius); p.setClipPath(clip);
    p.translate(attitude); p.rotate(-instruments_.roll_degrees);
    // In ROS FLU, positive pitch tips the nose down: the horizon moves upward.
    const double offset = -instruments_.pitch_degrees * radius / 45;
    p.fillRect(
      QRectF(-radius * 3, -radius * 4, radius * 6, radius * 4 + offset),
      QColor("#e2f2ff"));
    p.fillRect(QRectF(-radius * 3, offset, radius * 6, radius * 4), QColor("#e7ecf1"));
    p.setPen(QPen(QColor("#64748b"), 1.5));
    p.drawLine(QPointF(-radius * 2, offset), QPointF(radius * 2, offset));
    for (int angle : {-30, -20, -10, 10, 20, 30}) {
      const double y = offset + angle * radius / 45;
      p.drawLine(QPointF(-radius * .3, y), QPointF(radius * .3, y));
    }
    p.restore();
    p.setPen(QPen(QColor("#1769df"), 3));
    p.drawLine(attitude + QPointF(-radius * .55, 0), attitude + QPointF(-8, 0));
    p.drawLine(attitude + QPointF(8, 0), attitude + QPointF(radius * .55, 0));
    p.drawEllipse(attitude, 3, 3);
  }
  p.setPen(QPen(QColor("#cbd5e1"), 1.5)); p.setBrush(Qt::NoBrush);
  p.drawEllipse(attitude, radius, radius); p.drawEllipse(compass, radius, radius);
  p.save(); p.translate(compass);
  for (int angle = 0; angle < 360; angle += 15) {
    p.save(); p.rotate(angle); p.drawLine(
      QPointF(0, -radius + 2),
      QPointF(0, -radius + (angle % 90 == 0 ? 10 : 6))); p.restore();
  }
  p.setPen(QColor("#526176"));
  const double offset = radius - 20;
  p.drawText(QRectF(-12, -offset - 8, 24, 16), Qt::AlignCenter, "N");
  p.drawText(QRectF(-12, offset - 8, 24, 16), Qt::AlignCenter, "S");
  p.drawText(QRectF(offset - 12, -8, 24, 16), Qt::AlignCenter, "E");
  p.drawText(QRectF(-offset - 12, -8, 24, 16), Qt::AlignCenter, "W");
  if (instruments_.attitude_valid) {
    p.rotate(instruments_.heading_degrees);
    p.setPen(QPen(QColor("#1769df"), 1.5)); p.setBrush(QColor("#1769df"));
    p.drawPolygon(
      QPolygonF(
        QVector<QPointF>{QPointF(0, -radius * .57), QPointF(7, 10),
          QPointF(0, 5), QPointF(-7, 10)}));
  }
  p.restore(); p.setPen(QColor("#344256"));
  if (instruments_.attitude_valid) {
    const int y = static_cast<int>(radius * 2 + 36);
    p.drawText(
      QRect(0, y, width() / 2, 20), Qt::AlignCenter,
      QStringLiteral("R %1° / P %2°").arg(
        number(instruments_.roll_degrees, 1),
        number(instruments_.pitch_degrees, 1)));
    p.drawText(
      QRect(width() / 2, y, width() / 2, 20), Qt::AlignCenter,
      number(instruments_.heading_degrees, 1) + QStringLiteral("°"));
  } else {
    for (const auto & center : {attitude, compass}) {
      p.drawText(QRectF(center.x() - 30, center.y() - 10, 60, 20), Qt::AlignCenter, "—");
    }
    p.drawText(
      QRect(0, static_cast<int>(radius * 2 + 36), width(), 22), Qt::AlignCenter,
      QStringLiteral("姿态 / 航向不可用"));
  }
}
TextTable::TextTable(QStringList headers, QObject * parent)
: QAbstractTableModel(parent), headers_(std::move(headers)) {}
int TextTable::rowCount(const QModelIndex & parent) const
{
  return parent.isValid() ? 0 : rows_.size();
}
int TextTable::columnCount(const QModelIndex & parent) const
{
  return parent.isValid() ? 0 : headers_.size();
}
QVariant TextTable::data(const QModelIndex & index, int role) const
{
  if (!index.isValid() || index.row() >= rows_.size() ||
    index.column() >= rows_[index.row()].size()) {return {};}
  if (role == Qt::DisplayRole) {return rows_[index.row()][index.column()];}
  return {};
}
QVariant TextTable::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (role == Qt::DisplayRole && orientation == Qt::Horizontal && section >= 0 &&
    section < headers_.size())
  {
    return headers_[section];
  }
  return {};
}
void TextTable::replace(QVector<QStringList> rows)
{
  if (rows == rows_) {return;}
  beginResetModel(); rows_ = std::move(rows); endResetModel();
}
CameraCanvas::CameraCanvas(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(240); setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}
void CameraCanvas::frame(QImage image, const QString & caption)
{
  image_ = std::move(image); caption_ = caption;
  if (image_.isNull()) {overlay({});}
  setProperty("has_frame", !image_.isNull()); update();
}
void CameraCanvas::overlay(CameraOverlay overlay)
{
  overlay_ = std::move(overlay);
  setProperty("overlay_matched", overlay_.matched);
  setProperty("overlay_count", static_cast<int>(overlay_.boxes.size()));
  setProperty("hidden_overlay_labels", 0);
  update();
}
void CameraCanvas::paintEvent(QPaintEvent * event)
{
  static_cast<void>(event);
  QPainter painter(this); painter.fillRect(rect(), Qt::white);
  if (!image_.isNull()) {
    const QRect target = camera_image_rect(size(), image_.size());
    painter.drawImage(target, image_);
    painter.save(); painter.setClipRect(target);
    std::vector<QRect> occupied;
    int hidden = 0;
    for (const auto & box : overlay_.boxes) {
      const QRectF bounds(target.x() + box.normalized.x() * target.width(),
        target.y() + box.normalized.y() * target.height(), box.normalized.width() * target.width(),
        box.normalized.height() * target.height());
      const QColor color(box.tracking ? "#a65e00" : "#1769df");
      painter.setPen(QPen(color, 2)); painter.drawRect(bounds);
      const auto text = painter.fontMetrics().elidedText(
        box.label, Qt::ElideRight,
        std::max(0, std::min(220, target.width() / 2) - 8));
      const int label_width = std::min(
        target.width(), painter.fontMetrics().horizontalAdvance(
          text) + 8);
      const auto label = camera_label_rect(
        bounds.toAlignedRect(), target,
        QSize(label_width, 24), occupied);
      if (label.isEmpty()) {++hidden; continue;}
      occupied.push_back(label);
      painter.drawLine(QPointF(label.center().x(), label.bottom()), bounds.topLeft());
      painter.fillRect(label, color); painter.setPen(Qt::white);
      painter.drawText(label.adjusted(4, 0, -4, 0), Qt::AlignVCenter | Qt::AlignLeft, text);
    }
    setProperty("hidden_overlay_labels", hidden);
    painter.restore();
  } else {
    painter.setPen(QColor("#edf1f6"));
    for (int x = 0; x < width(); x += 40) {
      painter.drawLine(x, 0, x, height());
    }
    for (int y = 0; y < height(); y += 40) {
      painter.drawLine(0, y, width(), y);
    }
    painter.setPen(QColor("#526176"));
    painter.drawText(
      rect().adjusted(12, 12, -12, -48), Qt::AlignCenter,
      QStringLiteral("没有当前有效图像\n不会替换为另一相机或演示画面"));
  }
  painter.fillRect(QRect(0, height() - 48, width(), 48), QColor(255, 255, 255, 240));
  painter.setPen(QColor("#344256"));
  painter.drawText(
    rect().adjusted(
      12,
      height() - 44, -12, -4), Qt::AlignLeft | Qt::TextWordWrap, caption_);
}
MapCanvas::MapCanvas(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(300); setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}
void MapCanvas::snapshot(std::shared_ptr<const bridge::UiSnapshot> snapshot, double now)
{
  snapshot_ = std::move(snapshot); now_ = now;
  const auto instruments = snapshot_ ? flight_instruments(*snapshot_, now_) : FlightInstruments{};
  setProperty("position_valid", instruments.position_valid);
  setProperty("heading_valid", instruments.position_valid && instruments.attitude_valid);
  setProperty(
    "target_count", snapshot_ && geometry_current(*snapshot_, bridge::Key::Targets, now_) ?
    static_cast<int>(snapshot_->perception.at(
      bridge::Key::Targets).value->targets.targets.size()) : 0);
  setProperty(
    "obstacle_count", snapshot_ && geometry_current(*snapshot_, bridge::Key::Obstacles, now_) ?
    static_cast<int>(snapshot_->perception.at(bridge::Key::Obstacles).value->obstacles.obstacles.
    size()) : 0);
  update();
}
void MapCanvas::paintEvent(QPaintEvent * event)
{
  static_cast<void>(event);
  QPainter p(this); p.setRenderHint(QPainter::Antialiasing); p.fillRect(rect(), QColor("#ffffff"));
  double radius = 5;
  const auto include = [&radius](double x, double y) {
      if (std::isfinite(x) && std::isfinite(y) && std::abs(x) < 1e6 && std::abs(y) < 1e6) {
        radius = std::max(radius, std::max(std::abs(x), std::abs(y)) * 1.15);
      }
    };
  const bool clock_ok = snapshot_ &&
    snapshot_->clock.quality == bridge::Quality::ValidAtObservation;
  if (snapshot_ && clock_ok) {
    for (const auto & point : snapshot_->trajectory) {
      if (now_ - point.steady <= 300) {include(point.x, point.y);}
    }
    if (flight_instruments(*snapshot_, now_).position_valid) {
      include(snapshot_->state.sdk.value.position_x, snapshot_->state.sdk.value.position_y);
    }
    if (geometry_current(*snapshot_, bridge::Key::Targets, now_)) {
      for (const auto & t : snapshot_->perception.at(bridge::Key::Targets).value->targets.targets) {
        include(t.position_x, t.position_y);
      }
    }
    if (geometry_current(*snapshot_, bridge::Key::Obstacles, now_)) {
      for (const auto & o :
        snapshot_->perception.at(bridge::Key::Obstacles).value->obstacles.obstacles)
      {
        include(o.center_x, o.center_y);
      }
    }
  }
  const double scale = std::min(width(), height()) / (radius * 2.4);
  const auto pixel = [&](double x, double y) {
      return QPointF(width() / 2.0 + x * scale, height() / 2.0 - y * scale);
    };
  p.setPen(QColor("#e5eaf0"));
  for (int i = -5; i <= 5; ++i) {
    const double coordinate = i * radius / 5;
    p.drawLine(pixel(coordinate, -radius), pixel(coordinate, radius));
    p.drawLine(pixel(-radius, coordinate), pixel(radius, coordinate));
  }
  p.setPen(QColor("#94a3b8")); p.drawLine(pixel(-radius, 0), pixel(radius, 0));
  p.drawLine(pixel(0, -radius), pixel(0, radius));
  if (snapshot_ && clock_ok) {
    QPainterPath path; bool started = false;
    for (const auto & point : snapshot_->trajectory) {
      if (now_ - point.steady > 300) {started = false; continue;}
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || std::abs(point.x) > 1e6 ||
        std::abs(point.y) > 1e6) {started = false; continue;}
      if (!started) {path.moveTo(pixel(point.x, point.y)); started = true;} else {
        path.lineTo(pixel(point.x, point.y));
      }
    }
    p.setPen(QPen(QColor("#64748b"), 2)); p.drawPath(path);
    if (geometry_current(*snapshot_, bridge::Key::Obstacles, now_)) {
      p.setPen(QColor("#b66a0c")); p.setBrush(QColor(245, 158, 11, 45));
      for (const auto & o :
        snapshot_->perception.at(bridge::Key::Obstacles).value->obstacles.obstacles)
      {
        if (std::abs(o.center_x) > 1e6 || std::abs(o.center_y) > 1e6 || o.size_x > 1e6 ||
          o.size_y > 1e6) {continue;}
        const auto a = pixel(o.center_x - o.size_x / 2, o.center_y + o.size_y / 2);
        const auto b = pixel(o.center_x + o.size_x / 2, o.center_y - o.size_y / 2);
        p.drawRect(QRectF(a, b).normalized());
      }
    }
    if (geometry_current(*snapshot_, bridge::Key::Targets, now_)) {
      p.setPen(QColor("#7c3aed")); p.setBrush(QColor("#8b5cf6"));
      for (const auto & t : snapshot_->perception.at(bridge::Key::Targets).value->targets.targets) {
        if (std::abs(t.position_x) > 1e6 || std::abs(t.position_y) > 1e6) {continue;}
        p.drawEllipse(pixel(t.position_x, t.position_y), 5, 5);
      }
    }
    const auto & state = snapshot_->state;
    const auto instruments = flight_instruments(*snapshot_, now_);
    const bool position = instruments.position_valid;
    if (position && std::isfinite(state.sdk.value.position_x) &&
      std::isfinite(state.sdk.value.position_y) &&
      std::abs(state.sdk.value.position_x) < 1e6 && std::abs(state.sdk.value.position_y) < 1e6)
    {
      p.setPen(QPen(QColor("#1769df"), 2)); p.setBrush(QColor("#1769df"));
      const auto center = pixel(state.sdk.value.position_x, state.sdk.value.position_y);
      if (instruments.attitude_valid) {
        p.save(); p.translate(center); p.rotate(instruments.heading_degrees);
        p.drawPolygon(
          QPolygonF(
            QVector<QPointF>{QPointF(0, -14), QPointF(9, 10),
              QPointF(0, 6), QPointF(-9, 10)}));
        p.restore();
      } else {p.drawEllipse(center, 7, 7);}
    }
  }
  p.setPen(QColor("#475569"));
  p.drawText(
    QRect(12, 10, width() - 24, 50), Qt::TextWordWrap,
    QStringLiteral(
      "odom / ENU · X → 东，Y ↑ "
      "北 · ±%1 m\n灰：历史实际"
      "轨迹 · 蓝：当前机位 · "
      "紫：实测目标 · 黄：几何障"
      "碍").arg(
      number(radius, 1)));
  if (!clock_ok) {
    p.drawText(
      rect(), Qt::AlignCenter, QStringLiteral(
        "时间域不可用：不显示当前"
        "几何图层"));
  } else if (!property("position_valid").toBool()) {
    p.drawText(
      QRect(12, height() - 30, width() - 24, 24), Qt::AlignLeft,
      QStringLiteral("当前定位不可用 · 保留的灰线仅为历史轨迹"));
  }
}
}  // namespace easyfly_desktop
