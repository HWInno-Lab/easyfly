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

#include <algorithm>
#include <cmath>
#include <utility>

#include "easyfly_desktop/presentation.hpp"
#include "widgets.hpp"
namespace easyfly_desktop
{
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
  setProperty("has_frame", !image_.isNull()); update();
}
void CameraCanvas::paintEvent(QPaintEvent * event)
{
  static_cast<void>(event);
  QPainter painter(this); painter.fillRect(rect(), QColor("#101b2a"));
  if (!image_.isNull()) {
    const QSize size = image_.size().scaled(this->size(), Qt::KeepAspectRatio);
    const QRect target((width() - size.width()) / 2, (height() - size.height()) / 2, size.width(),
      size.height());
    painter.drawImage(target, image_);
  } else {
    painter.setPen(QColor("#40516a"));
    for (int x = 0; x < width(); x += 40) {
      painter.drawLine(x, 0, x, height());
    }
    for (int y = 0; y < height(); y += 40) {
      painter.drawLine(0, y, width(), y);
    }
    painter.setPen(QColor("#a9bdd2"));
    painter.drawText(
      rect().adjusted(12, 12, -12, -48), Qt::AlignCenter,
      QStringLiteral("没有当前有效图像\n不会替换为另一相机或演示画面"));
  }
  painter.fillRect(QRect(0, height() - 48, width(), 48), QColor(10, 18, 29, 220));
  painter.setPen(QColor("#d5e2ef"));
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
    if (snapshot_->state.position_valid &&
      display_quality(
        snapshot_->state.evidence, *snapshot_, now_, 1,
        true) == bridge::Quality::ValidAtObservation)
    {
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
    const bool position = state.position_valid &&
      display_quality(
      state.evidence, *snapshot_, now_, 1,
      true) == bridge::Quality::ValidAtObservation;
    if (position && std::isfinite(state.sdk.value.position_x) &&
      std::isfinite(state.sdk.value.position_y) &&
      std::abs(state.sdk.value.position_x) < 1e6 && std::abs(state.sdk.value.position_y) < 1e6)
    {
      p.setPen(QPen(QColor("#1769df"), 2)); p.setBrush(QColor("#1769df"));
      p.drawEllipse(pixel(state.sdk.value.position_x, state.sdk.value.position_y), 7, 7);
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
  }
}
}  // namespace easyfly_desktop
