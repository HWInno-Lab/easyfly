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
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainterPath>
#include <QPolygonF>
#include <QEvent>
#include <QLabel>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include "widgets.hpp"
#include "easyfly_desktop/presentation.hpp"
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
void FlightInstrumentCanvas::set_compact(bool compact)
{
  compact_ = compact; setMinimumSize(compact ? QSize(196, 108) : QSize(220, 190)); update();
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
  if (compact_) {auto font = p.font(); font.setPixelSize(12); p.setFont(font);}
  const double radius = std::max(
    compact_ ? 18.0 : 30.0,
    std::min(width() / 4.0 - 14, (height() - (compact_ ? 60 : 68)) / 2.0));
  const int top = compact_ ? 20 : 29;
  const QPointF attitude(width() * .25, radius + top), compass(width() * .75, radius + top);
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
  const double offset = radius - (compact_ ? 9 : 20);
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
    const int y = static_cast<int>(radius * 2 + (compact_ ? 25 : 36));
    p.drawText(
      QRect(0, y, width() / 2, compact_ ? 30 : 20), Qt::AlignCenter,
      (compact_ ? QStringLiteral("R %1°\nP %2°") : QStringLiteral("R %1° / P %2°")).arg(
        number(instruments_.roll_degrees, 1),
        number(instruments_.pitch_degrees, 1)));
    p.drawText(
      QRect(width() / 2, y, width() / 2, compact_ ? 30 : 20), Qt::AlignCenter,
      number(instruments_.heading_degrees, 1) + QStringLiteral("°"));
  } else {
    for (const auto & center : {attitude, compass}) {
      p.drawText(QRectF(center.x() - 30, center.y() - 10, 60, 20), Qt::AlignCenter, "—");
    }
    p.drawText(
      QRect(0, static_cast<int>(radius * 2 + (compact_ ? 25 : 36)), width(), 22), Qt::AlignCenter,
      QStringLiteral("姿态 / 航向不可用"));
  }
}
FlightInstrumentOverlay::FlightInstrumentOverlay(QWidget * parent)
: QWidget(parent)
{
  setObjectName("overviewInstrumentOverlay"); setAttribute(Qt::WA_StyledBackground);
  setStyleSheet(
    "QWidget#overviewInstrumentOverlay {background: #ffffff; border: 1px solid #cdd8e6; "
    "border-radius: 6px;}");
  setAccessibleName(QStringLiteral("地图内姿态与局部航向，只读观测"));
  auto * layout = new QVBoxLayout(this); layout->setContentsMargins(6, 4, 6, 4);
  layout->setSpacing(2); auto * header = new QHBoxLayout;
  header->setContentsMargins(0, 0, 0, 0); header->setSpacing(4);
  auto * title = new QLabel(QStringLiteral("姿态与局部航向"), this);
  title->setStyleSheet("font-size: 12px; color: #344256;");
  header->addWidget(title, 1);
  toggle_ = new QToolButton(this); toggle_->setObjectName("overviewInstrumentsToggle");
  toggle_->setCheckable(true); toggle_->setChecked(true); toggle_->setFixedSize(40, 24);
  toggle_->setFocusPolicy(Qt::StrongFocus); header->addWidget(toggle_); layout->addLayout(header);
  canvas_ = new FlightInstrumentCanvas(this); canvas_->setObjectName("overviewInstruments");
  canvas_->set_compact(true); layout->addWidget(canvas_, 1);
  QObject::connect(
    toggle_, &QToolButton::clicked, this, [this](bool expanded) {
      requested_expanded_ = expanded; user_toggled_ = true; position_overlay();
    });
  attach(parent);
}
FlightInstrumentCanvas * FlightInstrumentOverlay::canvas() const {return canvas_;}
void FlightInstrumentOverlay::attach(QWidget * canvas)
{
  if (!canvas) {return;}
  if (parentWidget() != canvas) {
    if (parentWidget()) {parentWidget()->removeEventFilter(this);}
    setParent(canvas);
  }
  canvas->installEventFilter(this); position_overlay(); show(); raise();
}
bool FlightInstrumentOverlay::eventFilter(QObject * watched, QEvent * event)
{
  if (watched == parentWidget() &&
    (event->type() == QEvent::Resize || event->type() == QEvent::Show))
  {
    position_overlay();
  }
  return QWidget::eventFilter(watched, event);
}
void FlightInstrumentOverlay::position_overlay()
{
  auto * host = parentWidget(); if (!host) {return;}
  const bool narrow = host->width() < 480 || host->height() < 260;
  const bool expanded = requested_expanded_ && (user_toggled_ || !narrow);
  canvas_->setVisible(expanded);
  const QSignalBlocker blocker(toggle_); toggle_->setChecked(expanded);
  toggle_->setText(expanded ? QStringLiteral("收起") : QStringLiteral("展开"));
  toggle_->setAccessibleName(
    expanded ? QStringLiteral("收起姿态与局部航向") : QStringLiteral("展开姿态与局部航向"));
  toggle_->setToolTip(toggle_->accessibleName());
  setProperty("expanded", expanded); setProperty("automatic_compact", narrow && !user_toggled_);
  setFixedSize(224, expanded ? 148 : 34);
  move(std::max(0, host->width() - width() - 10), std::max(0, host->height() - height() - 10));
  raise();
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
  set_frozen(false);
  setToolTip(caption); setAccessibleDescription(caption);
  if (image_.isNull()) {overlay({});}
  setProperty("has_frame", !image_.isNull()); update();
}
void CameraCanvas::set_frozen(bool frozen)
{
  frozen_ = frozen && !image_.isNull(); setProperty("frame_frozen", frozen_);
  const auto description = frozen_ ? QStringLiteral("最近画面（已过期） · ") + caption_ : caption_;
  setAccessibleDescription(description); setToolTip(description); update();
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
  const int caption_required = painter.boundingRect(
    QRect(12, 0, std::max(1, width() - 24), 10000),
    Qt::AlignLeft | Qt::TextWordWrap, caption_).height() + 8;
  const int caption_height = std::min(height(), std::max(48, caption_required));
  setProperty("caption_height", caption_height);
  setProperty("caption_required_height", caption_required);
  if (!image_.isNull()) {
    const QRect target = camera_image_rect(size(), image_.size(), caption_height);
    painter.drawImage(target, image_);
    painter.save(); painter.setClipRect(target);
    std::vector<QRect> occupied;
    int hidden = 0;
    for (const auto & box : overlay_.boxes) {
      const QRectF bounds(target.x() + box.normalized.x() * target.width(),
        target.y() + box.normalized.y() * target.height(), box.normalized.width() * target.width(),
        box.normalized.height() * target.height());
      const QColor color(box.tracking ? "#a65e00" : "#1769df");
      painter.setPen(QPen(color, 2));
      if (box.shape == CameraOverlayMode::Ellipses) {
        painter.save(); painter.translate(bounds.center()); painter.rotate(box.angle_degrees);
        painter.drawEllipse(
          QRectF(
            -bounds.width() / 2, -bounds.height() / 2, bounds.width(),
            bounds.height()));
        painter.restore();
      } else if (box.shape == CameraOverlayMode::ColorLines) {
        const auto pixel = [&](const QPointF & point) {
            return QPointF(
              target.x() + point.x() * target.width(),
              target.y() + point.y() * target.height());
          };
        painter.drawLine(QLineF(pixel(box.segment.p1()), pixel(box.segment.p2())));
      } else {painter.drawRect(bounds);}
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
    if (frozen_) {
      const QString message = QStringLiteral("画面暂停 · 已过期");
      painter.save(); auto font = painter.font(); font.setPixelSize(12); painter.setFont(font);
      const int banner_height = painter.boundingRect(
        QRect(0, 0, std::max(1, width() - 32), 1000), Qt::TextWordWrap, message).height() + 12;
      const QRect banner(8, 8, std::max(1, width() - 16), banner_height);
      painter.fillRect(banner, QColor("#fff0c2")); painter.setPen(QColor("#805000"));
      painter.drawText(banner.adjusted(8, 4, -8, -4), Qt::AlignCenter | Qt::TextWordWrap, message);
      painter.restore();
    }
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
      rect().adjusted(12, 12, -12, -caption_height), Qt::AlignCenter,
      QStringLiteral("没有当前有效图像\n不会替换为另一相机或演示画面"));
  }
  painter.fillRect(
    QRect(0, height() - caption_height, width(), caption_height), QColor(255, 255, 255, 240));
  painter.setPen(QColor("#344256"));
  painter.drawText(
    rect().adjusted(
      12,
      height() - caption_height + 4, -12, -4), Qt::AlignLeft | Qt::TextWordWrap, caption_);
}
MapCanvas::MapCanvas(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(300); setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setAccessibleName(QStringLiteral("局部地图，只读观测"));
  const auto description = QStringLiteral(
    "坐标系 odom / ENU：X 向东，Y 向北，单位米。\n"
    "栅格深色为占据，浅灰为未知，留白为未标占据；留白不表示已探索或可安全通行。\n"
    "灰线为实际飞行轨迹，蓝色为飞机，紫色为当前感知目标，黄色框为可选实时障碍叠层。\n"
    "绑定任务后，虚线表示任务顺序，绿色表示已完成，橙色表示当前目标。");
  setToolTip(description); setAccessibleDescription(description);
  setProperty("grid_enabled", grid_enabled_); setProperty("obstacle_overlay", obstacle_overlay_);
  animation_timer_ = new QTimer(this); animation_timer_->setInterval(16);
  connect(
    animation_timer_, &QTimer::timeout, this, [this] {
      const double now = now_ + std::max(
        0., bridge::detail::steady_seconds() -
        snapshot_received_steady_);
      if (!snapshot_ || !geometry_current(*snapshot_, bridge::Key::Obstacles, now)) {
        obstacles_.clear();
      }
      if (!obstacles_.active(now)) {animation_timer_->stop();}
      update();
    });
}
void MapCanvas::snapshot(std::shared_ptr<const bridge::UiSnapshot> snapshot, double now)
{
  if (!snapshot || snapshot->observation_session_id != fit_session_ ||
    snapshot->time_epoch != fit_epoch_)
  {
    fitted_radius_ = 5;
    obstacles_.clear(); grid_source_.reset(); grid_image_ = {};
    fit_session_ = snapshot ? snapshot->observation_session_id : 0;
    fit_epoch_ = snapshot ? snapshot->time_epoch : 0;
  }
  snapshot_ = std::move(snapshot); now_ = now;
  snapshot_received_steady_ = bridge::detail::steady_seconds();
  update_local_layers();
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
void MapCanvas::set_grid_enabled(bool enabled)
{
  grid_enabled_ = enabled; setProperty("grid_enabled", enabled);
  update_local_layers(); update();
}
void MapCanvas::set_obstacle_overlay(bool enabled)
{
  if (obstacle_overlay_ != enabled) {obstacles_.clear(); animation_timer_->stop();}
  obstacle_overlay_ = enabled; setProperty("obstacle_overlay", enabled);
  update_local_layers(); update();
}
void MapCanvas::update_local_layers()
{
  bool grid_valid = false;
  if (!grid_enabled_) {grid_status_ = QStringLiteral("栅格图层已关闭");} else if (!snapshot_) {
    grid_status_ = QStringLiteral("等待局部栅格地图");
  } else {
    const auto & observation = snapshot_->occupancy_grid;
    auto quality = display_quality(observation.evidence, *snapshot_, now_, 2., true);
    if (quality == bridge::Quality::ValidAtObservation && observation.value) {
      const double age = observation.evidence.receipt_age + snapshot_delay(*snapshot_, now_);
      quality = observation.value == grid_source_ ?
        bridge::detail::source_quality(observation.value->header, snapshot_->clock, age, 2.) :
        bridge::detail::validate_occupancy_grid(*observation.value, snapshot_->clock, age);
    }
    grid_valid = quality == bridge::Quality::ValidAtObservation && observation.value;
    grid_status_ = grid_valid ? QStringLiteral("局部栅格地图 · 当前") :
      (quality == bridge::Quality::NotReceived ? QStringLiteral("等待局部栅格地图") :
      QStringLiteral("局部栅格地图不可用：%1").arg(quality_text(quality)));
    if (grid_valid && grid_source_ != observation.value) {
      grid_source_ = observation.value;
      setProperty("grid_image_builds", property("grid_image_builds").toInt() + 1);
      const auto & grid = *grid_source_;
      grid_image_ = QImage(grid.info.width, grid.info.height, QImage::Format_ARGB32);
      grid_image_.fill(Qt::transparent);
      int min_x = grid.info.width, min_y = grid.info.height, max_x = -1, max_y = -1;
      int occupied = 0;
      for (uint32_t y = 0; y < grid.info.height; ++y) {
        auto * row = reinterpret_cast<QRgb *>(grid_image_.scanLine(y));
        for (uint32_t x = 0; x < grid.info.width; ++x) {
          const auto value = grid.data[y * grid.info.width + x];
          if (value >= 50) {
            row[x] = qRgba(51, 65, 85, 225); ++occupied;
            min_x = std::min(min_x, static_cast<int>(x));
            min_y = std::min(min_y, static_cast<int>(y));
            max_x = std::max(max_x, static_cast<int>(x));
            max_y = std::max(max_y, static_cast<int>(y));
          } else if (value < 0) {row[x] = qRgba(148, 163, 184, 90);}
        }
      }
      grid_occupied_bounds_ = occupied ?
        QRect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1) : QRect{};
      setProperty("occupied_cells", occupied);
    }
  }
  if (!grid_valid) {
    grid_source_.reset(); grid_image_ = {}; grid_occupied_bounds_ = {};
    setProperty("occupied_cells", 0);
  }
  setProperty("grid_valid", grid_valid); setProperty("grid_status", grid_status_);
  if (!obstacle_overlay_ || !snapshot_ ||
    !geometry_current(*snapshot_, bridge::Key::Obstacles, now_))
  {
    obstacles_.clear(); animation_timer_->stop(); return;
  }
  const auto & response = *snapshot_->perception.at(bridge::Key::Obstacles).value;
  std::vector<DisplayObstacle> boxes; boxes.reserve(response.obstacles.obstacles.size());
  for (const auto & o : response.obstacles.obstacles) {
    if (std::abs(o.center_x) > 1e6 || std::abs(o.center_y) > 1e6 ||
      o.size_x > 1e6 || o.size_y > 1e6) {continue;}
    boxes.push_back(
      {o.center_x - o.size_x / 2., o.center_y - o.size_y / 2.,
        o.center_x + o.size_x / 2., o.center_y + o.size_y / 2., o.is_dynamic, o.obstacle_type});
  }
  const auto identity = std::to_string(snapshot_->observation_session_id) + ":" +
    std::to_string(snapshot_->time_epoch) + ":" + response.source_topic + ":" +
    response.obstacles.header.frame_id;
  obstacles_.accept(
    std::move(boxes), identity, bridge::detail::stamp_ns(response.observed_at), now_);
  if (obstacles_.active(now_)) {animation_timer_->start();}
}
void MapCanvas::set_task_overlay(const OverviewTaskOverlay & task)
{
  task_ = task; setProperty("task_bound", task.bound); setProperty("task_fresh", task.fresh);
  setProperty("task_points", static_cast<int>(task.geometry.points.size())); update();
}
void MapCanvas::zoom(double factor)
{
  if (!std::isfinite(factor) || factor <= 0) {return;}
  if (!user_view_) {zoom_ = 5 / std::max(.001, last_radius_);}
  user_view_ = true; zoom_ = std::clamp(zoom_ * factor, .00001, 100.); update();
}
void MapCanvas::fit() {user_view_ = false; zoom_ = 1; fitted_radius_ = 5; update();}
void MapCanvas::center_on_vehicle() {follow_ = true; update();}
void MapCanvas::set_follow(bool follow) {follow_ = follow; update();}
bool MapCanvas::following() const {return follow_;}
void MapCanvas::wheelEvent(QWheelEvent * event)
{zoom(event->angleDelta().y() > 0 ? 1.25 : .8); event->accept();}
void MapCanvas::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    if (!user_view_) {zoom_ = 5 / std::max(.001, last_radius_); user_view_ = true;}
    panning_ = true; last_mouse_ = event->pos(); setCursor(Qt::ClosedHandCursor);
  }
}
void MapCanvas::mouseMoveEvent(QMouseEvent * event)
{
  if (!panning_ || !std::isfinite(last_scale_) || last_scale_ <= 0) {return;}
  const auto delta = event->pos() - last_mouse_; last_mouse_ = event->pos();
  center_x_ -= delta.x() / last_scale_; center_y_ += delta.y() / last_scale_;
  follow_ = false; user_view_ = true; update();
}
void MapCanvas::mouseReleaseEvent(QMouseEvent *) {panning_ = false; unsetCursor();}
void MapCanvas::paintEvent(QPaintEvent * event)
{
  static_cast<void>(event);
  const double render_now = now_ + std::max(
    0., bridge::detail::steady_seconds() -
    snapshot_received_steady_);
  QPainter p(this); p.setRenderHint(QPainter::Antialiasing); p.fillRect(rect(), QColor("#ffffff"));
  if (follow_ && snapshot_ && flight_instruments(*snapshot_, render_now).position_valid) {
    center_x_ = snapshot_->state.sdk.value.position_x;
    center_y_ = snapshot_->state.sdk.value.position_y;
  }
  double radius = 5;
  const auto include = [this, &radius](double x, double y) {
      if (std::isfinite(x) && std::isfinite(y) && std::abs(x) < 1e6 && std::abs(y) < 1e6) {
        radius =
          std::max(radius, std::max(std::abs(x - center_x_), std::abs(y - center_y_)) * 1.15);
      }
    };
  const bool clock_ok = snapshot_ &&
    snapshot_->clock.quality == bridge::Quality::ValidAtObservation;
  bool grid_visible = grid_enabled_ && grid_source_ && clock_ok;
  if (grid_visible) {
    const auto & observation = snapshot_->occupancy_grid;
    const auto quality = display_quality(observation.evidence, *snapshot_, render_now, 2., true);
    grid_visible = quality == bridge::Quality::ValidAtObservation &&
      bridge::detail::source_quality(
      grid_source_->header, snapshot_->clock,
      observation.evidence.receipt_age + snapshot_delay(*snapshot_, render_now), 2.) ==
      bridge::Quality::ValidAtObservation;
    if (!grid_visible) {
      grid_source_.reset(); grid_image_ = {}; grid_occupied_bounds_ = {};
      setProperty("grid_valid", false); setProperty("occupied_cells", 0);
      grid_status_ = QStringLiteral("局部栅格地图不可用：数据已过期");
      setProperty("grid_status", grid_status_);
    }
  }
  const auto grid_world = [this](double x, double y) {
      const auto & origin = grid_source_->info.origin;
      const auto & q = origin.orientation;
      const double yaw = std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z);
      const double resolution = grid_source_->info.resolution;
      return QPointF(
        origin.position.x + resolution * (x * std::cos(yaw) - y * std::sin(yaw)),
        origin.position.y + resolution * (x * std::sin(yaw) + y * std::cos(yaw)));
    };
  if (grid_visible && !grid_occupied_bounds_.isEmpty()) {
    const auto & b = grid_occupied_bounds_;
    for (const auto & point : {grid_world(b.x(), b.y()),
        grid_world(b.x() + b.width(), b.y()),
        grid_world(b.x(), b.y() + b.height()),
        grid_world(b.x() + b.width(), b.y() + b.height())})
    {
      include(point.x(), point.y());
    }
  }
  if (snapshot_ && clock_ok) {
    for (const auto & point : snapshot_->trajectory) {
      if (render_now - point.steady <= 300) {include(point.x, point.y);}
    }
    if (flight_instruments(*snapshot_, render_now).position_valid) {
      include(snapshot_->state.sdk.value.position_x, snapshot_->state.sdk.value.position_y);
    }
    if (geometry_current(*snapshot_, bridge::Key::Targets, render_now)) {
      for (const auto & t : snapshot_->perception.at(bridge::Key::Targets).value->targets.targets) {
        include(t.position_x, t.position_y);
      }
    }
    if (obstacle_overlay_ && geometry_current(*snapshot_, bridge::Key::Obstacles, render_now)) {
      for (const auto & o :
        snapshot_->perception.at(bridge::Key::Obstacles).value->obstacles.obstacles)
      {
        include(o.center_x, o.center_y);
      }
    }
  }
  const bool task_visible = task_.bound && task_.geometry.accepted && task_.frame == "odom";
  if (task_visible) {
    for (const auto & point : task_.geometry.points) {
      include(point.x, point.y);
    }
    for (const auto & area : task_.geometry.areas) {
      include(area.x, area.y); include(area.x + area.width, area.y + area.height);
    }
  }
  if (user_view_) {radius = 5;} else {
    // Expired optional geometry must not zoom the whole map in and out.
    fitted_radius_ = std::max(fitted_radius_, radius); radius = fitted_radius_;
  }
  radius /= zoom_; last_radius_ = radius; setProperty("mapRadius", radius);
  last_scale_ = std::min(width(), height()) / (radius * 2.4);
  const double scale = last_scale_;
  const auto pixel = [&](double x, double y) {
      return QPointF(
        width() / 2.0 + (x - center_x_) * scale,
        height() / 2.0 - (y - center_y_) * scale);
    };
  p.setPen(QColor("#e5eaf0"));
  for (int i = -5; i <= 5; ++i) {
    const double coordinate = i * radius / 5;
    p.drawLine(
      pixel(center_x_ + coordinate, center_y_ - radius),
      pixel(center_x_ + coordinate, center_y_ + radius));
    p.drawLine(
      pixel(center_x_ - radius, center_y_ + coordinate),
      pixel(center_x_ + radius, center_y_ + coordinate));
  }
  p.setPen(QColor("#94a3b8")); p.drawLine(pixel(-radius, 0), pixel(radius, 0));
  p.drawLine(pixel(0, -radius), pixel(0, radius));
  if (grid_visible) {
    const auto origin = pixel(grid_world(0, 0).x(), grid_world(0, 0).y());
    const auto x = pixel(grid_world(1, 0).x(), grid_world(1, 0).y()) - origin;
    const auto y = pixel(grid_world(0, 1).x(), grid_world(0, 1).y()) - origin;
    p.save(); p.setRenderHint(QPainter::SmoothPixmapTransform, false);
    p.setWorldTransform(QTransform(x.x(), x.y(), y.x(), y.y(), origin.x(), origin.y()));
    p.drawImage(QPointF(0, 0), grid_image_); p.restore();
  }
  if (task_visible && clock_ok) {
    const auto color = [this](size_t step) {
        if (!task_.fresh) {return QColor("#94a3b8");}
        if (step < static_cast<size_t>(task_.completed_steps)) {return QColor("#2e7d32");}
        return static_cast<int>(step) == task_.current_step ? QColor("#c96500") : QColor("#466a99");
      };
    p.setPen(QPen(QColor("#94a3b8"), 1.5, Qt::DashLine));
    for (const auto & link : task_.geometry.links) {
      const auto & a = task_.geometry.points[link.from];
      const auto & b = task_.geometry.points[link.to];
      p.drawLine(pixel(a.x, a.y), pixel(b.x, b.y));
    }
    for (const auto & area : task_.geometry.areas) {
      auto c = area.forbidden ? QColor("#b91c1c") : color(area.step);
      p.setPen(QPen(c, 1.5, Qt::DashLine)); c.setAlpha(24); p.setBrush(c);
      p.drawRect(
        QRectF(
          pixel(area.x, area.y),
          pixel(area.x + area.width, area.y + area.height)).normalized());
    }
    for (const auto & point : task_.geometry.points) {
      const auto at = pixel(point.x, point.y); p.setPen(QPen(color(point.step), 2));
      p.setBrush(
        point.step < static_cast<size_t>(task_.completed_steps) ?
        color(point.step) : QColor("#ffffff")); p.drawEllipse(at, 7, 7);
      p.drawText(at + QPointF(9, -8), QString::number(point.step + 1));
    }
    if (task_.fresh && task_.current_target) {
      const auto & target = *task_.current_target;
      p.setPen(QPen(QColor("#c96500"), 3)); p.setBrush(Qt::NoBrush);
      p.drawEllipse(pixel(target[0], target[1]), 11, 11);
    }
  }
  if (snapshot_ && clock_ok) {
    QPainterPath path; bool started = false; double previous_receipt = 0;
    for (const auto & point : snapshot_->trajectory) {
      if (previous_receipt > 0 && point.steady - previous_receipt > 2) {started = false;}
      previous_receipt = point.steady;
      if (render_now - point.steady > 300) {started = false; continue;}
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || std::abs(point.x) > 1e6 ||
        std::abs(point.y) > 1e6) {started = false; continue;}
      if (!started) {path.moveTo(pixel(point.x, point.y)); started = true;} else {
        path.lineTo(pixel(point.x, point.y));
      }
    }
    p.setPen(QPen(QColor("#64748b"), 2)); p.drawPath(path);
    if (obstacle_overlay_ && geometry_current(*snapshot_, bridge::Key::Obstacles, render_now)) {
      p.setPen(QColor("#b66a0c")); p.setBrush(QColor(245, 158, 11, 45));
      for (const auto & o : obstacles_.at(render_now)) {
        const auto a = pixel(o.left, o.top);
        const auto b = pixel(o.right, o.bottom);
        p.drawRect(QRectF(a, b).normalized());
      }
    } else {obstacles_.clear();}
    if (geometry_current(*snapshot_, bridge::Key::Targets, render_now)) {
      p.setPen(QColor("#7c3aed")); p.setBrush(QColor("#8b5cf6"));
      for (const auto & t : snapshot_->perception.at(bridge::Key::Targets).value->targets.targets) {
        if (std::abs(t.position_x) > 1e6 || std::abs(t.position_y) > 1e6) {continue;}
        p.drawEllipse(pixel(t.position_x, t.position_y), 5, 5);
      }
    }
    const auto & state = snapshot_->state;
    const auto instruments = flight_instruments(*snapshot_, render_now);
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
  auto legend = QStringLiteral("odom / ENU · ±%1 m").arg(number(radius, 1));
  if (grid_enabled_) {
    auto status = grid_status_;
    status.replace(QStringLiteral("局部栅格地图"), QStringLiteral("栅格"));
    status.replace(QStringLiteral(" · 当前"), QStringLiteral("：当前"));
    legend += QStringLiteral(" · %1\n深色：占据 · 浅灰：未知 · 留白：未标占据").arg(status);
  }
  legend += QStringLiteral("\n灰线：实际轨迹 · 蓝：飞机");
  if (property("target_count").toInt() > 0 && snapshot_ &&
    geometry_current(*snapshot_, bridge::Key::Targets, render_now))
  {legend += QStringLiteral(" · 紫：感知目标");}
  if (obstacle_overlay_) {legend += QStringLiteral(" · 黄框：实时障碍");}
  if (task_visible) {legend += QStringLiteral("\n虚线：任务顺序 · 绿：已完成 · 橙：当前目标");}
  const auto legend_box = p.boundingRect(
    QRect(12, 10, std::max(1, width() - 24), 200),
    Qt::TextWordWrap, legend);
  p.fillRect(legend_box.adjusted(-3, -2, 3, 2), QColor(255, 255, 255, 230));
  p.drawText(legend_box, Qt::TextWordWrap, legend);
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
