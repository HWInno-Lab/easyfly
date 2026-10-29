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
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <set>
#include <utility>
#include "easyfly_desktop/peripheral_panel.hpp"
#include "easyfly_desktop/presentation.hpp"
#include "easyfly_desktop/camera_overlay.hpp"
namespace easyfly_desktop
{
namespace control = easyfly_ui_control;
RoiCanvas::RoiCanvas(QWidget * parent)
: QWidget(parent)
{setMinimumSize(280, 220); setMouseTracking(true); setObjectName("singleRoiCanvas");}
void RoiCanvas::frame(std::shared_ptr<const ConvertedFrame> frame)
{
  if (!frame || !frame_ || !(frame->binding == frame_->binding) ||
    frame->image.size() != frame_->image.size())
  {
    roi_ = {};
    dragging_ = false;
    painted_.reset();
  }
  frame_ = std::move(frame); tracked_ = {}; update();
}
std::shared_ptr<const ConvertedFrame> RoiCanvas::displayed() const {return painted_;}
QRectF RoiCanvas::selection() const {return roi_;}
void RoiCanvas::tracked(QRectF bounds) {tracked_ = bounds; update();}
std::optional<QPointF> RoiCanvas::normalized(const QPoint & p) const
{
  if (!frame_ || frame_->image.isNull()) {return {};}
  const auto rect = camera_image_rect(size(), frame_->image.size());
  if (!rect.contains(p) || rect.width() <= 0 || rect.height() <= 0) {return {};}
  return QPointF(
    static_cast<double>(p.x() - rect.x()) / rect.width(),
    static_cast<double>(p.y() - rect.y()) / rect.height());
}
void RoiCanvas::mousePressEvent(QMouseEvent * event)
{
  const auto point = normalized(event->pos());
  if (event->button() != Qt::LeftButton || !point) {return;}
  begin_ = *point; roi_ = {}; dragging_ = true; update();
}
void RoiCanvas::mouseMoveEvent(QMouseEvent * event)
{
  const auto point = normalized(event->pos());
  if (dragging_ && point) {roi_ = QRectF(begin_, *point).normalized(); update();}
}
void RoiCanvas::mouseReleaseEvent(QMouseEvent * event)
{if (event->button() == Qt::LeftButton) {mouseMoveEvent(event); dragging_ = false;}}
void RoiCanvas::paintEvent(QPaintEvent * event)
{
  static_cast<void>(event);
  QPainter p(this); p.fillRect(rect(), Qt::white);
  if (!frame_ || frame_->image.isNull()) {
    p.setPen(QColor("#526176"));
    p.drawText(
      rect(),
      Qt::AlignCenter, QStringLiteral("等待当前前视原图"));
    return;
  }
  const auto target = camera_image_rect(size(), frame_->image.size()); p.drawImage(
    target,
    frame_->image);
  painted_ = frame_;
  const auto box = [&](const QRectF & value, const QColor & color, Qt::PenStyle style) {
      if (value.isEmpty()) {return;} p.setPen(QPen(color, 2, style));
      p.drawRect(
        QRectF(
          target.x() + value.x() * target.width(), target.y() + value.y() * target.height(),
          value.width() * target.width(), value.height() * target.height()));
    };
  p.setClipRect(target); box(roi_, QColor("#1769df"), Qt::DashLine);
  box(tracked_, QColor("#a65e00"), Qt::SolidLine); p.setClipping(false);
  p.setPen(QColor("#344256"));
  p.drawText(
    rect().adjusted(8, height() - 42, -8, -4), Qt::TextWordWrap,
    QStringLiteral(
      "蓝色虚线=人工 ROI；橙框=同帧 CSRT · "
      "点击初始化绑定此显示帧\nstamp=%1").arg(frame_->stamp_ns));
}
namespace
{
class RealPeripheral : public PeripheralPort
{
  control::PeripheralControl control_;

public:
  explicit RealPeripheral(const control::PeripheralConfig & c)
  : control_(c) {}
  std::shared_ptr<const control::PeripheralSnapshot> snapshot() const override
  {
    return control_.snapshot();
  }
  bool single(const easyfly_msgs::srv::ManageSingleTrack::Request & r) override
  {
    return control_.single_confirmed(r);
  }
  bool angle(const easyfly_msgs::srv::SetGimbalAngle::Request & r) override
  {
    return control_.angle_confirmed(r);
  }
  bool tracking(const easyfly_msgs::srv::SetGimbalTracking::Request & r) override
  {
    return control_.tracking_confirmed(r);
  }
  void inhibit(bool value) override {control_.inhibit(value);}
  void stop() override {control_.stop();}
};
QPushButton * button(const char * name, const QString & text, QWidget * parent)
{auto * b = new QPushButton(text, parent); b->setObjectName(name); return b;}
}  // namespace
PeripheralFactory peripheral_factory()
{return [](const auto & config) {return std::make_unique<RealPeripheral>(config);};}
struct PeripheralPanel::Impl
{
  PeripheralPanel * owner;
  bool allowed, available{false}, simulation{true}, bound_simulation{true}, closing{false};
  PeripheralFactory factory;
  std::unique_ptr<PeripheralPort> port;
  std::shared_ptr<const bridge::UiSnapshot> source;
  std::shared_ptr<const control::PeripheralSnapshot> state;
  RoiCanvas * canvas;
  QPushButton * open, * initialize, * reset, * query, * angle, * track, * disable;
  QCheckBox * permit;
  QLineEdit * category;
  QComboBox * target;
  QDoubleSpinBox * pitch, * yaw, * roll;
  QLabel * status, * single_status, * gimbal_status;
  bool target_fresh{false};
  uint64_t session{0}, epoch{0};
  Impl(PeripheralPanel * parent, bool enabled, PeripheralFactory f)
  : owner(parent), allowed(enabled), factory(std::move(f))
  {
    auto * layout = new QVBoxLayout(owner); auto * top = new QHBoxLayout;
    open = button("openPeripherals", QStringLiteral("连接专项观测"), owner);
    permit = new QCheckBox(QStringLiteral("允许本次单目标／仿真云台操作"), owner);
    permit->setObjectName("peripheralPermit"); permit->setEnabled(allowed);
    top->addWidget(open); top->addWidget(permit); top->addStretch(); layout->addLayout(top);
    status = new QLabel(
      QStringLiteral(
        "尚未连接；不会启动后端或发送命令"
        "。"),
      owner);
    status->setObjectName("peripheralOperationStatus"); status->setTextFormat(Qt::PlainText);
    status->setWordWrap(true); layout->addWidget(status);
    auto * tabs = new QTabWidget(owner);
    tabs->setObjectName("peripheralTabs");
    layout->addWidget(
      tabs);
    auto * single = new QWidget(tabs); auto * row = new QHBoxLayout(single);
    canvas = new RoiCanvas(single); row->addWidget(canvas, 1);
    auto * controls = new QWidget(single); auto * right = new QVBoxLayout(controls);
    category = new QLineEdit(controls); category->setObjectName("singleClassName");
    category->setMaxLength(128);
    category->setPlaceholderText(QStringLiteral("人工类别备注（可空）"));
    right->addWidget(category);
    initialize = button(
      "initializeSingleRoi", QStringLiteral("使用当前显示帧初始化"),
      controls);
    reset = button("resetSingleRoi", QStringLiteral("重置单目标跟踪"), controls);
    query = button("querySingleRoi", QStringLiteral("查询单目标状态"), controls);
    right->addWidget(initialize); right->addWidget(reset); right->addWidget(query);
    single_status = new QLabel(controls); single_status->setObjectName("singleTrackingState");
    single_status->setTextFormat(Qt::PlainText);
    single_status->setWordWrap(true);
    right->addWidget(
      single_status);
    right->addStretch(); controls->setMaximumWidth(340); row->addWidget(controls);
    tabs->addTab(single, QStringLiteral("单目标 ROI"));
    auto * gimbal = new QWidget(tabs); auto * g = new QVBoxLayout(gimbal);
    auto * form = new QFormLayout;
    const auto spin = [&](const char * name, double limit) {
        auto * s = new QDoubleSpinBox(gimbal);
        s->setObjectName(name);
        s->setRange(-limit, limit);
        s->setDecimals(1); s->setSuffix(QStringLiteral(" °")); return s;
      };
    pitch = spin("gimbalPitch", 90); yaw = spin("gimbalYaw", 180); roll = spin("gimbalRoll", 45);
    form->addRow(QStringLiteral("俯仰（正值向上）"), pitch);
    form->addRow(QStringLiteral("偏航（正值向左）"), yaw);
    form->addRow(QStringLiteral("横滚（绕前轴）"), roll); g->addLayout(form);
    angle = button(
      "setGimbalAngle", QStringLiteral("提交仿真云台角度"),
      gimbal);
    g->addWidget(angle);
    auto * tracking_row = new QHBoxLayout; target = new QComboBox(gimbal); target->setObjectName(
      "gimbalTargetId");
    track = button("enableGimbalTracking", QStringLiteral("跟踪所选目标"), gimbal);
    disable = button("disableGimbalTracking", QStringLiteral("停止云台跟踪"), gimbal);
    tracking_row->addWidget(target, 1); tracking_row->addWidget(track); tracking_row->addWidget(
      disable);
    g->addLayout(tracking_row); gimbal_status = new QLabel(gimbal); gimbal_status->setObjectName(
      "gimbalActualState");
    gimbal_status->setTextFormat(Qt::PlainText); gimbal_status->setWordWrap(true); g->addWidget(
      gimbal_status);
    g->addWidget(
      new QLabel(
        QStringLiteral(
          "云台为仿真后端；不改变飞机航向或"
          "下视相机外参。服务接受后仍需观察"
          "实际角度。"),
        gimbal));
    tabs->addTab(gimbal, QStringLiteral("仿真云台"));
    QObject::connect(
      open, &QPushButton::clicked, owner, [this] {
        if (!available || closing || port) {return;}
        try {
          bound_simulation = simulation; port = factory(
            {simulation, allowed && simulation,
              2});
        } catch (const std::exception & e) {
          status->setText(text(e.what()));
        }
        refresh();
      });
    QObject::connect(permit, &QCheckBox::toggled, owner, [this] {refresh();});
    QObject::connect(
      initialize, &QPushButton::clicked, owner, [this] {
        refresh(); if (!initialize->isEnabled()) {return;}
        const auto f = canvas->displayed(); const auto box = canvas->selection();
        easyfly_msgs::srv::ManageSingleTrack::Request r; r.operation = "initialize";
        r.image_header.stamp.sec = static_cast<int32_t>(f->stamp_ns / 1000000000LL);
        r.image_header.stamp.nanosec = static_cast<uint32_t>(f->stamp_ns % 1000000000LL);
        r.image_header.frame_id = f->source_frame; r.roi_x = box.x(); r.roi_y = box.y();
        r.roi_width = box.width();
        r.roi_height = box.height();
        r.class_name = category->text().toStdString();
        if (!port->single(r)) {
          status->setText(
            QStringLiteral(
              "初始化未提交；请检查帧、ROI 或未"
              "决请求。"));
        }
        refresh();
      });
    QObject::connect(
      reset, &QPushButton::clicked, owner, [this] {
        refresh(); if (!reset->isEnabled()) {return;}
        easyfly_msgs::srv::ManageSingleTrack::Request r; r.operation = "reset"; port->single(
          r); refresh();
      });
    QObject::connect(
      query, &QPushButton::clicked, owner, [this] {
        refresh(); if (!query->isEnabled()) {return;}
        easyfly_msgs::srv::ManageSingleTrack::Request r; r.operation = "query"; port->single(
          r); refresh();
      });
    QObject::connect(
      angle, &QPushButton::clicked, owner, [this] {
        refresh(); if (!angle->isEnabled()) {return;}
        easyfly_msgs::srv::SetGimbalAngle::Request r;
        r.pitch_angle = pitch->value(); r.yaw_angle = yaw->value(); r.roll_angle = roll->value();
        port->angle(r); refresh();
      });
    QObject::connect(
      track, &QPushButton::clicked, owner, [this] {
        refresh(); if (!track->isEnabled()) {return;}
        easyfly_msgs::srv::SetGimbalTracking::Request r; r.enabled = true;
        r.target_id = target->currentData().toInt(); port->tracking(r); refresh();
      });
    QObject::connect(
      disable, &QPushButton::clicked, owner, [this] {
        refresh(); if (!disable->isEnabled()) {return;}
        easyfly_msgs::srv::SetGimbalTracking::Request r; r.enabled = false; r.target_id = -1;
        port->tracking(r); refresh();
      });
    refresh();
  }
  bool frame_fresh(double now) const
  {
    const auto f = canvas->displayed();
    return available && source && source->clock.quality == bridge::Quality::ValidAtObservation &&
           f &&
           f->binding.session == source->observation_session_id &&
           f->binding.epoch == source->time_epoch &&
           f->binding.camera == 0 && f->binding.topic == "/camera/color/image_raw" &&
           f->source_frame == "camera_color_optical_frame" && f->stamp_ns > 0 &&
           source->clock.now_ns - f->stamp_ns >= -200000000LL &&
           source->clock.now_ns - f->stamp_ns <= 500000000LL &&
           now - f->first_received >= 0 && now - f->first_received <= .5 && f->error.empty();
  }
  void targets(double now)
  {
    const int previous = target->currentData().isValid() ? target->currentData().toInt() : -1;
    target->clear(); target_fresh = false;
    if (!source) {return;}
    const auto & o = source->camera_perception[0].targets;
    if (!available || !o.value || display_quality(o.evidence, *source, now, .5, true) !=
      bridge::Quality::ValidAtObservation) {return;}
    std::set<int32_t> ids;
    for (const auto & t : o.value->targets) {
      if (t.track_id < 0 || !t.position_valid || t.header.frame_id != "odom") {continue;}
      if (!ids.insert(t.track_id).second) {target->clear(); return;}
      target->addItem(
        QStringLiteral("ID %1 · %2").arg(t.track_id).arg(
          text(
            t.class_name,
            128)), t.track_id);
    }
    // Selection is explicit: never silently move to a different target.
    target->setCurrentIndex(target->findData(previous)); target_fresh = true;
  }
  void refresh()
  {
    if (port) {state = port->snapshot();}
    const double now = bridge::detail::steady_seconds();
    const bool live = state && state->running && !state->stopped &&
      now >= state->produced_steady && now - state->produced_steady <= .5;
    const bool permission = allowed &&
      simulation &&
      bound_simulation == simulation &&
      available &&
      !closing && permit->isChecked() && live;
    const bool base = permission && !state->pending && !state->uncertain;
    if (port) {port->inhibit(!permission);}
    open->setEnabled(available && !closing && !port);
    permit->setEnabled(allowed && available && simulation && !closing);
    const auto f = canvas->displayed(); const auto roi = canvas->selection();
    initialize->setEnabled(
      base && state->single_ready && frame_fresh(now) &&
      !roi.isEmpty() &&
      roi.width() * f->image.width() >= 8 &&
      roi.height() * f->image.height() >= 8 &&
      category->text().toUtf8().size() <= 128);
    reset->setEnabled(base && state->single_ready);
    query->setEnabled(
      available && !closing && live && state->single_ready &&
      !state->pending && !state->uncertain);
    const bool device = live && state->gimbal_fresh && state->gimbal &&
      state->gimbal->is_connected &&
      state->gimbal->header.frame_id == "base_link" &&
      state->gimbal->gimbal_model == "SIMULATION_ONLY_rate_limited";
    angle->setEnabled(base && device && state->angle_ready);
    track->setEnabled(
      base && device && state->tracking_ready && target_fresh && target->currentIndex() >= 0);
    disable->setEnabled(base && state->tracking_ready);
    if (state) {
      status->setText(
        QStringLiteral("请求 #%1 · %2 · %3\n%4").arg(state->sequence)
        .arg(
          text(state->operation, 64),
          state->uncertain ? QStringLiteral("结果未知；禁止重提") :
          state->pending ? QStringLiteral("等待服务响应") : state->reply_received ?
          (state->accepted ? QStringLiteral("服务已接受；未证明跟踪／到位") :
          QStringLiteral("服务已拒绝")) :
          QStringLiteral("无未决请求"), text(state->message, 512)));
    }
    canvas->tracked({});
    if (live && state->single_fresh && state->single) {
      const auto & m = *state->single;
      single_status->setText(
        QStringLiteral(
          "状态 %1 · ID %2\n%3\n%4 @ %5\n置信度未知"
          "；仅图像跟踪，不是 3D 目标。")
        .arg(text(m.phase, 32)).arg(m.track_id).arg(
          text(m.reason, 512),
          text(m.header.frame_id, 256))
        .arg(bridge::detail::stamp_ns(m.header.stamp)));
      if (m.valid) {
        const auto & t = m.track;
        single_status->setText(
          single_status->text() + QStringLiteral(
            "\n源帧框：中心 [%1, %2]，尺寸 [%3, %4]"
            "（归一化）")
          .arg(
            number(t.bbox_center_x), number(t.bbox_center_y), number(t.bbox_width),
            number(t.bbox_height)));
        if (!f || f->stamp_ns != bridge::detail::stamp_ns(m.header.stamp)) {
          single_status->setText(
            single_status->text() + QStringLiteral(
              "\n显示帧与跟踪源帧不同，未叠加跟"
              "踪框。"));
        }
      }
      if (m.valid &&
        frame_fresh(now) &&
        f->stamp_ns == bridge::detail::stamp_ns(m.header.stamp) &&
        f->source_frame == m.header.frame_id)
      {
        const auto & t = m.track; canvas->tracked(
          {t.bbox_center_x - t.bbox_width / 2,
            t.bbox_center_y - t.bbox_height / 2, t.bbox_width, t.bbox_height});
      }
    } else {
      single_status->setText(
        QStringLiteral(
          "单目标状态未知／过期；旧框已清除"
          "。\n初始化只接受当前显示的精确前"
          "视帧。"));
    }
    if (device) {
      const auto & m = *state->gimbal;
      gimbal_status->setText(
        QStringLiteral("实际角度：pitch=%1° yaw=%2° roll=%3°\n连接有效 · %4 · %5")
        .arg(
          number(m.pitch_angle), number(m.yaw_angle), number(m.roll_angle),
          text(m.gimbal_model, 128),
          m.is_tracking ? QStringLiteral("跟踪 ID %1").arg(m.tracking_target_id) : QStringLiteral(
            "跟踪已停止")));
    } else {
      gimbal_status->setText(
        QStringLiteral(
          "实际云台状态未知／过期／断开；不"
          "沿用旧角度。"));
    }
  }
};
PeripheralPanel::PeripheralPanel(bool enabled, QWidget * parent, PeripheralFactory factory)
: QWidget(parent), impl_(std::make_unique<Impl>(this, enabled, std::move(factory))) {}
PeripheralPanel::~PeripheralPanel() = default;
void PeripheralPanel::observe(
  std::shared_ptr<const bridge::UiSnapshot> s,
  std::shared_ptr<const ConvertedFrame> frame, bool available, bool simulation)
{
  auto & p = *impl_; p.source = std::move(s); p.available = available;
  p.simulation = simulation;
  if (p.source &&
    (p.session != p.source->observation_session_id || p.epoch != p.source->time_epoch))
  {
    p.permit->setChecked(false); p.canvas->frame({});
    p.session = p.source->observation_session_id; p.epoch = p.source->time_epoch;
  }
  if (!available) {p.permit->setChecked(false);}
  const double now = bridge::detail::steady_seconds();
  const bool image_current = available && frame && p.source && frame->binding.camera == 0 &&
    p.source->clock.quality == bridge::Quality::ValidAtObservation &&
    p.source->clock.now_ns - frame->stamp_ns >= -200000000LL &&
    p.source->clock.now_ns - frame->stamp_ns <= 500000000LL &&
    now >= frame->first_received && now - frame->first_received <= .5;
  p.canvas->frame(image_current ? frame : nullptr);
  p.targets(bridge::detail::steady_seconds()); p.refresh();
}
bool PeripheralPanel::unresolved() const
{
  const auto s = impl_->port ? impl_->port->snapshot() : nullptr;
  return s && (s->pending || s->uncertain);
}
bool PeripheralPanel::stopped() const
{return !impl_->port || impl_->port->snapshot()->stopped;}
void PeripheralPanel::stop()
{
  impl_->closing = true; impl_->canvas->frame({}); if (impl_->port) {
    impl_->port->inhibit(true); impl_->port->stop();
  }
}
}  // namespace easyfly_desktop
