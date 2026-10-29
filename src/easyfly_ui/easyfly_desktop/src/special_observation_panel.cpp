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
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QTableWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <utility>
#include <easyfly_ui_bridge/special_observations.hpp>
#include "easyfly_desktop/special_observation_panel.hpp"
#include "easyfly_desktop/presentation.hpp"
namespace easyfly_desktop
{
namespace
{
QString yes(bool value) {return value ? QStringLiteral("是") : QStringLiteral("否");}
QTableWidget * table(const QString & name, const QStringList & headings, QWidget * parent)
{
  auto * t = new QTableWidget(0, headings.size(), parent); t->setObjectName(name);
  t->setHorizontalHeaderLabels(headings); t->setEditTriggers(QAbstractItemView::NoEditTriggers);
  t->setSelectionBehavior(QAbstractItemView::SelectRows); t->setWordWrap(false);
  t->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  t->horizontalHeader()->setStretchLastSection(true); return t;
}
void rows(QTableWidget * t, const QVector<QStringList> & values)
{
  if (t->rowCount() != values.size()) {t->setRowCount(values.size());}
  for (int row = 0; row < values.size(); ++row) {
    for (int col = 0; col < values[row].size(); ++col) {
      const auto * previous = t->item(row, col);
      if (previous && previous->text() == values[row][col]) {continue;}
      auto * item = new QTableWidgetItem(values[row][col]);
      item->setToolTip("<qt>" + values[row][col].toHtmlEscaped() + "</qt>");
      t->setItem(row, col, item);
    }
  }
}
QString stamp(const std_msgs::msg::Header & h)
{return text(h.frame_id, 256) + " @ " + QString::number(bridge::detail::stamp_ns(h.stamp));}
template<typename Message>
QString status(const bridge::Observation<Message> & o, const bridge::UiSnapshot & s, double now)
{
  const auto q = display_quality(o.evidence, s, now, .5, true);
  return text(
    o.evidence.endpoint,
    256) + " · " + quality_text(q) + " · " + ages(o.evidence, s, now) +
         (o.value ? "\n" + stamp(o.value->header) : QString{}) + "\n" +
         text(o.evidence.reason, 512);
}
template<typename Message>
bool valid(const bridge::Observation<Message> & o, const bridge::UiSnapshot & s, double now)
{
  return s.running && !s.stopped && o.value &&
         display_quality(o.evidence, s, now, .5, true) == bridge::Quality::ValidAtObservation;
}
QString xyz(double x, double y, double z)
{return QString("[%1, %2, %3]").arg(number(x), number(y), number(z));}
}  // namespace
struct SpecialObservationPanel::Impl
{
  bool landing;
  QTableWidget * first{nullptr}, *second{nullptr};
  QLabel * first_state{nullptr}, *second_state{nullptr};
  QPlainTextEdit * detail{nullptr}, *feedback{nullptr};
  explicit Impl(bool is_landing, QWidget * owner)
  : landing(is_landing)
  {
    auto * layout = new QVBoxLayout(owner); auto * tabs = new QTabWidget(owner);
    layout->addWidget(tabs);
    const auto page = [&](const QString & title, QLabel * & state) {
        auto * box = new QWidget(tabs); auto * l = new QVBoxLayout(box);
        state = new QLabel(box); state->setTextFormat(Qt::PlainText); state->setWordWrap(true);
        l->addWidget(state); tabs->addTab(box, title); return l;
      };
    if (landing) {
      owner->setObjectName("landingObservations");
      auto * tags = page(QStringLiteral("原始标签"), first_state);
      first = table(
        "tagObservationTable", {"ID", QStringLiteral("有效"), "光学 XYZ m",
          "尺寸 m", "误差 px", "距离 m", QStringLiteral("原因")}, owner);
      tags->addWidget(first);
      auto * local = page(QStringLiteral("所选标签与局部板"), second_state);
      detail = new QPlainTextEdit(owner); detail->setReadOnly(true);
      detail->setObjectName("landingObservationDetails"); local->addWidget(detail);
      feedback = new QPlainTextEdit(owner); feedback->setReadOnly(true);
      feedback->setObjectName("ownedStepFeedback");
      tabs->addTab(feedback, QStringLiteral("所属动作反馈"));
    } else {
      owner->setObjectName("predictionClassificationObservations");
      auto * predictions = page(QStringLiteral("模型预测（非实测）"), first_state);
      first = table(
        "predictionTable", {"ID", QStringLiteral("有效／原因"), QStringLiteral("类别"),
          "预测 XYZ m", "速度 m/s", "horizon s", "原观测龄 s",
          "原观测 frame @ ns", "模型方差 m²"}, owner);
      predictions->addWidget(first);
      auto * classified = page(QStringLiteral("独立障碍类别副本"), second_state);
      second = table(
        "classifiedObstacleTable", {"几何 ID", QStringLiteral("当前类别"), "目标 ID",
          "置信度", "中心 XYZ m", "尺寸 XYZ m", "observed ns", "expires ns"}, owner);
      classified->addWidget(second);
    }
  }
};
SpecialObservationPanel::SpecialObservationPanel(bool landing, QWidget * parent)
: QWidget(parent), impl_(std::make_unique<Impl>(landing, this)) {}
SpecialObservationPanel::~SpecialObservationPanel() = default;
void SpecialObservationPanel::observe(
  const bridge::UiSnapshot & s, double now,
  std::shared_ptr<const easyfly_ui_control::ControlSnapshot> owned)
{
  auto & p = *impl_; const auto & data = s.special;
  QVector<QStringList> values;
  if (p.landing) {
    p.first_state->setText(
      status(
        data.tags, s,
        now) + QStringLiteral(
        "\n检测有效不等于允许降落；不绘制"
        "未提供的实测角点。"));
    if (data.tags.value) {
      p.first_state->setText(
        p.first_state->text() +
        QString("\n数量=%1 · ").arg(data.tags.value->observations.size()) +
        text(data.tags.value->reason, 512));
    }
    if (valid(data.tags, s, now)) {
      for (const auto & tag : data.tags.value->observations) {
        values.push_back(
          {QString::number(tag.tag_id), yes(tag.valid), tag.valid ?
            xyz(tag.pose.position.x, tag.pose.position.y, tag.pose.position.z) : "—",
            tag.valid ? number(tag.tag_size_m) : "—", tag.valid ? number(
              tag.reprojection_error_px) : "—",
            tag.valid ? number(tag.range_m) : "—", text(tag.reason, 512)});
      }
    }
    rows(p.first, values);
    p.second_state->setText(
      QStringLiteral(
        "光学深度是相机 Z，局部板法向来自"
        "有效 guidance；均不是下降指令。"));
    QStringList details{status(data.landing, s, now)};
    if (data.landing.value) {details << "reason: " + text(data.landing.value->reason, 512);}
    if (valid(data.landing, s, now)) {
      const auto & m = *data.landing.value;
      details << QString("tag ID=%1; optical Z=%2 m; optical XY yaw=%3 rad; size=%4 m")
        .arg(m.tag_id).arg(number(m.optical_depth_m), number(m.yaw_rad), number(m.tag_size_m));
    }
    details << status(data.guidance, s, now);
    if (data.guidance.value) {details << "reason: " + text(data.guidance.value->reason, 512);}
    if (valid(data.guidance, s, now)) {
      const auto & m = *data.guidance.value;
      details << "source: " + stamp(m.source_header) << QString(
        "tag ID=%1; odom center=%2 m; normal=%3")
        .arg(m.tag_id).arg(
        xyz(m.position.x, m.position.y, m.position.z),
        xyz(m.plane_normal.x, m.plane_normal.y, m.plane_normal.z));
    }
    p.detail->setPlainText(details.join('\n'));
    QStringList action{status(data.step_feedback, s, now),
      QStringLiteral(
        "时间戳是反馈转发接收时刻；只显示"
        "与本地已登记父任务匹配的子动作反"
        "馈。")};
    if (data.step_feedback.value) {
      const auto & m = *data.step_feedback.value;
      action << "mission Goal: " + text(m.mission_goal_id, 32) << "child Goal: " + text(
        m.child_goal_id, 32);
      const bool associated = owned && owned->intent && !owned->intent->resolved &&
        !owned->stopped && owned->intent->goal == m.mission_goal_id &&
        now >= owned->session_received && now - owned->session_received <= 2;
      if (valid(data.step_feedback, s, now) && associated) {
        action << QString("%1 · %2 · step=%3 · SDK feedback receipt age=%4 s")
          .arg(
          text(m.step_type, 64),
          text(m.phase, 128)).arg(m.step_index + 1).arg(number(m.feedback_age_sec));
        if (m.step_type == "precision_land") {
          action << "target_valid=" + yes(m.target_valid);
          if (m.target_valid) {
            action << QString("horizontal error=%1 m; height above target=%2 m")
              .arg(number(m.horizontal_error_m), number(m.height_above_target_m));
          }
        } else {
          action << QString("coverage=%1; completed viewpoints=%2; next=%3 m")
            .arg(number(m.coverage)).arg(m.completed_viewpoints)
            .arg(xyz(m.next_viewpoint.x, m.next_viewpoint.y, m.next_viewpoint.z));
        }
      } else {
        action << QStringLiteral(
          "未取得新鲜、属于当前本地执行的动"
          "作反馈；阶段／误差不显示。服务状"
          "态仍以任务页为准。");
      }
    }
    p.feedback->setPlainText(action.join('\n'));
  } else {
    p.first_state->setText(
      status(
        data.predictions, s,
        now) + QStringLiteral(
        "\n预测参考时间与原观测时间分别保"
        "留；方差是模型估计。"));
    if (valid(data.predictions, s, now)) {
      for (const auto & m : data.predictions.value->predictions) {
        values.push_back(
          {QString::number(m.track_id), yes(m.valid) + " · " + text(m.reason, 512),
            text(m.class_name, 128),
            m.valid ? xyz(m.position_x, m.position_y, m.position_z) : "—",
            m.valid ? xyz(m.velocity_x, m.velocity_y, m.velocity_z) : "—",
            m.valid ? number(m.horizon_seconds) : "—", m.valid ? number(
              m.observation_age_seconds) : "—",
            stamp(m.observation_header), m.valid ? xyz(
              m.variance_x, m.variance_y,
              m.variance_z) : "—"});
      }
    }
    rows(p.first, values); values.clear();
    p.second_state->setText(
      status(
        data.classified, s,
        now) + QStringLiteral(
        "\n类别无效／过期时显示未知，合法"
        "几何保留；不修改原障碍或占据地图"
        "。"));
    if (data.classified.value) {
      p.second_state->setText(
        p.second_state->text() + "\n" +
        text(data.classified.value->reason, 512));
    }
    if (valid(data.classified, s, now)) {
      for (const auto & m : data.classified.value->obstacles) {
        const auto & o = m.obstacle;
        const bool associated = bridge::special::class_current(m, s.clock.now_ns);
        values.push_back(
          {QString::number(o.obstacle_id),
            associated ? text(o.obstacle_type, 128) : QStringLiteral("未知／过期"),
            associated ? QString::number(m.target_id) : "—", associated ? number(
              m.confidence) : "—",
            xyz(o.center_x, o.center_y, o.center_z), xyz(o.size_x, o.size_y, o.size_z),
            QString::number(bridge::detail::stamp_ns(m.observed_at)),
            QString::number(bridge::detail::stamp_ns(m.expires_at))});
      }
    }
    rows(p.second, values);
  }
}
}  // namespace easyfly_desktop
