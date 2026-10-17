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

#include <algorithm>
#include <cmath>

#include "easyfly_desktop/presentation.hpp"
namespace easyfly_desktop
{
QString text(const std::string & value, std::size_t limit)
{
  return QString::fromUtf8(value.data(), static_cast<int>(std::min(limit, value.size())));
}
QString number(double value, int precision)
{
  if (!std::isfinite(value)) {return QStringLiteral("—（非有限）");}
  return QString::number(value, std::abs(value) >= 1e6 ? 'g' : 'f', precision);
}
std::optional<double> enu_heading_degrees(double yaw_radians)
{
  if (!std::isfinite(yaw_radians)) {return std::nullopt;}
  constexpr double pi = 3.14159265358979323846;
  const double yaw = std::remainder(yaw_radians, 2 * pi);
  double heading = std::fmod(90 - yaw * 180 / pi, 360);
  if (heading < 0) {heading += 360;}
  return heading;
}
FlightInstruments flight_instruments(const bridge::UiSnapshot & s, double now)
{
  FlightInstruments result;
  const auto & state = s.state; const auto & v = state.sdk.value;
  const bool live = state.sdk.received && state.sdk.fresh && v.header.frame_id == "odom" &&
    display_quality(state.evidence, s, now, 1, true) == bridge::Quality::ValidAtObservation;
  result.position_valid = live && state.position_valid && v.pose_valid &&
    std::isfinite(v.position_x) && std::isfinite(v.position_y) && std::isfinite(v.position_z) &&
    std::abs(v.position_x) < 1e6 && std::abs(v.position_y) < 1e6 && std::abs(v.position_z) < 1e6;
  constexpr double pi = 3.14159265358979323846;
  const auto heading = enu_heading_degrees(v.yaw);
  result.attitude_valid = live && state.attitude_valid && v.attitude_valid && heading &&
    std::isfinite(v.roll) && std::isfinite(v.pitch) && std::abs(v.roll) <= pi &&
    std::abs(v.pitch) <= pi / 2;
  if (result.attitude_valid) {
    result.roll_degrees = v.roll * 180 / pi; result.pitch_degrees = v.pitch * 180 / pi;
    result.heading_degrees = *heading;
  }
  return result;
}
QString quality_text(bridge::Quality quality)
{
  switch (quality) {
    case bridge::Quality::NotReceived: return QStringLiteral("未收到");
    case bridge::Quality::ValidAtObservation: return QStringLiteral("本次观测有效");
    case bridge::Quality::Stale: return QStringLiteral("已过期 / 最后已知");
    case bridge::Quality::Invalid: return QStringLiteral("响应或数据无效");
    case bridge::Quality::ClockNotReady: return QStringLiteral("时钟未就绪");
    case bridge::Quality::ClockPaused: return QStringLiteral("模拟时钟暂停");
    case bridge::Quality::ClockReset: return QStringLiteral("时钟回退 / 旧时间域");
    case bridge::Quality::Unavailable: return QStringLiteral("端点不可用");
    case bridge::Quality::Unsupported: return QStringLiteral("不支持");
    case bridge::Quality::BudgetExceeded: return QStringLiteral("超过显示预算");
    case bridge::Quality::Unknown: return QStringLiteral("未知 / 未确认");
  }
  return QStringLiteral("未知");
}
double snapshot_delay(const bridge::UiSnapshot & snapshot, double now)
{
  return snapshot.produced_steady > 0 ? std::max(0.0, now - snapshot.produced_steady) : 0.0;
}
bridge::Quality display_quality(
  const bridge::Evidence & e, const bridge::UiSnapshot & s,
  double now, double ttl, bool clock_required)
{
  if (e.quality != bridge::Quality::ValidAtObservation) {return e.quality;}
  if (e.time_epoch != s.time_epoch) {return bridge::Quality::ClockReset;}
  if (!s.running || s.stopped || snapshot_delay(s, now) > 1.0 ||
    e.receipt_age + snapshot_delay(s, now) > ttl) {return bridge::Quality::Stale;}
  if (clock_required && s.clock.quality != bridge::Quality::ValidAtObservation) {
    return s.clock.quality;
  }
  return e.quality;
}
QString ages(const bridge::Evidence & e, const bridge::UiSnapshot & s, double now)
{
  const auto receipt = e.received_steady >
    0 ? number(e.receipt_age + snapshot_delay(s, now)) : QStringLiteral("—");
  const auto source = e.source_age_available ? number(e.source_age) : QStringLiteral("—");
  // Never advance ROS source age using steady time: simulation can pause or change rate.
  return QStringLiteral(
    "接收/查询年龄 %1 s · "
    "源年龄（快照时）%2 s")
         .arg(receipt, source);
}
QString phase_event(const bridge::UiSnapshot & s, double now)
{
  const auto & event = s.last_mission_event;
  if (!event.value) {
    return QStringLiteral("全局阶段事件：%1\n不关联执行 ID，不作为心跳或终态证明。")
           .arg(quality_text(event.evidence.quality));
  }
  const auto & v = *event.value;
  return QStringLiteral(

    "最近全局阶段：%1 · %2\n索"
    "引/总数（原值）：%3 / %4 "
    "· 进度（上游）：%5%\n%6\n"
    "%7\n无执行身份；长时间无"
    "更新不等于任务失败。")
         .arg(text(v.phase_name), quality_text(event.evidence.quality))
         .arg(v.current_waypoint).arg(v.total_waypoints).arg(number(v.progress * 100, 1))
         .arg(ages(event.evidence, s, now), text(v.message, 2048));
}
QString mission_result(
  const bridge::Observation<easyfly_msgs::srv::ManageMission::Response> & r,
  const bridge::UiSnapshot & s, double now, const std::string & requested_id)
{
  const auto q = display_quality(r.evidence, s, now, 2, false);
  const QString prefix = QStringLiteral("请求身份（服务 execution_id）：%1\n%2 · %3\n")
    .arg(text(requested_id), quality_text(q), ages(r.evidence, s, now));
  if (!r.value) {
    return prefix + text(r.evidence.reason, 2048) + QStringLiteral(
      "\n没有可核对结果；不能确"
      "认执行已停止或已排空。");
  }
  const auto & v = *r.value;
  if (v.success && v.execution_id != requested_id) {
    return prefix + QStringLiteral("响应身份错配；拒绝关联，不能确认执行已停止。");
  }
  const auto pending = v.success && v.execution_id == requested_id ?
    (v.execution_pending ? QStringLiteral("是（仍待确认排空）") :
    QStringLiteral("上游报告否（仅本次查询）")) :
    QStringLiteral("未确认；not_found/查询失败不证明已停止");
  const auto feedback = v.feedback_valid &&
    s.clock.quality == bridge::Quality::ValidAtObservation &&
    q == bridge::Quality::ValidAtObservation ? QStringLiteral("上游在查询时有效；不是当前心跳") :
    QStringLiteral("不可用或最后已知，不据此判断终态");
  return prefix + QStringLiteral(
    "服务查询处理：%1（不是任"
    "务成功）\n执行状态：%2\n"
    "返回执行身份：%3\n执行仍"
    "待排空：%4\n反馈：%5\n%6")
         .arg(
    v.success ? QStringLiteral("成功") : QStringLiteral("失败"), text(v.status),
    text(v.execution_id), pending, feedback, text(v.message, 2048));
}
bool geometry_current(const bridge::UiSnapshot & s, bridge::Key key, double now)
{
  const auto found = s.perception.find(key);
  if (found == s.perception.end() || !found->second.value ||
    display_quality(found->second.evidence, s, now, 1, true) != bridge::Quality::ValidAtObservation)
  {return false;}
  return bridge::detail::validate_perception(*found->second.value, key, s.clock) ==
         bridge::Quality::ValidAtObservation;
}
}  // namespace easyfly_desktop
