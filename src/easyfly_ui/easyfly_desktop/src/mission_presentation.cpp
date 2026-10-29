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
#include <QStringList>
#include <cmath>
#include <algorithm>
#include "easyfly_desktop/presentation.hpp"
namespace easyfly_desktop
{
QString missing_mission_backends(const bridge::UiSnapshot & snapshot, double now)
{
  if (!snapshot.mission_plan) {return {};}
  const auto & steps = snapshot.mission_plan->steps;
  if (std::all_of(
      steps.begin(), steps.end(),
      [](const auto & step) {return step.type == "hover";}))
  {
    return {};
  }
  const bool depends_on_localization = std::any_of(
    steps.begin(), steps.end(),
    [](const auto & step) {
      return step.type == "navigate" || step.type == "track" || step.type == "search" ||
      step.type == "explore";
    });
  if (std::any_of(steps.begin(), steps.end(), [](const auto & s) {return s.type == "explore";}) &&
    (snapshot.selected_profile != "navigation_dev" ||
    snapshot.selected_localization_backend != "fast_lio"))
  {
    return QStringLiteral("FUEL 探索需要导航仿真、FAST-LIO、GCOPTER及探索后端；请检查连接方案");
  }
  if (depends_on_localization &&
    (snapshot.selected_localization_backend == "openvins" ||
    snapshot.selected_localization_backend == "orbslam3" ||
    snapshot.selected_localization_backend == "fast_livo2"))
  {
    const auto name = snapshot.selected_localization_backend == "openvins" ? "OpenVINS" :
      snapshot.selected_localization_backend == "orbslam3" ? "ORB-SLAM3" : "FAST-LIVO2";
    if (!snapshot.localization_navigation_enabled ||
      snapshot.selected_profile != "navigation_dev")
    {
      return QStringLiteral("%1 当前为观测模式；请在连接管理中选择导航仿真并重建会话").arg(name);
    }
    const auto & raw = snapshot.localization;
    const auto & nav = snapshot.navigation_localization;
    if (!raw.value || !nav.value ||
      display_quality(
        raw.evidence, snapshot, now, .5,
        true) != bridge::Quality::ValidAtObservation ||
      display_quality(
        nav.evidence, snapshot, now, .5,
        true) != bridge::Quality::ValidAtObservation ||
      bridge::detail::validate_navigation_localization(
        *nav.value, snapshot.selected_localization_backend, snapshot.clock,
        nav.evidence.receipt_age + snapshot_delay(snapshot, now)) !=
      bridge::Quality::ValidAtObservation ||
      raw.value->session_id != nav.value->session_id ||
      raw.value->generation != nav.value->generation)
    {
      return QStringLiteral("%1 导航定位尚未就绪、已过期或会话不一致；请检查算法状态").arg(name);
    }
  }
  if (depends_on_localization && snapshot.selected_localization_backend == "disabled") {
    return QStringLiteral("定位已关闭：导航/跟踪/搜索不可用");
  }
  if (!snapshot.running || snapshot.stopped || !std::isfinite(snapshot.backend_graph_observed) ||
    snapshot.backend_graph_observed <= 0 || now < snapshot.backend_graph_observed ||
    now - snapshot.backend_graph_observed > 2)
  {
    return QStringLiteral("任务依赖接口尚未取得新鲜观测；请等待连接或检查运行方案");
  }
  QStringList missing;
  for (const auto & step : steps) {
    if (step.type == "hover") {continue;}
    const auto backend = step.type == "waypoint" ? std::string("goto") : step.type;
    const auto found = snapshot.task_backends.find(backend);
    if (found == snapshot.task_backends.end() || !found->second) {
      missing.push_back(text(step.type, 64));
    }
  }
  missing.removeDuplicates();
  return missing.empty() ? QString{} : QStringLiteral(
    "缺少任务依赖接口：%1。请在连接与仿真检查运行方案；"
    "精降准备不包含完整精降。")
         .arg(missing.join(QStringLiteral("、")));
}
QString mission_step_parameters(const easyfly_mission_fsm::MissionStep & step)
{
  QStringList values;
  const auto field = [&](const char * name, double value, const char * unit) {
      values << QString("%1=%2 %3").arg(name, number(value, 6), unit).trimmed();
    };
  if (step.type == "takeoff") {
    field("altitude", step.altitude, "m"); field("climb_speed", step.climb_speed, "m/s");
  } else if (step.type == "waypoint" || step.type == "navigate") {
    field("x", step.x, "m"); field("y", step.y, "m"); field("z", step.z, "m");
    field("max_speed", step.max_speed, "m/s");
    if (step.type == "waypoint") {
      field("yaw", step.yaw, "rad"); field("acceptance_radius", step.acceptance_radius, "m");
    }
  } else if (step.type == "hover") {
    field("duration", step.wait, "s");
  } else if (step.type == "track") {
    values << QString("target_id=%1").arg(step.target_id);
    field("duration", step.duration, "s"); field("follow_distance", step.follow_distance, "m");
    field("follow_height", step.follow_height, "m");
  } else if (step.type == "search") {
    field("center_x", step.center_x, "m"); field("center_y", step.center_y, "m");
    field("area_width", step.area_width, "m"); field("area_height", step.area_height, "m");
    field("altitude", step.altitude, "m"); field("max_speed", step.max_speed, "m/s");
    values << "pattern=boustrophedon (geometric coverage estimate only)";
  } else if (step.type == "explore") {
    values << QString("min=[%1,%2,%3] m; max=[%4,%5,%6] m; 禁飞盒=%7; 返回起点=%8")
      .arg(
      number(step.min_bound[0], 6), number(step.min_bound[1], 6), number(step.min_bound[2], 6),
      number(step.max_bound[0], 6), number(step.max_bound[1], 6), number(step.max_bound[2], 6))
      .arg(step.forbidden_boxes.size()).arg(
      step.return_home ? QStringLiteral("是") : QStringLiteral(
        "否"));
  } else if (step.type == "precision_land") {values << QString("tag_id=%1").arg(step.tag_id);}
  return values.isEmpty() ? QStringLiteral("无任务文件参数") : values.join("; ");
}
}  // namespace easyfly_desktop
