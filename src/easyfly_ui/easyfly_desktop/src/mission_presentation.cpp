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
      return step.type == "navigate" || step.type == "track" || step.type == "search";
    });
  if (depends_on_localization && snapshot.selected_localization_backend == "openvins") {
    return QStringLiteral("OpenVINS 当前仅观测：导航/跟踪/搜索尚未完成 odom 对齐与地图接入");
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
  } else if (step.type == "precision_land") {values << QString("tag_id=%1").arg(step.tag_id);}
  return values.isEmpty() ? QStringLiteral("无任务文件参数") : values.join("; ");
}
}  // namespace easyfly_desktop
