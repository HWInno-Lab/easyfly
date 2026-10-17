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

#include "easyfly_desktop/presentation.hpp"
namespace easyfly_desktop
{
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
