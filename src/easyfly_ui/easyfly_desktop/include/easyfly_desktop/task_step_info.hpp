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
#ifndef EASYFLY_DESKTOP__TASK_STEP_INFO_HPP_
#define EASYFLY_DESKTOP__TASK_STEP_INFO_HPP_
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>
#include <string>
namespace easyfly_desktop
{
struct TaskStepInfo {QString key, title, description, requirements;};
inline const QVector<TaskStepInfo> & task_step_infos()
{
  static const QVector<TaskStepInfo> values{
    {"takeoff", QStringLiteral("起飞"), QStringLiteral("起飞到指定高度，使用设定的爬升速度。"),
      QStringLiteral("需要飞控通信、飞控控制与有效定位。")},
    {"hover", QStringLiteral("悬停／等待"),
      QStringLiteral("等待指定时间；空中延续原有保持，地面仅等待，不会自行起飞。"),
      QStringLiteral("需要任务状态机；空中保持仍依赖已有飞控控制。")},
    {"waypoint", QStringLiteral("航点飞行"),
      QStringLiteral("直接提交位置与航向目标，不经过导航避障规划。"),
      QStringLiteral("需要飞控控制与有效定位；到达条件由位置半径等参数决定。")},
    {"navigate", QStringLiteral("规划导航"),
      QStringLiteral("由当前已启动的导航策略规划并前往目标坐标。"),
      QStringLiteral("需要定位、地图、导航与所选规划后端就绪。")},
    {"track", QStringLiteral("目标跟随"),
      QStringLiteral("在指定时长内跟随目标，保持距离及相对高度。"),
      QStringLiteral("需要目标三维观测、飞行跟随后端和飞控；图像跟踪不能替代飞行跟随。")},
    {"search", QStringLiteral("区域搜索"),
      QStringLiteral("在矩形区域内按往复式扫线导航，并统计观察到的目标。"),
      QStringLiteral("需要区域搜索、导航、定位与地图；完成路线不代表传感器覆盖保证。")},
    {"land", QStringLiteral("普通降落"), QStringLiteral("执行普通下降着陆，不使用视觉标记对准。"),
      QStringLiteral("需要飞控通信与降落控制后端。")},
    {"precision_land", QStringLiteral("精准降落"),
      QStringLiteral("对准指定 AprilTag 标记后下降着陆。"),
      QStringLiteral("需要完整精降链、下视相机与静态 AprilTag 36h11 标记板；普通降落准备不够。")},
    {"explore", QStringLiteral("自主探索"),
      QStringLiteral("在给定三维范围内自主选择探索视点；当前探索后端为 FUEL。"),
      QStringLiteral(
        "需要当前平台的 FAST-LIO、三维地图、FUEL 与 GCOPTER；"
        "A*／EGO不能代替此探索链。")}};
  return values;
}
inline TaskStepInfo task_step_info(const std::string & key)
{
  for (const auto & value : task_step_infos()) {
    if (value.key == QString::fromStdString(key)) {return value;}
  }
  return {QString::fromStdString(key), QString::fromStdString(key), {}, {}};
}
inline QString task_step_title(const std::string & key)
{
  const auto info = task_step_info(key); return info.title + " (" + info.key + ")";
}
inline QString task_step_description(const std::string & key)
{return task_step_info(key).description;}
inline QString task_step_requirements(const std::string & key)
{return task_step_info(key).requirements;}
inline QString task_parameter_title(const std::string & key)
{
  static const QMap<QString, QString> names{
    {"altitude", QStringLiteral("目标高度")}, {"climb_speed", QStringLiteral("爬升速度")},
    {"duration", QStringLiteral("持续时间")}, {"timeout", QStringLiteral("步骤超时")},
    {"x", QStringLiteral("目标 X 坐标")}, {"y", QStringLiteral("目标 Y 坐标")},
    {"z", QStringLiteral("目标 Z 坐标")}, {"yaw", QStringLiteral("目标航向")},
    {"max_speed", QStringLiteral("最大速度")}, {"acceptance_radius", QStringLiteral("到达半径")},
    {"target_id", QStringLiteral("跟随目标 ID")}, {"follow_distance", QStringLiteral("跟随距离")},
    {"follow_height", QStringLiteral("跟随高度偏移")}, {"center_x", QStringLiteral("搜索中心 X")},
    {"center_y", QStringLiteral("搜索中心 Y")}, {"area_width", QStringLiteral("搜索区域宽度")},
    {"area_height", QStringLiteral("搜索区域长度")}, {"tag_id", QStringLiteral("降落标记 ID")},
    {"min_x", QStringLiteral("探索最小 X")}, {"min_y", QStringLiteral("探索最小 Y")},
    {"min_z", QStringLiteral("探索最低 Z")}, {"max_x", QStringLiteral("探索最大 X")},
    {"max_y", QStringLiteral("探索最大 Y")}, {"max_z", QStringLiteral("探索最高 Z")}};
  const auto value = QString::fromStdString(key); return names.value(value, value);
}
inline QString task_parameter_hint(const std::string & type, const std::string & key)
{
  QString constraint;
  if (key == "timeout") {
    constraint = type == "explore" ? QStringLiteral("5～900秒，包含成功后的返程。") :
      QStringLiteral("0.05～3600秒；步骤时间上限，不是预计飞行时间。");
  } else if (key == "duration") {
    constraint = type == "hover" ? QStringLiteral("0～3600秒，不能超过步骤超时。") :
      QStringLiteral("大于0且不超过3600秒，也不能超过步骤超时。");
  } else if (key == "altitude") {
    constraint = type == "search" ? QStringLiteral("0.5～50米。") :
      QStringLiteral("大于0且不超过50米。");
  } else if (key == "climb_speed" || key == "max_speed") {
    constraint = QStringLiteral("0.01～20米/秒，运行时仍受平台限制。");
  } else if (key == "x" || key == "y" || key == "center_x" || key == "center_y") {
    constraint = QStringLiteral("odom 局部坐标，-1000～1000米；以定位原点为基准。");
  } else if (key == "z") {
    constraint = QStringLiteral("odom 局部高度坐标，0～50米，Z向上。");
  } else if (key == "yaw") {
    constraint = QStringLiteral("-π～π弧度。");
  } else if (key == "acceptance_radius") {
    constraint = QStringLiteral("0.05～5米。");
  } else if (key == "target_id") {
    constraint = QStringLiteral("整数，-1～2147483647；按实际目标跟随后端核对ID策略。");
  } else if (key == "tag_id") {
    constraint = QStringLiteral("整数，0～2147483647；须与实际降落标记一致。");
  } else if (key == "follow_distance") {
    constraint = QStringLiteral("0.1～30米。");
  } else if (key == "follow_height") {
    constraint = QStringLiteral("相对目标的高度偏移，-10～10米。");
  } else if (key == "area_width" || key == "area_height") {
    constraint = QStringLiteral("0.1～100米；搜索按往复式路线覆盖。");
  } else if (key.find("min_") == 0 || key.find("max_") == 0) {
    constraint = QStringLiteral(
      "odom 三维边界坐标，每轴最大值减最小值至少0.8米；Z范围须在0.5～2.5米内。");
  }
  return QString::fromStdString(key) + "\n" + constraint +
         QStringLiteral("\n这些是文件格式限制；设备就绪与运行准入由平台另行检查。");
}
struct TaskTemplateInfo {QString key, title, note; QStringList types;};
inline const QVector<TaskTemplateInfo> & task_template_infos()
{
  static const QVector<TaskTemplateInfo> values{
    {"basic", QStringLiteral("基础起飞与降落"),
      QStringLiteral("起飞、等待、降落；需飞控与任务状态机。"),
      {"takeoff", "hover", "land"}},
    {"waypoint", QStringLiteral("航点巡检"),
      QStringLiteral("往返两个航点；直接航点不提供导航避障，须修改坐标与高度。"),
      {"takeoff", "waypoint", "waypoint", "land"}},
    {"search", QStringLiteral("区域搜索"),
      QStringLiteral("需定位、地图、导航与区域搜索后端；须核对区域。"),
      {"takeoff", "search", "land"}},
    {"explore", QStringLiteral("自主探索"),
      QStringLiteral("当前需 FAST-LIO、FUEL、GCOPTER；须核对三维范围。"),
      {"takeoff", "explore", "land"}},
    {"precision_land", QStringLiteral("精准降落"),
      QStringLiteral("需完整下视 AprilTag 精降链；须按实际标记修改ID。"),
      {"takeoff", "navigate", "precision_land"}},
    {"track", QStringLiteral("目标跟随"),
      QStringLiteral("需三维目标与飞行跟随后端；须按实际目标修改ID。"),
      {"takeoff", "track", "land"}}};
  return values;
}
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__TASK_STEP_INFO_HPP_
