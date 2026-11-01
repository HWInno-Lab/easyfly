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
#include "easyfly_desktop/overview_task.hpp"
#include "easyfly_desktop/presentation.hpp"
#include "easyfly_desktop/task_step_info.hpp"
namespace easyfly_desktop
{
OverviewTaskOverlay overview_task_overlay(const bridge::UiSnapshot & snapshot, double now)
{
  OverviewTaskOverlay result;
  result.reason = QStringLiteral("等待实际执行任务；编辑草稿和目录选择不作为执行进度。");
  const auto & observation = snapshot.mission_execution;
  if (!observation.value) {return result;}
  const auto & execution = *observation.value;
  const auto quality = display_quality(observation.evidence, snapshot, now, 2, true);
  if (quality != bridge::Quality::ValidAtObservation && quality != bridge::Quality::Stale) {
    result.reason = QStringLiteral("任务观测不可用：") + quality_text(quality); return result;
  }
  if (!execution.plan || !execution.plan->valid() || execution.frame != "odom" ||
    execution.total_steps != static_cast<int>(execution.plan->steps.size()) ||
    execution.completed_steps < 0 || execution.completed_steps > execution.total_steps ||
    execution.current_step < -1 || execution.current_step >= execution.total_steps)
  {return result;}
  result.bound = true; result.fresh = quality == bridge::Quality::ValidAtObservation;
  result.task_name = execution.name.rfind("easyfly-d2-mission-", 0) == 0 ?
    QStringLiteral("当前执行任务") : text(execution.name, 128);
  result.goal_id = text(execution.goal_id, 32);
  result.state = text(execution.state, 32); result.frame = text(execution.frame, 128);
  result.geometry = draft_geometry(*execution.plan);
  result.total_steps = execution.total_steps; result.completed_steps = execution.completed_steps;
  result.current_step = result.fresh && execution.state == "active" ? execution.current_step : -1;
  const auto prefix = QStringLiteral("%1 · 已完成 %2/%3 步")
    .arg(result.task_name).arg(result.completed_steps).arg(result.total_steps);
  if (!result.fresh) {
    result.summary = prefix + QStringLiteral(" · 最后已知，进度未更新");
  } else if (execution.state == "active" && result.current_step >= 0) {
    result.summary = prefix + QStringLiteral(" · 第 %1/%2 步：%3")
      .arg(result.current_step + 1).arg(result.total_steps)
      .arg(task_step_title(execution.plan->steps[result.current_step].type));
    const auto & step = execution.plan->steps[result.current_step];
    if (step.type == "waypoint" || step.type == "navigate") {
      result.current_target = std::array<double, 3>{step.x, step.y, step.z};
    } else if (step.type == "explore") {
      const auto & feedback = snapshot.special.step_feedback;
      if (feedback.value && feedback.value->active &&
        feedback.value->mission_goal_id == execution.goal_id &&
        feedback.value->step_index == static_cast<unsigned>(result.current_step) &&
        feedback.value->step_type == "explore" && feedback.value->header.frame_id == "odom" &&
        display_quality(feedback.evidence, snapshot, now, .5, true) ==
        bridge::Quality::ValidAtObservation)
      {
        const auto & point = feedback.value->next_viewpoint;
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
          result.current_target = std::array<double, 3>{point.x, point.y, point.z};
        }
      }
    }
  } else {
    const auto state = execution.state == "succeeded" ? QStringLiteral("任务步骤完成") :
      execution.state == "failed" ? QStringLiteral("任务失败") :
      execution.state == "canceled" ? QStringLiteral("任务已取消") : QStringLiteral("等待步骤反馈");
    result.summary = prefix + QStringLiteral(" · ") + state;
  }
  result.reason = QStringLiteral("关联实际执行 %1；虚线仅表示任务步骤顺序。")
    .arg(result.goal_id.left(8));
  return result;
}
}  // namespace easyfly_desktop
