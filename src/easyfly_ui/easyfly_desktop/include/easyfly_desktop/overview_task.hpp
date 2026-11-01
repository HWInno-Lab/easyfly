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
#ifndef EASYFLY_DESKTOP__OVERVIEW_TASK_HPP_
#define EASYFLY_DESKTOP__OVERVIEW_TASK_HPP_
#include <QString>
#include <array>
#include <optional>
#include "easyfly_desktop/mission_geometry.hpp"
#include "easyfly_desktop/source.hpp"
namespace easyfly_desktop
{
struct OverviewTaskOverlay
{
  bool bound{false}, fresh{false};
  QString task_name, goal_id, state, summary, reason, frame{"odom"};
  DraftGeometry geometry;
  int current_step{-1}, completed_steps{0}, total_steps{0};
  std::optional<std::array<double, 3>> current_target;
};
OverviewTaskOverlay overview_task_overlay(const bridge::UiSnapshot & snapshot, double now);
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__OVERVIEW_TASK_HPP_
