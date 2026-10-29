// Copyright 2026 EasyFly Project
//
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
#pragma once
#include <cmath>
#include <easyfly_msgs/action/navigate_to.hpp>
namespace easyfly_global_planner
{
// NaN target_z is the documented "keep current altitude" sentinel.  The
// other coordinates and max_speed must be finite before they enter planning;
// finite non-positive max_speed remains valid and is replaced by the default
// speed in navigateLoop().
inline bool validNavigateGoal(const easyfly_msgs::action::NavigateTo::Goal & goal)
{
  return std::isfinite(goal.target_x) && std::isfinite(goal.target_y) &&
         !std::isinf(goal.target_z) && std::isfinite(goal.max_speed);
}
}  // namespace easyfly_global_planner
