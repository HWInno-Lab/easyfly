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
#include <cstddef>
#include <memory>
#include <string>
#include "easyfly_grid_planner_api/planner.hpp"

namespace easyfly_global_planner
{
struct AStarParams
{
  std::size_t max_cells{4000000};
  int unstick_cells{5};
};
std::unique_ptr<easyfly::grid_planning::GridPlanner> make_planner(
  const std::string & backend, const AStarParams & params = {});
}  // namespace easyfly_global_planner
