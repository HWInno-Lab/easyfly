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
#include <algorithm>
#include <cmath>
namespace easyfly_global_planner
{
// A waypoint is handed to flight_control before the planner advances to the
// next A* segment.  Keeping this below half of the default 0.2m grid cell
// prevents a child action from cutting a corner through the inflated AABB.
constexpr double kDefaultSegmentAcceptance{0.10};
constexpr double kMaxSegmentAcceptance{0.10};
inline double normalizeSegmentAcceptance(const double configured)
{
  if (!std::isfinite(configured) || configured <= 0.0) {
    return kDefaultSegmentAcceptance;
  }
  return std::min(configured, kMaxSegmentAcceptance);
}
}  // namespace easyfly_global_planner
