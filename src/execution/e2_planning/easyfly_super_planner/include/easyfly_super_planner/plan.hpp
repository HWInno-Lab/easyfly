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
#include <easyfly_trajectory_optimizer/grid.hpp>
#include <data_structure/base/trajectory.h>
#include <string>

namespace easyfly::superapi
{
using Trajectory = geometry_utils::Trajectory;
using Piece = geometry_utils::Piece;
using easyfly::trajectory::Grid;
using easyfly::trajectory::Vec;
using easyfly::trajectory::Box;
using easyfly::trajectory::Steady;
using easyfly::trajectory::budget;
// Tracking budget for the current position/velocity-only PX4 command chain.
constexpr double kTrackingAcceleration = .15;
constexpr double kTrackingJerk = .3;
struct Request
{
  Grid grid;
  Vec start, velocity, goal;
  double yaw{0};
  bool execute_committed{false};
  double max_speed{.6}, max_acceleration{2.0}, max_jerk{8.0};
};
struct Plan
{
  Trajectory trajectory;
  double backup_start{-1.};
  bool partial{false};
  std::vector<Vec> route;
  std::vector<Box> boxes;
  double cost{0}, maximum_speed{0}, maximum_acceleration{0}, maximum_jerk{0};
};
// Expand unobserved cells by the physical/tracking envelope before admission.
void reserveUnknown(Grid & grid, double radius);
bool collisionFree(
  const Grid & grid, const Trajectory & trajectory,
  double from = 0., Steady::time_point deadline = Steady::time_point::max());
Plan optimize(const Request & request, Steady::time_point deadline);
void validate(
  const Request & request, Plan & plan, const Grid & current_grid,
  Steady::time_point deadline);
void writeRequest(const std::string & file, const Request & request);
Request readRequest(const std::string & file);
void writePlan(const std::string & file, const Plan & plan);
Plan readPlan(const std::string & file);
}  // namespace easyfly::superapi
