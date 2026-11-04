// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_trajectory_optimizer/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
#include "easyfly_trajectory_optimizer/grid.hpp"
#include <gcopter/trajectory.hpp>
#include <string>

namespace easyfly::trajectory
{
// Conservative simulation profile for the position/velocity-only controller.
// These tighten generation/admission; the 0.20 m tracking stop is unchanged.
constexpr double kTrackingAcceleration = .15;
constexpr double kTrackingJerk = .3;
struct Request
{
  Grid grid;
  Vec start, velocity, goal;
  double max_speed{.6}, max_acceleration{2.0}, max_jerk{8.0};
};
struct Plan
{
  Trajectory<5> trajectory;
  std::vector<Vec> route;
  std::vector<Box> boxes;
  double cost{0}, maximum_speed{0}, maximum_acceleration{0}, maximum_jerk{0};
};
bool collisionFree(
  const Grid & grid, const Trajectory<5> & trajectory,
  double from = 0., Steady::time_point deadline = Steady::time_point::max());
Plan optimize(const Request & request, Steady::time_point deadline);
void validate(
  const Request & request, Plan & plan, const Grid & current_grid,
  Steady::time_point deadline);
void writeRequest(const std::string & file, const Request & request);
Request readRequest(const std::string & file);
void writePlan(const std::string & file, const Plan & plan);
Plan readPlan(const std::string & file);
}  // namespace easyfly::trajectory
