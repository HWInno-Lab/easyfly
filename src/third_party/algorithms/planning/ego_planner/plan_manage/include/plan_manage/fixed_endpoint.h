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

#pragma once
#include <Eigen/Core>
#include <cmath>
namespace ego_planner
{
inline bool setFixedCubicEndpoint(
  Eigen::MatrixXd & points, double interval,
  const Eigen::Vector3d & target,
  const Eigen::Vector3d & velocity)
{
  if (points.rows() != 3 || points.cols() < 7 || !points.allFinite() ||
    !std::isfinite(interval) || interval <= 0.0 ||
    !target.allFinite() || !velocity.allFinite()) {return false;}
  const Eigen::Index end = points.cols();
  points.col(end - 3) = target - interval * velocity;
  points.col(end - 2) = target;
  points.col(end - 1) = target + interval * velocity;
  return points.allFinite();
}
}  // namespace ego_planner
