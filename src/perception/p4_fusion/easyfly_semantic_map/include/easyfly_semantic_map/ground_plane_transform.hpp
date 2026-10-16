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
#include <Eigen/Geometry>
#include "easyfly_semantic_map/ground_plane_tracker.hpp"

namespace easyfly_semantic_map
{
// n_G . (R_GW p_W + t_GW) + d_G = 0. Transport a fixed plane;
// this never fits/fuses an armed candidate or changes physical ground.
inline GroundPlaneModel groundPlaneInWorld(
  const GroundPlaneModel & ground, const Eigen::Isometry3d & world_to_ground)
{
  const Eigen::Vector3d normal(ground.normal[0], ground.normal[1], ground.normal[2]);
  const Eigen::Vector3d transformed = world_to_ground.linear().transpose() * normal;
  GroundPlaneModel world;
  world.normal = {transformed.x(), transformed.y(), transformed.z()};
  world.offset = ground.offset + normal.dot(world_to_ground.translation());
  return world;
}
inline bool groundBasisContinuous(
  const Eigen::Isometry3d & previous, const Eigen::Isometry3d & current)
{
  return previous.matrix().allFinite() && current.matrix().allFinite() &&
         (current.translation() - previous.translation()).norm() <= 0.25 &&
         std::abs(Eigen::AngleAxisd(previous.linear().transpose() * current.linear()).angle()) <=
         0.10;
}
inline bool groundBasisStampFresh(double now, double stamp, double timeout)
{
  return std::isfinite(now) && std::isfinite(stamp) && std::isfinite(timeout) &&
         stamp > 0 && timeout > 0 && now - stamp >= -0.2 && now - stamp <= timeout;
}
}  // namespace easyfly_semantic_map
