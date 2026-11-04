// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_ground_segmentation/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
#include <Eigen/Geometry>
#include "easyfly_ground_segmentation/ground_plane_tracker.hpp"

namespace easyfly::ground
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
}  // namespace easyfly::ground
