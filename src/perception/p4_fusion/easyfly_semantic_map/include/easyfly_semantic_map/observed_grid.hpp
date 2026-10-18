// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "easyfly_semantic_map/ground_plane_tracker.hpp"
#include "easyfly_occupancy_mapping/observed_grid.hpp"
namespace easyfly_semantic_map
{
// Preserve the original concrete-plane signature for existing callers.
template<typename Points>
bool rasterizeObservedFootprints(
  const Points & points, std::size_t width, std::size_t height,
  double resolution, double origin_x, double origin_y, double radius,
  std::vector<int8_t> & output,
  const GroundPlaneModel * ground = nullptr, double height_limit = 0.0,
  double voxel_half_width = 0.0)
{
  return easyfly::occupancy::rasterizeObservedFootprints(
    points, width, height, resolution, origin_x, origin_y, radius,
    output, ground, height_limit, voxel_half_width);
}
}
