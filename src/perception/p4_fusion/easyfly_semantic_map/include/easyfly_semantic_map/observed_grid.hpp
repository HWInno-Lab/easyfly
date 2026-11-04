// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
//
#pragma once
#include <vector>
#include "easyfly_occupancy_mapping/observed_grid.hpp"
#include "easyfly_semantic_map/ground_plane_tracker.hpp"
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
}  // namespace easyfly_semantic_map
