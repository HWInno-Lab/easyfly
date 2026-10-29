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
