// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>

namespace easyfly_semantic_map
{

// Select only physically possible ground samples for the RANSAC fit. This
// does not filter the obstacle cloud, change the tracker acceptance rules,
// or change the full-frame denominator used for the inlier confidence ratio.
inline bool isGroundFitSample(double point_z, double vehicle_z, double min_clearance)
{
  return std::isfinite(point_z) && std::isfinite(vehicle_z) &&
         std::isfinite(min_clearance) && min_clearance >= 0.0 &&
         point_z <= vehicle_z - min_clearance;
}

}  // namespace easyfly_semantic_map
