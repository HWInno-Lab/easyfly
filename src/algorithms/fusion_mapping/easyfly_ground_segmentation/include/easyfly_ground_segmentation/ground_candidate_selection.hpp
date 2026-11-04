// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_ground_segmentation/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once

#include <cmath>

namespace easyfly::ground
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

}  // namespace easyfly::ground
