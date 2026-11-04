// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_occupancy_mapping/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once

#include <algorithm>
#include <cmath>

namespace easyfly::occupancy
{

// Dynamic N2 traces showed about 0.13m consumed by grid discretization,
// box/center jitter, and flight-control tracking beyond the nominal 0.5m
// obstacle margin.  Add the smallest evidenced 0.2m map-side margin while
// keeping the acceptance check at obstacle AABB + 0.5m.
constexpr double kDefaultGridInflationRadius{0.70};
constexpr double kMinimumGridInflationRadius{0.70};

inline double normalizeGridInflationRadius(const double configured)
{
  if (!std::isfinite(configured) || configured < 0.0) {
    return kDefaultGridInflationRadius;
  }
  return std::max(configured, kMinimumGridInflationRadius);
}

}  // namespace easyfly::occupancy
