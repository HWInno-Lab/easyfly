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
