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
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace easyfly::coverage {
struct SearchPoint {double x, y, z; double completed_area;};
struct SearchBounds {
  double min_x{-50}, max_x{50}, min_y{-50}, max_y{50}, min_z{.5}, max_z{2.5};
};
inline std::vector<SearchPoint> searchRoute(double cx, double cy, double width, double height,
  double altitude, double spacing, const SearchBounds & bounds) {
  const double values[]{cx, cy, width, height, altitude, spacing,
    bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y, bounds.min_z, bounds.max_z};
  for (auto v : values) if (!std::isfinite(v)) throw std::invalid_argument("nonfinite search geometry");
  if (bounds.min_x >= bounds.max_x || bounds.min_y >= bounds.max_y || bounds.min_z > bounds.max_z ||
    width < .1 || width > 100 || height < .1 || height > 100 || spacing < .1 || spacing > 20 ||
    cx - width / 2 < bounds.min_x || cx + width / 2 > bounds.max_x ||
    cy - height / 2 < bounds.min_y || cy + height / 2 > bounds.max_y ||
    altitude < bounds.min_z || altitude > bounds.max_z)
    throw std::invalid_argument("search rectangle outside configured flight bounds");
  const auto strips = static_cast<size_t>(std::ceil(height / spacing));
  if (!strips || strips > 256) throw std::invalid_argument("search exceeds 256 strips");
  const auto band = height / strips;
  std::vector<SearchPoint> result;
  for (size_t i = 0; i < strips; ++i) {
    const auto y = cy - height / 2 + (i + .5) * band;
    const auto start = cx + (i % 2 ? width / 2 : -width / 2);
    const auto end = cx + (i % 2 ? -width / 2 : width / 2);
    // Transit/approach cannot be counted as scanned area.
    result.push_back({start, y, altitude, width * band * i});
    result.push_back({end, y, altitude, width * band * (i + 1)});
  }
  return result;
}
}  // namespace easyfly::coverage
