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
#include <cstdint>
#include <vector>

namespace easyfly_semantic_map
{

// Project all trusted nonground samples, independent of semantic clustering.
// Every cell touched by a square footprint is occupied. The caller adds the
// voxel half-width to the unchanged safety radius. A 2-D difference array
// bounds work by O(points + grid cells), rather than points * inflation area.
template<typename Points>
bool rasterizeObservedFootprints(
  const Points & points, std::size_t width, std::size_t height,
  double resolution, double origin_x, double origin_y, double radius,
  std::vector<int8_t> & output)
{
  if (!width || !height || width > 1000000 || height > 1000000 / width ||
    points.size() > 200000 || !std::isfinite(resolution) || resolution <= 0 ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    !std::isfinite(radius) || radius < 0 || output.size() != width * height)
  {
    return false;
  }
  const double xmax = origin_x + width * resolution;
  const double ymax = origin_y + height * resolution;
  if (!std::isfinite(xmax) || !std::isfinite(ymax)) {return false;}
  const auto stride = width + 1;
  std::vector<int32_t> delta(stride * (height + 1), 0);
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return false;
    }
    const double x1 = static_cast<double>(point.x) - radius,
      x2 = static_cast<double>(point.x) + radius;
    const double y1 = static_cast<double>(point.y) - radius,
      y2 = static_cast<double>(point.y) + radius;
    if (!std::isfinite(x1) || !std::isfinite(x2) ||
      !std::isfinite(y1) || !std::isfinite(y2)) {return false;}
    if (x2 < origin_x || x1 >= xmax || y2 < origin_y || y1 >= ymax) {continue;}
    // Clamp in floating point before integer conversion, including huge but
    // finite coordinates outside the map. Outside centers may still overlap it.
    const auto cell = [](double value, double origin, double res, std::size_t size) {
        return static_cast<std::size_t>(std::clamp(
                 std::floor((value - origin) / res), 0.0, static_cast<double>(size - 1)));
      };
    const auto c1 = cell(x1, origin_x, resolution, width);
    const auto c2 = cell(x2, origin_x, resolution, width);
    const auto r1 = cell(y1, origin_y, resolution, height);
    const auto r2 = cell(y2, origin_y, resolution, height);
    ++delta[r1 * stride + c1]; --delta[r1 * stride + c2 + 1];
    --delta[(r2 + 1) * stride + c1]; ++delta[(r2 + 1) * stride + c2 + 1];
  }
  for (std::size_t row = 0; row < height; ++row) {
    int32_t horizontal = 0;
    for (std::size_t col = 0; col < width; ++col) {
      const auto i = row * stride + col;
      horizontal += delta[i];
      delta[i] = horizontal + (row ? delta[i - stride] : 0);
      if (delta[i] > 0) {output[row * width + col] = 100;}
    }
  }
  return true;
}

}  // namespace easyfly_semantic_map
