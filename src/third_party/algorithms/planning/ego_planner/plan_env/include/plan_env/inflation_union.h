// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <new>
#include <vector>

namespace plan_env
{
struct InflationBox
{
  std::array<int, 3> low;
  std::array<int, 3> high;  // inclusive; already clipped to the map
};

// Exact union of axis-aligned voxel boxes. Eight difference-grid corners per
// box replace repeated writes of thousands of overlapping kernels. No seeds,
// inflation distance, occupied voxels or persistence frames are discarded.
// All validation/allocation happens before visiting any occupied voxel.
template<class Visitor>
bool visitInflationUnion(const std::vector<InflationBox> & boxes, Visitor visit)
{
  if (boxes.empty()) {return true;}
  if (boxes.size() > 1000000) {return false;}
  std::array<int, 3> low = boxes.front().low;
  std::array<int, 3> high = boxes.front().high;
  for (const auto & box : boxes) {
    for (int axis = 0; axis < 3; ++axis) {
      if (box.low[axis] < 0 || box.high[axis] < box.low[axis]) {return false;}
      low[axis] = std::min(low[axis], box.low[axis]);
      high[axis] = std::max(high[axis], box.high[axis]);
    }
  }
  constexpr std::size_t max_elements = 32U * 1024U * 1024U;
  std::array<std::size_t, 3> size;
  std::size_t count = 1;
  for (int axis = 0; axis < 3; ++axis) {
    size[axis] = static_cast<std::size_t>(high[axis]) - low[axis] + 2;
    if (size[axis] > max_elements / count) {return false;}
    count *= size[axis];
  }
  std::vector<std::int32_t> difference;
  try {difference.assign(count, 0);} catch (const std::bad_alloc &) {return false;}
  const std::size_t stride_x = size[1] * size[2];
  const std::size_t stride_y = size[2];
  const auto address = [&](std::size_t x, std::size_t y, std::size_t z) {
      return x * stride_x + y * stride_y + z;
    };
  for (const auto & box : boxes) {
    for (unsigned corner = 0; corner < 8; ++corner) {
      std::array<std::size_t, 3> index;
      int sign = 1;
      for (int axis = 0; axis < 3; ++axis) {
        const bool upper = (corner & (1U << axis)) != 0;
        index[axis] = static_cast<std::size_t>(upper ? box.high[axis] : box.low[axis]) -
          low[axis] + (upper ? 1 : 0);
        if (upper) {sign = -sign;}
      }
      difference[address(index[0], index[1], index[2])] += sign;
    }
  }
  for (std::size_t x = 1; x < size[0]; ++x) {
    for (std::size_t offset = 0; offset < stride_x; ++offset) {
      difference[x * stride_x + offset] += difference[(x - 1) * stride_x + offset];
    }
  }
  for (std::size_t x = 0; x < size[0]; ++x) {
    for (std::size_t y = 1; y < size[1]; ++y) {
      for (std::size_t z = 0; z < size[2]; ++z) {
        difference[address(x, y, z)] += difference[address(x, y - 1, z)];
      }
    }
  }
  for (std::size_t x = 0; x + 1 < size[0]; ++x) {
    for (std::size_t y = 0; y + 1 < size[1]; ++y) {
      std::int32_t overlap = 0;
      for (std::size_t z = 0; z + 1 < size[2]; ++z) {
        overlap += difference[address(x, y, z)];
        if (overlap > 0) {
          visit(static_cast<int>(x) + low[0], static_cast<int>(y) + low[1],
            static_cast<int>(z) + low[2]);
        }
      }
    }
  }
  return true;
}
}  // namespace plan_env
