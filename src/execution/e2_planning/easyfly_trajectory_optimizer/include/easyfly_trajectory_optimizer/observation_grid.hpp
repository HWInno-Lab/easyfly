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
#include "easyfly_trajectory_optimizer/grid.hpp"

namespace easyfly::trajectory
{
inline void reserveUnknown(Grid & grid, double radius)
{
  if (!grid.valid() || !std::isfinite(radius) || radius < .35 || radius > 1.) {
    throw std::invalid_argument("unknown envelope");
  }
  // Separable box dilation is conservative for a spherical envelope, linear
  // in cell count, and never clears an occupied cell. Include the full unknown
  // cube and outward cell rounding, not just its centre.
  const int extent = static_cast<int>(std::ceil(radius / grid.resolution)) + 1;
  std::vector<uint8_t> mask(grid.data.size()), next(mask.size());
  for (size_t i = 0; i < mask.size(); ++i) {
    mask[i] = grid.data[i] == -1;
  }
  const int strides[]{1, grid.size.x(), grid.size.x() * grid.size.y()};
  for (int axis = 0; axis < 3; ++axis) {
    const int stride = strides[axis], length = grid.size[axis];
    for (size_t index = 0; index < mask.size(); ++index) {
      if ((index / stride) % length != 0) {continue;}
      int unknown = 0;
      for (int k = 0; k <= std::min(extent, length - 1); ++k) {
        unknown += mask[index + k * stride];
      }
      for (int k = 0; k < length; ++k) {
        next[index + k * stride] = unknown > 0;
        if (k - extent >= 0) {unknown -= mask[index + (k - extent) * stride];}
        if (k + extent + 1 < length) {unknown += mask[index + (k + extent + 1) * stride];}
      }
    }
    mask.swap(next);
  }
  for (size_t i = 0; i < mask.size(); ++i) {
    if (mask[i] && grid.data[i] == 0) {
      grid.data[i] = -1;
    }
  }
}
// Per-scan evidence, not a time-based occupancy expiry. A hit always wins over
// any number of misses from the same scan; old and duplicate scans are rejected
// by the source-time mapper before reaching this accumulator.
class ObservationGrid
{
public:
  explicit ObservationGrid(Grid & grid, unsigned clear_scans = 0)
  : grid_(grid), clear_scans_(clear_scans), misses_(grid.data.size(), 0),
    free_(grid.data.size(), 0), hit_(grid.data.size(), 0)
  {
    if (!grid.valid() || (clear_scans != 0 && (clear_scans < 3 || clear_scans > 20))) {
      throw std::invalid_argument("observation grid limits");
    }
  }
  void begin()
  {
    std::fill(free_.begin(), free_.end(), 0);
    std::fill(hit_.begin(), hit_.end(), 0);
  }
  void observeFree(const Cell & cell) {if (grid_.inside(cell)) {free_[grid_.index(cell)] = 1;}}
  void observeHit(const Cell & cell) {if (grid_.inside(cell)) {hit_[grid_.index(cell)] = 1;}}
  void commit()
  {
    for (size_t i = 0; i < grid_.data.size(); ++i) {
      if (hit_[i]) {grid_.data[i] = 100; misses_[i] = 0;} else if (free_[i]) {
        if (grid_.data[i] == -1) {
          grid_.data[i] = 0;
        } else if (grid_.data[i] == 100 && clear_scans_ && ++misses_[i] >= clear_scans_) {
          grid_.data[i] = 0; misses_[i] = 0;
        }
      } else {misses_[i] = 0;}
    }
  }
  void reset()
  {
    std::fill(grid_.data.begin(), grid_.data.end(), -1);
    std::fill(misses_.begin(), misses_.end(), 0); begin();
  }
  // A lattice-aligned sliding window preserves world coordinates, accumulated
  // evidence and conservative hit history in the overlap. New cells are unknown.
  // This is a revision, not a coordinate-frame reset/generation change.
  void shift(const Cell & offset)
  {
    if (offset.isZero()) {return;}
    std::vector<int8_t> cells(grid_.data.size(), -1);
    std::vector<uint8_t> misses(misses_.size(), 0);
    for (size_t i = 0; i < cells.size(); ++i) {
      const Cell old = grid_.cell(static_cast<int>(i)) + offset;
      if (grid_.inside(old)) {
        cells[i] = grid_.data[grid_.index(old)]; misses[i] = misses_[grid_.index(old)];
      }
    }
    grid_.origin += offset.cast<double>() * grid_.resolution;
    grid_.data.swap(cells); misses_.swap(misses); begin();
  }

private:
  Grid & grid_;
  unsigned clear_scans_;
  std::vector<uint8_t> misses_, free_, hit_;
};
}  // namespace easyfly::trajectory
