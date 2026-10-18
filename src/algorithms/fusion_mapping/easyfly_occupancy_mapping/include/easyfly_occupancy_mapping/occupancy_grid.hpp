// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace easyfly::occupancy
{
struct GridGeometry
{
  std::uint32_t width{0}, height{0};
  double resolution{0.0}, origin_x{0.0}, origin_y{0.0};
};
// Dimensions/resolution must be finite, positive and representable, as admitted by caller.
inline GridGeometry makeGridGeometry(double width_m, double height_m, double resolution)
{
  return {static_cast<uint32_t>(width_m / resolution),
    static_cast<uint32_t>(height_m / resolution), resolution, -width_m / 2.0, -height_m / 2.0};
}

// Finite boxes and bounded cell conversions; output must have width*height elements.
inline void rasterizeClusterBoxes(
  const std::vector<std::array<double, 6>> & boxes, const GridGeometry & geometry,
  double inf, std::vector<int8_t> & output)
{
  for (const auto & b : boxes) {
    const int c1 = std::max(
      0, static_cast<int>(std::floor(
        (b[0] - inf -
        geometry.origin_x) / geometry.resolution)));
    const int c2 = std::min(
      static_cast<int>(geometry.width) - 1,
      static_cast<int>(std::floor(
        (b[3] + inf -
        geometry.origin_x) / geometry.resolution)));
    const int r1 = std::max(
      0, static_cast<int>(std::floor(
        (b[1] - inf -
        geometry.origin_y) / geometry.resolution)));
    const int r2 = std::min(
      static_cast<int>(geometry.height) - 1,
      static_cast<int>(std::floor(
        (b[4] + inf -
        geometry.origin_y) / geometry.resolution)));
    for (int r = r1; r <= r2; ++r) {
      for (int c = c1; c <= c2; ++c) {
        output[r * geometry.width + c] = 100;
      }
    }
  }
}

// Count must refer to the appended tail and be divisible by the nonzero footprint size.
// All indices and values have already been admitted by the caller.
template<typename Points>
void appendPredictionFootprintBoxes(
  const Points & points, std::size_t published_prediction_points, std::size_t footprint,
  std::vector<std::array<double, 6>> & dynamic_boxes)
{
  for (std::size_t i = 0; i < published_prediction_points; i += footprint) {
    const auto start = points.size() - published_prediction_points + i;
    const auto & p = points[start];
    std::array<double, 6> b{p.x, p.y, p.z, p.x, p.y, p.z};
    for (std::size_t j = 1; j < footprint; ++j) {
      const auto & v = points[start + j];
      b[0] = std::min(b[0], static_cast<double>(v.x)); b[1] = std::min(
        b[1],
        static_cast<double>(v.y));
      b[3] = std::max(b[3], static_cast<double>(v.x)); b[4] = std::max(
        b[4],
        static_cast<double>(v.y));
    }
    dynamic_boxes.push_back(b);
  }
}

class GridPersistence
{
public:
  void reset()
  {
    grid_persist_.clear();
    grid_miss_.clear();
  }

  void apply(std::vector<int8_t> & output, int persist_frames)
  {
    const size_t n = output.size();
    // 持久化融合 (两段式): 最近 persist_frames 帧内见过 → 100, 超时 → 0。
    // 不用数值衰减: 衰减值会在 A* 占据阈值 (>=50) 边缘抖动 — 测试目标格
    // 采样时 55(占据)、规划执行时 48(空闲), 同一障碍时有时无。
    // 历史尺寸不符则重置。
    if (grid_persist_.size() != n || grid_miss_.size() != n) {
      grid_persist_.assign(n, 0);
      grid_miss_.assign(n, 0);
    }
    for (size_t i = 0; i < n; ++i) {
      if (output[i] == 100) {
        grid_persist_[i] = 100;
        grid_miss_[i] = 0;
      } else if (grid_persist_[i] > 0) {
        if (++grid_miss_[i] > static_cast<uint32_t>(
            std::max(0, persist_frames)))
        {
          grid_persist_[i] = 0;
          grid_miss_[i] = 0;
        }
        output[i] = grid_persist_[i];
      }
    }
  }

private:
  std::vector<int8_t> grid_persist_;
  std::vector<uint32_t> grid_miss_;
};
}  // namespace easyfly::occupancy
