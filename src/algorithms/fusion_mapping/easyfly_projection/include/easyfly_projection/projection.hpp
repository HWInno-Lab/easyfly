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
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace easyfly::projection
{
struct Box {
  float bbox_center_x, bbox_center_y, bbox_width, bbox_height;
};
// Borrowed host-endian float32 depth bytes; stride is in bytes, not pixels.
struct DepthView {
  const std::uint8_t * data;
  std::size_t byte_count;
  int width, height;
  std::size_t row_stride;
};
struct Options {
  double min_depth{0.3}, max_depth{9.5}, min_valid_ratio{0.1};
};
inline float medianDepth(const Box & t, const DepthView & depth, const Options & options = {})
{
        const int W = static_cast<int>(depth.width);
        const int H = static_cast<int>(depth.height);

        // bbox 归一化 → 像素 (clamp)
        int u1 = std::clamp(static_cast<int>((t.bbox_center_x - t.bbox_width / 2) * W), 0, W - 1);
        int u2 = std::clamp(static_cast<int>((t.bbox_center_x + t.bbox_width / 2) * W), 0, W - 1);
        int v1 = std::clamp(static_cast<int>((t.bbox_center_y - t.bbox_height / 2) * H), 0, H - 1);
        int v2 = std::clamp(static_cast<int>((t.bbox_center_y + t.bbox_height / 2) * H), 0, H - 1);
        if (u2 <= u1 || v2 <= v1) return -1.0f;

        const float * depth_ptr = reinterpret_cast<const float *>(depth.data);
        const size_t row_step = depth.row_stride / sizeof(float);

        // 下采样采集 (步长按框大小自适应, 最多 ~200 采样点)
        std::vector<float> vals;
        vals.reserve(200);
        const int du = std::max(1, (u2 - u1) / 14);
        const int dv = std::max(1, (v2 - v1) / 14);
        for (int v = v1; v <= v2; v += dv)
            for (int u = u1; u <= u2; u += du) {
                const float d = depth_ptr[v * row_step + u];
                if (std::isfinite(d) && d > options.min_depth && d < options.max_depth)
                    vals.push_back(d);
            }

        const size_t total = static_cast<size_t>((u2 - u1) / du + 1) * ((v2 - v1) / dv + 1);
        if (vals.empty() || vals.size() < total * options.min_valid_ratio) return -1.0f;

        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
        return vals[vals.size() / 2];
}
inline std::array<double, 3> pinhole(
  int u, int v, float depth, double fx, double fy, double cx, double cy)
{
  return {(u - cx) * depth / fx, (v - cy) * depth / fy, depth};
}
}  // namespace easyfly::projection
