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

// EasyFly P1: 点云滤波核心逻辑 (纯函数, 无 ROS/PCL 依赖, 便于单元测试)
//
// 流程: 距离滤波 (球域) → 体素降采样 (每体素保留首个点)
// 输入点通常来自 livox_frame, 由节点层负责 TF 变换与消息封装。
#ifndef EASYFLY_POINTCLOUD_FILTERS__FILTER_CORE_HPP_
#define EASYFLY_POINTCLOUD_FILTERS__FILTER_CORE_HPP_

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace easyfly::pointcloud_filters
{

struct Pt
{
  float x, y, z, intensity;
};

/// 距离滤波: 保留 range_min <= |p| <= range_max 的点 (盲区/远噪剔除)
inline std::vector<Pt> rangeFilter(
  const std::vector<Pt> & in, float range_min, float range_max)
{
  std::vector<Pt> out;
  out.reserve(in.size());
  const float r2min = range_min * range_min;
  const float r2max = range_max * range_max;
  for (const auto & p : in) {
    const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (r2 >= r2min && r2 <= r2max) out.push_back(p);
  }
  return out;
}

/// 体素降采样: 空间栅格化, 每个体素只保留第一个落入的点 (first-wins, 确定性)
/// voxel_size <= 0 时原样返回
inline std::vector<Pt> voxelDownsample(const std::vector<Pt> & in, float voxel_size)
{
  if (voxel_size <= 0.0f || in.empty()) return in;

  const float inv = 1.0f / voxel_size;
  std::unordered_map<uint64_t, const Pt *> grid;
  grid.reserve(in.size() * 2);

  for (const auto & p : in) {
    const int32_t ix = static_cast<int32_t>(std::floor(p.x * inv));
    const int32_t iy = static_cast<int32_t>(std::floor(p.y * inv));
    const int32_t iz = static_cast<int32_t>(std::floor(p.z * inv));
    // 3D 网格键打包 (21+21+21 位, 坐标 ±1000m 内无碰撞)
    const uint64_t key =
      (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 42) ^
      (static_cast<uint64_t>(static_cast<uint32_t>(iy)) << 21) ^
      static_cast<uint64_t>(static_cast<uint32_t>(iz));
    grid.emplace(key, &p);  // 已存在则保留第一个
  }

  std::vector<Pt> out;
  out.reserve(grid.size());
  for (const auto & kv : grid) out.push_back(*kv.second);
  return out;
}

/// 完整滤波链: 距离 → 体素
inline std::vector<Pt> filterCloud(
  const std::vector<Pt> & in, float range_min, float range_max, float voxel_size)
{
  return voxelDownsample(rangeFilter(in, range_min, range_max), voxel_size);
}

}  // namespace easyfly::pointcloud_filters

#endif  // EASYFLY_POINTCLOUD_FILTERS__FILTER_CORE_HPP_
