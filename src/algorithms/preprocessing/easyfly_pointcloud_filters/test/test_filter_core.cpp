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

// filter_core 单元测试: 距离滤波 + 体素降采样
#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "easyfly_pointcloud_filters/filter_core.hpp"

using easyfly::pointcloud_filters::Pt;
using easyfly::pointcloud_filters::rangeFilter;
using easyfly::pointcloud_filters::voxelDownsample;
using easyfly::pointcloud_filters::filterCloud;

TEST(RangeFilter, KeepsPointsInRange)
{
    std::vector<Pt> in = {
        {0.1f, 0, 0, 0},     // 盲区内 (0.1 < 0.3) → 剔除
        {1.0f, 0, 0, 10},    // 1m → 保留
        {0, 0, 5.0f, 20},    // 5m → 保留
        {30.0f, 0, 0, 0},    // 超远界 → 剔除
        {0.31f, 0, 0, 5},    // 恰好在 min 边界 → 保留
    };
    auto out = rangeFilter(in, 0.3f, 25.0f);
    ASSERT_EQ(out.size(), 3u);
    EXPECT_FLOAT_EQ(out[0].x, 1.0f);
    EXPECT_FLOAT_EQ(out[1].z, 5.0f);
    EXPECT_FLOAT_EQ(out[2].x, 0.31f);
}

TEST(RangeFilter, EmptyInput)
{
    EXPECT_TRUE(rangeFilter({}, 0.3f, 25.0f).empty());
}

TEST(VoxelDownsample, MergesPointsInSameVoxel)
{
    // 体素 1.0m: (0.1,0,0) 与 (0.2,0,0) 同体素 → 只留 1 个;
    // (1.5,0,0) 在 x=1 体素 → 共 2 个输出
    std::vector<Pt> in = {{0.1f, 0, 0, 1}, {0.2f, 0, 0, 2}, {1.5f, 0, 0, 3}};
    auto out = voxelDownsample(in, 1.0f);
    EXPECT_EQ(out.size(), 2u);
    // first-wins: 同体素保留先到的 (intensity=1)
    bool has_first = false;
    for (const auto & p : out)
        if (p.intensity == 1.0f) has_first = true;
    EXPECT_TRUE(has_first);
}

TEST(VoxelDownsample, NonPositiveSizeIsNoop)
{
    std::vector<Pt> in = {{0, 0, 0, 0}, {1, 1, 1, 1}};
    EXPECT_EQ(voxelDownsample(in, 0.0f).size(), 2u);
    EXPECT_EQ(voxelDownsample(in, -1.0f).size(), 2u);
}

TEST(FilterChain, ReducesSyntheticCloud)
{
    // 10000 个 0~10m 随机点, 体素 0.5m → 输出应远小于输入且非零
    std::vector<Pt> in;
    in.reserve(10000);
    for (int i = 0; i < 10000; ++i) {
        const float f = static_cast<float>(i) * 0.001f;
        in.push_back({10.0f * std::sin(f * 12.9898f),
                      10.0f * std::sin(f * 78.233f),
                      5.0f * std::sin(f * 37.719f), static_cast<float>(i % 255)});
    }
    auto out = filterCloud(in, 0.5f, 9.0f, 0.5f);
    EXPECT_GT(out.size(), 100u);
    EXPECT_LT(out.size(), in.size() / 2u);
}

TEST(FilterMigration, InclusiveRangePreservesAllPointAttributes)
{
  const std::vector<Pt> input = {{.3f, 0, 0, 11}, {-25, 0, 0, 12},
    {.29f, 0, 0, 13}, {26, 0, 0, 14}};
  const auto output = rangeFilter(input, .3f, 25);
  ASSERT_EQ(output.size(), 2u);
  EXPECT_FLOAT_EQ(output[0].x, .3f);
  EXPECT_FLOAT_EQ(output[0].intensity, 11);
  EXPECT_FLOAT_EQ(output[1].x, -25);
  EXPECT_FLOAT_EQ(output[1].y, 0);
  EXPECT_FLOAT_EQ(output[1].z, 0);
  EXPECT_FLOAT_EQ(output[1].intensity, 12);
}

TEST(FilterMigration, SignedVoxelFirstWinsUsesInputOrderAndKeepsFullPoint)
{
  const std::vector<Pt> input = {{.11f, .12f, .13f, 11}, {.15f, .16f, .17f, 22},
    {-.01f, -.02f, -.03f, 33}, {-.1f, -.2f, -.3f, 44}, {1.1f, 0, 0, 55}};
  for (const bool reverse : {false, true}) {
    auto points = input;
    if (reverse) {points.assign(input.rbegin(), input.rend());}
    const auto output = voxelDownsample(points, 1);
    ASSERT_EQ(output.size(), 3u);
    bool positive = false, negative = false, far = false;
    for (const auto & point : output) {
      const auto & expected = point.x >= 1 ? input[4] :
        point.x < 0 ? input[reverse ? 3 : 2] : input[reverse ? 1 : 0];
      EXPECT_FLOAT_EQ(point.x, expected.x);
      EXPECT_FLOAT_EQ(point.y, expected.y);
      EXPECT_FLOAT_EQ(point.z, expected.z);
      EXPECT_FLOAT_EQ(point.intensity, expected.intensity);
      if (point.x >= 1) {far = true;} else if (point.x < 0) {negative = true;} else {positive = true;}
    }
    EXPECT_TRUE(positive && negative && far);
  }
}
