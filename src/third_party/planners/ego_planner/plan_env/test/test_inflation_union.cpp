// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <set>
#include <tuple>
#include "plan_env/inflation_union.h"

namespace
{
using Voxel = std::tuple<int, int, int>;
std::set<Voxel> reference(const std::vector<plan_env::InflationBox> & boxes)
{
  std::set<Voxel> result;
  for (const auto & box : boxes) {
    for (int x = box.low[0]; x <= box.high[0]; ++x) {
      for (int y = box.low[1]; y <= box.high[1]; ++y) {
        for (int z = box.low[2]; z <= box.high[2]; ++z) {result.emplace(x, y, z);}
      }
    }
  }
  return result;
}
std::set<Voxel> fast(const std::vector<plan_env::InflationBox> & boxes)
{
  std::set<Voxel> result;
  EXPECT_TRUE(plan_env::visitInflationUnion(boxes,
      [&](int x, int y, int z) {EXPECT_TRUE(result.emplace(x, y, z).second);}));
  return result;
}
TEST(InflationUnion, OverlappingAnisotropicBoxesMatchBruteForceExactly)
{
  std::mt19937 random(2026);
  std::vector<plan_env::InflationBox> boxes;
  for (int i = 0; i < 100; ++i) {
    const int x = random() % 25, y = random() % 30, z = random() % 12;
    boxes.push_back({{x, y, z}, {x + 16, y + 16, z + 8}});
  }
  EXPECT_EQ(fast(boxes), reference(boxes));
}
TEST(InflationUnion, SingleVoxelClippedBordersAndDisjointRegions)
{
  const std::vector<plan_env::InflationBox> boxes{
    {{0, 0, 0}, {0, 0, 0}}, {{0, 4, 0}, {2, 6, 2}}, {{18, 19, 3}, {20, 20, 4}}};
  EXPECT_EQ(fast(boxes), reference(boxes));
  EXPECT_TRUE(fast({}).empty());
}
TEST(InflationUnion, ThousandsOfCoincidentSeedsDoNotLoseOrDuplicateVoxels)
{
  const std::vector<plan_env::InflationBox> boxes(6000, {{5, 6, 7}, {21, 22, 15}});
  EXPECT_EQ(fast(boxes), reference({boxes.front()}));
}
TEST(InflationUnion, InvalidOrExcessiveWorkCannotPartiallyMark)
{
  int marks = 0;
  const auto visit = [&](int, int, int) {++marks;};
  EXPECT_FALSE(plan_env::visitInflationUnion({{{-1, 0, 0}, {2, 2, 2}}}, visit));
  EXPECT_FALSE(plan_env::visitInflationUnion({{{2, 0, 0}, {1, 2, 2}}}, visit));
  EXPECT_FALSE(plan_env::visitInflationUnion({{{0, 0, 0}, {100000, 100000, 100000}}}, visit));
  EXPECT_EQ(marks, 0);
}
TEST(InflationUnion, PhysicalEndpointBoxesConservativelyContainOldFloatKernel)
{
  constexpr double resolution = 0.1;
  const std::array<double, 3> origin{{-40.0, -12.5, 0.5}};
  const std::array<int, 3> dimensions{{800, 250, 50}};
  const std::array<int, 3> steps{{8, 8, 4}};
  for (double offset : {-1e-7, 0.0, 1e-7}) {
    const std::array<double, 3> point{{2.3 + offset, -1.0 + offset, 1.5 + offset}};
    plan_env::InflationBox box;
    for (int axis = 0; axis < 3; ++axis) {
      box.low[axis] = std::max(0, static_cast<int>(std::floor(
          (point[axis] - steps[axis] * resolution - origin[axis]) / resolution)));
      box.high[axis] = std::min(dimensions[axis] - 1, static_cast<int>(std::floor(
          (point[axis] + steps[axis] * resolution - origin[axis]) / resolution)));
    }
    const auto actual = fast({box});
    for (int x = -8; x <= 8; ++x) {
      for (int y = -8; y <= 8; ++y) {
        for (int z = -4; z <= 4; ++z) {
          const std::array<int, 3> delta{{x, y, z}};
          std::array<int, 3> index;
          for (int axis = 0; axis < 3; ++axis) {
            index[axis] = static_cast<int>(std::floor(
              (point[axis] + delta[axis] * resolution - origin[axis]) / resolution));
          }
          EXPECT_EQ(actual.count({index[0], index[1], index[2]}), 1u);
        }
      }
    }
  }
}
}  // namespace
