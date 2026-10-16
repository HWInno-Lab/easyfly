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

#include <gtest/gtest.h>
#include <limits>
#include "easyfly_semantic_map/observed_grid.hpp"

namespace
{
struct Point {double x, y, z;};
constexpr std::size_t kSize = 100;
std::size_t at(double x, double y)
{
  return std::size_t(std::floor((y + 10.) / .2)) * kSize +
         std::size_t(std::floor((x + 10.) / .2));
}
bool draw(const std::vector<Point> & points, std::vector<int8_t> & grid)
{
  return easyfly_semantic_map::rasterizeObservedFootprints(
    points, kSize, kSize, .2, -10., -10., .75, grid);
}

TEST(ObservedGrid, ConnectedRoomWallsDoNotFillTheirInterior)
{
  std::vector<Point> points;
  // A connected 12m U-shaped cluster exceeds the semantic size limit.
  for (int i = -60; i <= 60; ++i) {
    points.push_back({i * .1, -4., 1.});
    points.push_back({-6., i / 15., 1.});
    points.push_back({6., i / 15., 1.});
  }
  std::vector<int8_t> grid(kSize * kSize, 0);
  ASSERT_TRUE(draw(points, grid));
  EXPECT_EQ(grid[at(0., 0.)], 0);
  EXPECT_EQ(grid[at(1.5, 0.)], 0);
  EXPECT_EQ(grid[at(0., -3.4)], 100);
  EXPECT_EQ(grid[at(-5.4, 0.)], 100);
  EXPECT_EQ(grid[at(5.4, 0.)], 100);
}

TEST(ObservedGrid, EverySampleRetainsFullMarginIncludingIsolatedPoints)
{
  std::vector<Point> points{{.037, .063, 1.}};
  std::vector<int8_t> grid(kSize * kSize, 0);
  ASSERT_TRUE(draw(points, grid));
  for (double x : {-.70, 0., .70}) {
    for (double y : {-.70, 0., .70}) {
      EXPECT_EQ(grid[at(points[0].x + x, points[0].y + y)], 100);
    }
  }
  EXPECT_EQ(grid[at(1.1, 0.)], 0);
}

TEST(ObservedGrid, ClipsOverlappingOutsideSamplesAndPreservesExistingEvidence)
{
  std::vector<int8_t> grid(kSize * kSize, 0);
  grid[at(0., 0.)] = 100;
  ASSERT_TRUE(draw({{-10.5, 0., 1.}, {1e300, 1e300, 1.}}, grid));
  EXPECT_EQ(grid[at(-9.95, 0.)], 100);
  EXPECT_EQ(grid[at(0., 0.)], 100);
  EXPECT_EQ(grid[at(5., 0.)], 0);
}

TEST(ObservedGrid, RejectsInvalidEvidenceBeforePublishingAFreeGrid)
{
  std::vector<int8_t> grid(kSize * kSize, 0);
  EXPECT_FALSE(draw({{std::numeric_limits<double>::quiet_NaN(), 0., 1.}}, grid));
  EXPECT_FALSE(
    easyfly_semantic_map::rasterizeObservedFootprints(
      std::vector<Point>{}, kSize, kSize, 0., -10., -10., .75, grid));
}
}  // namespace
