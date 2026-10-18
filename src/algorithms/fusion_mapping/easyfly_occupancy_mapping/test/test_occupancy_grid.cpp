// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_occupancy_mapping/occupancy_grid.hpp"

namespace easyfly::occupancy
{
namespace
{
TEST(OccupancyGrid, GeometryKeepsTruncationAndWorldOrigin)
{
  const auto g = makeGridGeometry(4.1, 3.1, .2);
  EXPECT_EQ(g.width, 20U);
  EXPECT_EQ(g.height, 15U);
  EXPECT_DOUBLE_EQ(g.origin_x, -2.05);
  EXPECT_DOUBLE_EQ(g.origin_y, -1.55);
  EXPECT_DOUBLE_EQ(g.resolution, .2);
}
TEST(OccupancyGrid, InflatedBoxesClipAtMapEdgesAndPreserveExistingEvidence)
{
  const auto g = makeGridGeometry(10., 10., 1.);
  std::vector<int8_t> cells(100, 0);
  cells[0] = 100;
  rasterizeClusterBoxes({{-.1, -.1, 0., .1, .1, 1.}, {4.9, 4.9, 0., 6., 6., 1.},
    {20., 20., 0., 21., 21., 1.}}, g, .7, cells);
  EXPECT_EQ(cells[0], 100);
  EXPECT_EQ(cells[44], 100); EXPECT_EQ(cells[55], 100);
  EXPECT_EQ(cells[99], 100); EXPECT_EQ(cells[66], 0);
}
TEST(OccupancyGrid, CompletePredictionTailUsesEachWholeCrossSection)
{
  struct Point {float x, y, z;};
  std::vector<Point> points{{100.F, 100.F, 100.F}};
  for (float center : {1.F, 3.F}) {
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {points.push_back({center + x * .2F, y * .3F, 1.F});}
    }
  }
  std::vector<std::array<double, 6>> boxes{{-1., -1., 0., 0., 0., 1.}};
  appendPredictionFootprintBoxes(points, 18, 9, boxes);
  ASSERT_EQ(boxes.size(), 3U);
  EXPECT_NEAR(boxes[1][0], .8, 1e-6); EXPECT_NEAR(boxes[1][3], 1.2, 1e-6);
  EXPECT_NEAR(boxes[2][0], 2.8, 1e-6); EXPECT_NEAR(boxes[2][4], .3, 1e-6);
  EXPECT_DOUBLE_EQ(boxes[2][2], 1.);
}
TEST(OccupancyGrid, HoldForTwentyMissedFramesThenRelease)
{
  GridPersistence history;
  std::vector<int8_t> cells{100, 0};
  history.apply(cells, 20);
  for (int miss = 1; miss <= 21; ++miss) {
    cells.assign(2, 0);
    history.apply(cells, 20);
    EXPECT_EQ(cells[0], miss <= 20 ? 100 : 0) << miss;
    EXPECT_EQ(cells[1], 0);
  }
}
TEST(OccupancyGrid, ReobservationRestartsMissCountAndNonpositiveHoldExpiresImmediately)
{
  GridPersistence history;
  std::vector<int8_t> cells{100};
  history.apply(cells, 1);
  cells[0] = 0; history.apply(cells, 1); EXPECT_EQ(cells[0], 100);
  cells[0] = 100; history.apply(cells, 1);
  cells[0] = 0; history.apply(cells, 1); EXPECT_EQ(cells[0], 100);
  cells[0] = 0; history.apply(cells, 1); EXPECT_EQ(cells[0], 0);
  cells[0] = 100; history.apply(cells, -1);
  cells[0] = 0; history.apply(cells, -1); EXPECT_EQ(cells[0], 0);
}
TEST(OccupancyGrid, ResetAndSizeChangesDiscardHistoryWithoutRewritingUnobservedCells)
{
  GridPersistence history;
  std::vector<int8_t> cells{100};
  history.apply(cells, 20);
  history.reset();
  cells[0] = -1; history.apply(cells, 20); EXPECT_EQ(cells[0], -1);
  cells[0] = 100; history.apply(cells, 20);
  cells.assign(2, 0); history.apply(cells, 20);
  EXPECT_EQ(cells, (std::vector<int8_t>{0, 0}));
}
}  // namespace
}  // namespace easyfly::occupancy
