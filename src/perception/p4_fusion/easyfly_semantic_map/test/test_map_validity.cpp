// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include "easyfly_semantic_map/map_validity.hpp"
namespace easyfly_semantic_map
{
namespace
{
TEST(MapValidityTest, InvalidGridIsUnknownAndUnstamped)
{
  const auto grid = makeInvalidGrid(10.0, 8.0, 0.2, "odom");
  EXPECT_TRUE(zeroStamp(grid.header.stamp));
  ASSERT_FALSE(grid.data.empty());
  EXPECT_TRUE(
    std::all_of(
      grid.data.begin(), grid.data.end(), [](const int8_t cell) {
        return cell == -1;
      }));
}
TEST(MapValidityTest, InvalidObstaclesAreEmptyAndUnstamped)
{
  const auto obstacles = makeInvalidObstacles("odom");
  EXPECT_TRUE(zeroStamp(obstacles.header.stamp));
  EXPECT_TRUE(obstacles.obstacles.empty());
}
}  // namespace
}  // namespace easyfly_semantic_map
