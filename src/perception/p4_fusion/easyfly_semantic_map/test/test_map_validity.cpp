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
