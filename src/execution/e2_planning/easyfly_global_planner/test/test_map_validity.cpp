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
#include "easyfly_global_planner/map_validity.hpp"
namespace easyfly_global_planner
{
namespace
{
nav_msgs::msg::OccupancyGrid validGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "odom";
  grid.header.stamp.sec = 1;
  grid.info.resolution = 0.2F;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -1.0;
  grid.info.width = 2;
  grid.info.height = 2;
  grid.data = {0, 0, -1, 100};
  return grid;
}
TEST(MapValidityTest, RejectsSemanticMapReset)
{
  auto invalid = validGrid();
  invalid.header.stamp = builtin_interfaces::msg::Time{};
  invalid.data.assign(4, -1);
  EXPECT_FALSE(occupancyGridReady(invalid));
}
TEST(MapValidityTest, RejectsRecentButMalformedOrUnknownGrid)
{
  auto recent_unknown = validGrid();
  recent_unknown.data.assign(4, -1);
  EXPECT_FALSE(occupancyGridReady(recent_unknown));
  auto malformed = validGrid();
  malformed.data.resize(3);
  EXPECT_FALSE(occupancyGridReady(malformed));
}
TEST(MapValidityTest, RejectsUnexpectedFrameOrNonFiniteGeometry)
{
  auto wrong_frame = validGrid();
  wrong_frame.header.frame_id = "map";
  EXPECT_FALSE(occupancyGridReady(wrong_frame));
  auto nan_resolution = validGrid();
  nan_resolution.info.resolution = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(occupancyGridReady(nan_resolution));
  auto inf_origin = validGrid();
  inf_origin.info.origin.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(occupancyGridReady(inf_origin));
  auto nan_origin = validGrid();
  nan_origin.info.origin.position.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(occupancyGridReady(nan_origin));
}
TEST(MapValidityTest, AcceptsStampedGridWithKnownCells)
{
  EXPECT_TRUE(occupancyGridReady(validGrid()));
}
}  // namespace
}  // namespace easyfly_global_planner
