// Copyright 2026 EasyFly Project
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

#include <cmath>
#include <limits>
#include <memory>

#include "easyfly_ui_bridge/model.hpp"

using easyfly_ui_bridge::ClockState;
using easyfly_ui_bridge::Quality;
namespace detail = easyfly_ui_bridge::detail;
namespace
{
using Grid = nav_msgs::msg::OccupancyGrid;
Grid grid(int second = 10)
{
  Grid m; m.header.frame_id = "odom"; m.header.stamp.sec = second;
  m.info.width = m.info.height = 2; m.info.resolution = .2F;
  m.info.origin.position.x = m.info.origin.position.y = -50;
  m.info.origin.orientation.w = 1.; m.data = {0, 100, -1, 0}; return m;
}
ClockState map_clock(int second = 10, uint64_t epoch = 0)
{return {Quality::ValidAtObservation, int64_t(second) * 1000000000LL, epoch};}
}  // namespace

TEST(OccupancyGrid, BoundedPlanarGeometryAndIndependentTimeEvidence)
{
  auto m = grid();
  const auto validate = [&]() {return detail::validate_occupancy_grid(m, map_clock(), 0);};
  EXPECT_EQ(validate(), Quality::ValidAtObservation);
  m.info.origin.orientation.z = std::sin(.3); m.info.origin.orientation.w = std::cos(.3);
  EXPECT_EQ(validate(), Quality::ValidAtObservation);  // A planar rotated map is supported.
  m.info.origin.orientation.x = .01; EXPECT_EQ(validate(), Quality::Invalid);
  m = grid(); m.info.origin.orientation.w = .5; EXPECT_EQ(validate(), Quality::Invalid);
  m = grid(); m.header.frame_id = "map"; EXPECT_EQ(validate(), Quality::Invalid);
  m = grid(); m.info.width = 4097; EXPECT_EQ(validate(), Quality::BudgetExceeded);
  m = grid(); m.info.width = m.info.height = 1024;
  EXPECT_EQ(validate(), Quality::BudgetExceeded);
  m = grid(); m.info.width = 0; EXPECT_EQ(validate(), Quality::Invalid);
  m = grid(); m.data.pop_back(); EXPECT_EQ(validate(), Quality::Invalid);
  m = grid(); m.data[0] = 101; EXPECT_EQ(validate(), Quality::Invalid);
  m.data[0] = -2; EXPECT_EQ(validate(), Quality::Invalid);
  m = grid(); m.info.resolution = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(validate(), Quality::Invalid);
  m = grid(); m.info.origin.position.y = std::numeric_limits<double>::infinity();
  EXPECT_EQ(validate(), Quality::Invalid);
  m = grid(); m.header.stamp.nanosec = 1000000000U; EXPECT_EQ(validate(), Quality::Invalid);
  m = grid(0); EXPECT_EQ(validate(), Quality::Invalid);
  m = grid();
  EXPECT_EQ(detail::validate_occupancy_grid(m, map_clock(13), 0), Quality::Stale);
  EXPECT_EQ(detail::validate_occupancy_grid(m, map_clock(), 2.01), Quality::Stale);
  EXPECT_EQ(detail::validate_occupancy_grid(m, map_clock(9), 0), Quality::Invalid);
  EXPECT_EQ(detail::validate_occupancy_grid(m, map_clock(), -.1), Quality::Invalid);
}

TEST(OccupancyGrid, BeforeClockRetainsOriginalReceiptAndDuplicateNeverRenews)
{
  detail::OccupancyGridBuffer buffer;
  auto m = std::make_shared<Grid>(grid());
  ClockState waiting;
  buffer.observe(m, waiting, 100., "source");
  auto s = buffer.snapshot(waiting, 100.2, "source");
  EXPECT_FALSE(s.value); EXPECT_EQ(s.evidence.quality, Quality::ClockNotReady);
  s = buffer.snapshot(map_clock(), 100.5, "source");
  ASSERT_TRUE(s.value); EXPECT_DOUBLE_EQ(s.evidence.receipt_age, .5);
  buffer.observe(std::make_shared<Grid>(*m), map_clock(), 101.9, "source");
  s = buffer.snapshot(map_clock(), 102.01, "source");
  EXPECT_FALSE(s.value); EXPECT_EQ(s.evidence.quality, Quality::Stale);
  EXPECT_DOUBLE_EQ(s.evidence.first_source_steady, 100.);
  EXPECT_DOUBLE_EQ(s.evidence.received_steady, 100.);
  // Starting the clock after expiry cannot turn cached data into a current map.
  buffer.clear(); buffer.observe(m, waiting, 200., "source");
  EXPECT_EQ(buffer.snapshot(map_clock(), 202.1, "source").evidence.quality, Quality::Stale);
}

TEST(OccupancyGrid, ConflictsRegressionsResetAndOriginChangesClearCurrentGeometry)
{
  detail::OccupancyGridBuffer buffer;
  const auto put = [&](Grid m, double steady, const std::string & source = "a") {
      buffer.observe(std::make_shared<Grid>(std::move(m)), map_clock(), steady, source);
    };
  put(grid(), 100.);
  const auto immutable = buffer.snapshot(map_clock(), 100., "a"); ASSERT_TRUE(immutable.value);
  auto conflict = grid(); conflict.data[0] = 100; put(conflict, 100.1);
  EXPECT_EQ(buffer.snapshot(map_clock(), 100.1, "a").evidence.quality, Quality::Invalid);
  EXPECT_FALSE(buffer.snapshot(map_clock(), 100.1, "a").value);
  put(grid(), 100.2); EXPECT_FALSE(buffer.snapshot(map_clock(), 100.2, "a").value);
  auto newer = grid(); newer.header.stamp.nanosec = 100000000;
  put(newer, 100.3); EXPECT_TRUE(buffer.snapshot(map_clock(), 100.3, "a").value);
  put(grid(9), 100.4); EXPECT_FALSE(buffer.snapshot(map_clock(), 100.4, "a").value);
  auto sentinel = grid(0); sentinel.data.assign(4, -1); put(sentinel, 100.5);
  auto s = buffer.snapshot(map_clock(), 100.5, "a");
  EXPECT_FALSE(s.value); EXPECT_EQ(s.evidence.quality, Quality::Invalid);
  EXPECT_FALSE(s.evidence.source_age_available);
  put(grid(), 100.6); EXPECT_TRUE(buffer.snapshot(map_clock(), 100.6, "a").value);
  EXPECT_FALSE(buffer.snapshot(map_clock(), 100.7, "").value);  // Ambiguous publisher.
  put(grid(), 100.8, "b"); EXPECT_TRUE(buffer.snapshot(map_clock(), 100.8, "b").value);
  EXPECT_FALSE(buffer.snapshot(map_clock(10, 1), 100.9, "b").value);
  EXPECT_TRUE(immutable.value); EXPECT_EQ(immutable.value->data[0], 0);
  buffer.clear(); EXPECT_FALSE(buffer.snapshot(map_clock(), 101., "b").value);
}
