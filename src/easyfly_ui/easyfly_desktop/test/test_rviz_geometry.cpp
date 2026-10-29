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
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
#include "easyfly_desktop/rviz_geometry.hpp"
namespace geo = easyfly_desktop::spatial;
namespace
{
std_msgs::msg::Header header()
{
  std_msgs::msg::Header h; h.frame_id = "odom"; h.stamp.sec = 10; return h;
}
sensor_msgs::msg::PointCloud2 cloud(size_t count = 3, bool big = false)
{
  sensor_msgs::msg::PointCloud2 m; m.header = header(); m.width = count; m.height = 1;
  m.point_step = 12; m.row_step = count * 12; m.is_bigendian = big; m.data.resize(m.row_step);
  for (unsigned i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f; f.name = std::vector<std::string>{"x", "y", "z"}[i];
    f.offset = i * 4; f.datatype = f.FLOAT32; f.count = 1; m.fields.push_back(f);
  }
  for (size_t i = 0; i < count; ++i) {
    for (unsigned j = 0; j < 3; ++j) {
      float value = static_cast<float>(i % 10) + j; uint32_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      for (unsigned k = 0; k < 4; ++k) {
        m.data[i * 12 + j * 4 + k] = (bits >> (8 * (big ? 3 - k : k))) & 255;
      }
    }
  }
  return m;
}
nav_msgs::msg::Path path(size_t count = 4)
{
  nav_msgs::msg::Path m; m.header = header();
  for (size_t i = 0; i < count; ++i) {
    geometry_msgs::msg::PoseStamped p; p.header = m.header; p.pose.position.x = i;
    p.pose.orientation.w = 1; m.poses.push_back(p);
  }
  return m;
}
nav_msgs::msg::OccupancyGrid map()
{
  nav_msgs::msg::OccupancyGrid m; m.header = header(); m.info.width = m.info.height = 2;
  m.info.resolution = 0.5; m.info.origin.orientation.w = 1;
  m.data = {-1, 0, 50, 100}; return m;
}
easyfly_msgs::msg::ObstacleInfoArray obstacles()
{
  easyfly_msgs::msg::ObstacleInfoArray m; m.header = header();
  easyfly_msgs::msg::ObstacleInfo box; box.header = m.header;
  box.center_z = 1; box.size_x = box.size_y = box.size_z = 1;
  m.obstacles.push_back(box); return m;
}
}  // namespace
TEST(RvizGeometry, PathUsesLatestSourceObservationNotStaleContainerStamp)
{
  auto m = path(); m.header.stamp.sec = 1;
  auto g = geo::convert(m); ASSERT_TRUE(g.valid); EXPECT_EQ(g.stamp_ns, 10000000000LL);
  EXPECT_EQ(g.vertices.size(), 4U); EXPECT_EQ(g.frame, "odom");
}
TEST(RvizGeometry, PathLimitsEndpointsAndRejectsMixedInvalidOrReversedInput)
{
  auto m = path(geo::kPathInput); auto g = geo::convert(m);
  ASSERT_TRUE(g.valid); EXPECT_EQ(g.vertices.size(), geo::kPathPoints);
  EXPECT_FLOAT_EQ(g.vertices.front().x, 0);
  EXPECT_FLOAT_EQ(g.vertices.back().x, geo::kPathInput - 1);
  m.poses.emplace_back(); EXPECT_FALSE(geo::convert(m).valid);
  m = path(); m.poses[0].header.frame_id = "map"; EXPECT_FALSE(geo::convert(m).valid);
  m = path(); m.poses[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(geo::convert(m).valid);
  m = path(); m.poses[0].header.stamp.sec = 11; EXPECT_FALSE(geo::convert(m).valid);
  m = path(); m.poses[0].header.stamp.sec = 0;
  m.poses[0].header.stamp.nanosec = 0; EXPECT_FALSE(geo::convert(m).valid);
}
TEST(RvizGeometry, EmptyZeroFutureAndStaleHaveExplicitMeanings)
{
  auto m = path(0); auto g = geo::convert(m);
  EXPECT_TRUE(g.valid); EXPECT_TRUE(g.vertices.empty());
  EXPECT_TRUE(geo::validity(g, 10000000000LL, 100, 100).empty());
  EXPECT_FALSE(geo::validity(g, 0, 100, 100).empty());
  EXPECT_FALSE(geo::validity(g, 9000000000LL, 100, 100).empty());
  EXPECT_FALSE(geo::validity(g, 13000000000LL, 100, 100).empty());
  EXPECT_FALSE(geo::validity(g, 10000000000LL, 100, 3000000100LL).empty());
  EXPECT_FALSE(geo::validity(g, 10000000000LL, 100, 99).empty());
  m.header.stamp.sec = 0; m.header.stamp.nanosec = 0; EXPECT_FALSE(geo::convert(m).valid);
}
TEST(RvizGeometry, MarkerOnlyAcceptsBoundedLineAndAppliesItsPose)
{
  visualization_msgs::msg::Marker m; m.header = header(); m.type = m.LINE_STRIP;
  m.pose.orientation.w = 1; m.pose.position.x = 2; m.points.resize(3); m.points[1].x = 1;
  auto g = geo::convert(m); ASSERT_TRUE(g.valid); EXPECT_FLOAT_EQ(g.vertices[1].x, 3);
  m.frame_locked = true; EXPECT_FALSE(geo::convert(m).valid); m.frame_locked = false;
  m.type = m.MESH_RESOURCE; EXPECT_FALSE(geo::convert(m).valid);
  m.action = m.DELETEALL; EXPECT_TRUE(geo::convert(m).vertices.empty());
  m.action = m.ADD; m.type = m.LINE_STRIP; m.points.resize(geo::kPathInput + 1);
  EXPECT_FALSE(geo::convert(m).valid);
}
TEST(RvizGeometry, CloudEndianAndOrganizedRowPadding)
{
  for (bool big : {false, true}) {
    auto m = cloud(2, big); auto g = geo::convert(m); ASSERT_TRUE(g.valid);
    EXPECT_FLOAT_EQ(g.vertices[1].x, 1); EXPECT_FLOAT_EQ(g.vertices[1].z, 3);
    m.height = 2; m.width = 1; m.row_step = 16;
    const auto original = m.data; m.data.assign(32, 0);
    std::copy(original.begin(), original.begin() + 12, m.data.begin());
    std::copy(original.begin() + 12, original.end(), m.data.begin() + 16);
    g = geo::convert(m); ASSERT_TRUE(g.valid); EXPECT_FLOAT_EQ(g.vertices[1].x, 1);
  }
}
TEST(RvizGeometry, CloudRejectsMalformedOffsetsDimensionsTypesAndBudgets)
{
  auto m = cloud(); m.fields[0].offset = 100; EXPECT_FALSE(geo::convert(m).valid);
  m = cloud(); m.data.pop_back(); EXPECT_FALSE(geo::convert(m).valid);
  m = cloud(); m.fields[0].datatype = m.fields[0].FLOAT64; EXPECT_FALSE(geo::convert(m).valid);
  m = cloud(); m.fields.push_back(m.fields[0]); EXPECT_FALSE(geo::convert(m).valid);
  m = cloud(); m.width = UINT32_MAX; m.height = UINT32_MAX;
  EXPECT_FALSE(geo::convert(m).valid);
  m = cloud(); m.data.resize(geo::kCloudBytes + 1); EXPECT_FALSE(geo::convert(m).valid);
}
TEST(RvizGeometry, CloudSamplesBoundedlyAndDropsNonFinite)
{
  auto m = cloud(geo::kInputPoints); auto g = geo::convert(m); ASSERT_TRUE(g.valid);
  EXPECT_LE(g.vertices.size(), geo::kCloudPoints); EXPECT_GT(g.omitted, 0U);
  m = cloud(); const float nan = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(m.data.data(), &nan, 4); g = geo::convert(m); ASSERT_TRUE(g.valid);
  EXPECT_EQ(g.vertices.size(), 2U); EXPECT_EQ(g.omitted, 1U);
}
TEST(RvizGeometry, OccupancyOnlyDrawsOccupiedCentersWithOriginTransform)
{
  auto m = map(); m.info.origin.position.x = 2; auto g = geo::convert(m);
  ASSERT_TRUE(g.valid); ASSERT_EQ(g.vertices.size(), 2U);
  EXPECT_FLOAT_EQ(g.vertices[0].x, 2.25); EXPECT_FLOAT_EQ(g.vertices[0].y, 0.75);
  EXPECT_FLOAT_EQ(g.point_size, 0.5);
}
TEST(RvizGeometry, OccupancyRejectsResetMalformedAndExcessOccupiedCells)
{
  auto m = map(); m.header.stamp.sec = 0; m.header.stamp.nanosec = 0;
  EXPECT_FALSE(geo::convert(m).valid);
  m = map(); m.data[0] = 101; EXPECT_FALSE(geo::convert(m).valid);
  m = map(); m.info.resolution = 0; EXPECT_FALSE(geo::convert(m).valid);
  m = map(); m.info.origin.orientation.w = 0; EXPECT_FALSE(geo::convert(m).valid);
  m = map(); m.info.width = geo::kCloudPoints + 1; m.info.height = 1;
  m.data.assign(m.info.width, 100);
  auto g = geo::convert(m); EXPECT_FALSE(g.valid); EXPECT_TRUE(g.vertices.empty());
}
TEST(RvizGeometry, ObstaclesHaveTwelveEdgesAndDoNotUseClassifiedCopy)
{
  auto m = obstacles(); auto g = geo::convert(m); ASSERT_TRUE(g.valid);
  EXPECT_EQ(g.vertices.size(), 24U); EXPECT_EQ(g.primitive, geo::Primitive::Segments);
  m.obstacles[0].obstacle_type = "person"; auto other = geo::convert(m);
  EXPECT_EQ(other.vertices.size(), g.vertices.size());
  m.obstacles.clear(); g = geo::convert(m); EXPECT_TRUE(g.valid); EXPECT_TRUE(g.vertices.empty());
}
TEST(RvizGeometry, ObstaclesRejectMixedHeadersBadSizesAndExcess)
{
  auto m = obstacles(); m.obstacles[0].header.frame_id = "map";
  EXPECT_FALSE(geo::convert(m).valid);
  m = obstacles(); m.obstacles[0].size_x = 0; EXPECT_FALSE(geo::convert(m).valid);
  m = obstacles(); m.obstacles.resize(geo::kBoxes + 1, m.obstacles[0]);
  EXPECT_FALSE(geo::convert(m).valid);
}

TEST(RvizGeometry, SpecializedLayersPreserveProvenanceAndExpireAtHalfSecond)
{
  easyfly_msgs::msg::LandingGuidanceTarget landing; landing.header = header();
  landing.source_header = header(); landing.source_header.frame_id = "camera_down_optical_frame";
  landing.tag_id = 7; landing.valid = true; landing.tag_size_m = .2; landing.plane_normal.z = 1;
  landing.position.z = .1;
  auto g = geo::convert(landing); ASSERT_TRUE(g.valid); ASSERT_EQ(g.vertices.size(), 2u);
  EXPECT_FLOAT_EQ(g.vertices[0].z, .1F); EXPECT_FLOAT_EQ(g.vertices[1].z, .6F);
  EXPECT_FALSE(geo::validity(g, 10600000000LL, 1000000000LL, 1100000000LL).empty());
  landing.source_header.stamp.sec = 9; EXPECT_FALSE(geo::convert(landing).valid);
  easyfly_msgs::msg::TargetPredictionArray prediction; prediction.header = header();
  easyfly_msgs::msg::TargetPrediction p; p.header = p.observation_header = header();
  p.valid = true; p.track_id = 1; p.horizon_seconds = .5; p.position_x = 2;
  prediction.predictions.push_back(p); g = geo::convert(prediction);
  ASSERT_TRUE(g.valid); EXPECT_EQ(g.vertices.size(), 6u);
  EXPECT_TRUE(g.detail.find("非实测") != std::string::npos);
  prediction.predictions.push_back(p); EXPECT_FALSE(geo::convert(prediction).valid);
  easyfly_msgs::msg::ClassifiedObstacleArray classified; classified.header = header();
  classified.geometry_valid = true; easyfly_msgs::msg::ClassifiedObstacle c;
  c.obstacle.header = header(); c.obstacle.obstacle_id = 1;
  c.obstacle.size_x = c.obstacle.size_y = c.obstacle.size_z = 1;
  c.obstacle.obstacle_type = "unknown"; c.association_valid = false; classified.obstacles.push_back(
    c);
  g = geo::convert(classified); ASSERT_TRUE(g.valid); EXPECT_EQ(g.vertices.size(), 24u);
  classified.geometry_valid = false; EXPECT_FALSE(geo::convert(classified).valid);
}
