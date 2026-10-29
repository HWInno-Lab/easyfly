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
#include "easyfly_global_planner/rolling_target.hpp"
namespace gp = easyfly_global_planner;
static easyfly_msgs::msg::VoxelGrid map()
{
  easyfly_msgs::msg::VoxelGrid m;m.valid = true;m.resolution = .2;m.inflation_radius = .65;
  m.origin.x = -4.;m.origin.y = -4.;m.origin.z = -.2;m.size_x = 60;m.size_y = 40;m.size_z = 18;
  m.data.assign(60 * 40 * 18, 0);return m;
}
TEST(RollingTarget, OutsideGoalProducesObservedIntermediateNotFalseArrival)
{
  auto m = map();auto target = gp::rollingTarget(m, 0., 0., 30., 0., 1.5, 4.);
  ASSERT_TRUE(target);EXPECT_TRUE(target->partial);EXPECT_NEAR(target->x, 4., 1e-8);
  m.origin.x = 24.;target = gp::rollingTarget(m, 28., 0., 30., 0., 1.5, 4.);
  ASSERT_TRUE(target);EXPECT_FALSE(target->partial);EXPECT_DOUBLE_EQ(target->x, 30.);
}
TEST(RollingTarget, UnknownNearEndpointAndInvalidLayoutAreRejected)
{
  auto m = map();
  // Unknown .6 m beside an otherwise free centre prevents full-envelope admission.
  m.data[40 + 60 * (23 + 40 * 8)] = -1;
  EXPECT_FALSE(gp::observedVoxelEndpoint(m, 4., 0., 1.5));
  std::fill(m.data.begin(), m.data.end(), -1);
  EXPECT_FALSE(gp::rollingTarget(m, 0., 0., 30., 0., 1.5, 4.));
  m = map();m.data.pop_back();EXPECT_FALSE(gp::observedVoxelEndpoint(m, 4., 0., 1.5));
  m = map();m.resolution = 0.;EXPECT_FALSE(gp::observedVoxelEndpoint(m, 4., 0., 1.5));
}
