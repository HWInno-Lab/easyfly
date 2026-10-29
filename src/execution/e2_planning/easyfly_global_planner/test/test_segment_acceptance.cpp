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
#include <cmath>
#include <limits>
#include "easyfly_global_planner/segment_acceptance.hpp"
namespace easyfly_global_planner
{
namespace
{
TEST(SegmentAcceptanceTest, KeepsSmallConfiguredTolerance)
{
  EXPECT_DOUBLE_EQ(normalizeSegmentAcceptance(0.05), 0.05);
}
TEST(SegmentAcceptanceTest, ClampsToleranceToSafeUpperBound)
{
  EXPECT_DOUBLE_EQ(normalizeSegmentAcceptance(0.5), kMaxSegmentAcceptance);
  EXPECT_DOUBLE_EQ(normalizeSegmentAcceptance(0.10), kMaxSegmentAcceptance);
}
TEST(SegmentAcceptanceTest, UsesSafeDefaultForInvalidTolerance)
{
  EXPECT_DOUBLE_EQ(normalizeSegmentAcceptance(0.0), kDefaultSegmentAcceptance);
  EXPECT_DOUBLE_EQ(normalizeSegmentAcceptance(-0.1), kDefaultSegmentAcceptance);
  EXPECT_DOUBLE_EQ(
    normalizeSegmentAcceptance(std::numeric_limits<double>::quiet_NaN()),
    kDefaultSegmentAcceptance);
  EXPECT_DOUBLE_EQ(
    normalizeSegmentAcceptance(std::numeric_limits<double>::infinity()),
    kDefaultSegmentAcceptance);
}
}  // namespace
}  // namespace easyfly_global_planner
