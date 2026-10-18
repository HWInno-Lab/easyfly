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

#include "easyfly_occupancy_mapping/grid_inflation.hpp"

namespace easyfly::occupancy
{
namespace
{

TEST(GridInflationTest, KeepsEvidenceBasedMinimum)
{
  EXPECT_DOUBLE_EQ(normalizeGridInflationRadius(0.5), kMinimumGridInflationRadius);
  EXPECT_DOUBLE_EQ(normalizeGridInflationRadius(0.7), kMinimumGridInflationRadius);
  EXPECT_DOUBLE_EQ(normalizeGridInflationRadius(0.8), 0.8);
}

TEST(GridInflationTest, InvalidRadiusFallsBackToSafeDefault)
{
  EXPECT_DOUBLE_EQ(
    normalizeGridInflationRadius(std::numeric_limits<double>::quiet_NaN()),
    kDefaultGridInflationRadius);
  EXPECT_DOUBLE_EQ(
    normalizeGridInflationRadius(std::numeric_limits<double>::infinity()),
    kDefaultGridInflationRadius);
  EXPECT_DOUBLE_EQ(normalizeGridInflationRadius(-0.1), kDefaultGridInflationRadius);
}

}  // namespace
}  // namespace easyfly::occupancy
