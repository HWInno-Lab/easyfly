// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_occupancy_mapping/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

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
