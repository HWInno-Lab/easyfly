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
#include "easyfly_trajectory_optimizer/observation_grid.hpp"
namespace et = easyfly::trajectory;
static et::Grid grid()
{
  et::Grid g;g.size = {10, 8, 6};g.origin = {-1., -1., 0.};g.resolution = .2;
  g.data.assign(g.size.prod(), -1);return g;
}
TEST(Observations, DistinctConsecutiveScansAndHitPriority)
{
  auto g = grid();et::ObservationGrid evidence(g, 3);const et::Cell cell{4, 4, 3};
  evidence.begin();evidence.observeHit(cell);evidence.commit();
  for (int scan = 0; scan < 2; ++scan) {
    evidence.begin();for (int ray = 0; ray < 1000; ++ray) {
      evidence.observeFree(cell);
    }
    evidence.commit();EXPECT_EQ(g.data[g.index(cell)], 100);
  }
  evidence.begin();evidence.commit(); // no visibility interrupts the clearing streak
  evidence.begin();evidence.observeFree(cell);evidence.commit();
  EXPECT_EQ(g.data[g.index(cell)], 100);
  evidence.begin();evidence.observeHit(cell);evidence.observeFree(cell);evidence.commit();
  EXPECT_EQ(g.data[g.index(cell)], 100);
  for (int scan = 0; scan < 3; ++scan) {
    evidence.begin();evidence.observeFree(cell);evidence.commit();
    EXPECT_EQ(g.data[g.index(cell)], scan == 2 ? 0 : 100);
  }
  EXPECT_EQ(g.data[g.index({0, 0, 0})], -1);
}
TEST(Observations, DefaultRetainsOccupiedCells)
{
  auto g = grid();et::ObservationGrid evidence(g);const et::Cell cell{4, 4, 3};
  evidence.begin();evidence.observeHit(cell);evidence.commit();
  for (int scan = 0; scan < 50; ++scan) {
    evidence.begin();evidence.observeFree(cell);evidence.commit();
  }
  EXPECT_EQ(g.data[g.index(cell)], 100);
}
TEST(Observations, RollingPreservesWorldEvidenceAndLeavesNewCellsUnknown)
{
  auto g = grid();et::ObservationGrid evidence(g, 3);const et::Cell cell{4, 4, 3};
  const auto world = g.center(cell);
  evidence.begin();evidence.observeHit(cell);evidence.commit();
  evidence.begin();evidence.observeFree(cell);evidence.commit();
  evidence.shift({3, -2, 0});const auto shifted = g.cell(world);
  EXPECT_NEAR((g.center(shifted) - world).norm(), 0., 1e-12);
  EXPECT_EQ(g.data[g.index(shifted)], 100);
  EXPECT_EQ(g.data[g.index({9, 0, 0})], -1);
  evidence.begin();evidence.observeFree(shifted);evidence.commit();
  EXPECT_EQ(g.data[g.index(shifted)], 100);
  evidence.begin();evidence.observeFree(shifted);evidence.commit();
  EXPECT_EQ(g.data[g.index(shifted)], 0);
  evidence.shift({-3, 2, 0});EXPECT_EQ(g.data[g.index(cell)], 0);
  evidence.reset();EXPECT_TRUE(
    std::all_of(
      g.data.begin(), g.data.end(), [](auto v) {
        return v == -1;
      }));
}

TEST(Observations, UnknownEnvelopeMatchesIndependentBoxSearch)
{
  et::Grid g;g.size = {19, 17, 15};g.origin = {-2., -2., 0.};g.resolution = .2;
  g.data.assign(g.size.prod(), 0);
  g.data[g.index({0, 0, 0})] = -1;g.data[g.index({11, 9, 7})] = -1;
  g.data[g.index({10, 9, 7})] = 100;
  const auto original = g;et::reserveUnknown(g, .65);
  for (int i = 0; i < g.size.prod(); ++i) {
    const auto cell = g.cell(i);bool blocked = false;
    for (int j = 0; j < original.size.prod(); ++j) {
      const auto other = original.cell(j);
      if (original.data[j] == -1 && (cell - other).cwiseAbs().maxCoeff() <= 5) {blocked = true;}
    }
    EXPECT_EQ(g.data[i], original.data[i] == 100 ? 100 : (blocked ? -1 : 0));
  }
  EXPECT_THROW(et::reserveUnknown(g, .1), std::invalid_argument);
}
