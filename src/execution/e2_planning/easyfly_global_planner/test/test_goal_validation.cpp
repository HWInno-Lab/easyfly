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
#include "easyfly_global_planner/goal_validation.hpp"
namespace easyfly_global_planner
{
namespace
{
using Goal = easyfly_msgs::action::NavigateTo::Goal;
Goal validGoal()
{
  Goal goal;
  goal.target_x = 1.0;
  goal.target_y = -2.0;
  goal.target_z = std::numeric_limits<double>::quiet_NaN();
  goal.max_speed = 0.0;
  return goal;
}
TEST(GoalValidationTest, AllowsNaNHeightAndDefaultSpeed)
{
  EXPECT_TRUE(validNavigateGoal(validGoal()));
}
TEST(GoalValidationTest, RejectsNonFiniteHorizontalCoordinates)
{
  auto goal = validGoal();
  goal.target_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validNavigateGoal(goal));
  goal = validGoal();
  goal.target_y = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validNavigateGoal(goal));
}
TEST(GoalValidationTest, RejectsInfiniteHeightAndSpeed)
{
  auto goal = validGoal();
  goal.target_z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validNavigateGoal(goal));
  goal = validGoal();
  goal.target_z = -std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validNavigateGoal(goal));
  goal = validGoal();
  goal.max_speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validNavigateGoal(goal));
  goal = validGoal();
  goal.max_speed = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validNavigateGoal(goal));
}
TEST(GoalValidationTest, KeepsFiniteNonPositiveSpeedContract)
{
  auto goal = validGoal();
  goal.max_speed = -1.0;
  EXPECT_TRUE(validNavigateGoal(goal));
}
}  // namespace
}  // namespace easyfly_global_planner
