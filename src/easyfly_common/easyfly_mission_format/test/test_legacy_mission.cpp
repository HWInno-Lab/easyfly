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
#include <easyfly_mission_format/legacy_mission.hpp>
using easyfly_mission_fsm::validateLegacyMission;
TEST(LegacyMission, ValidCompleteMissionAndDefaults) {
  std::istringstream in(
    "# trusted flat mission\n type=takeoff\naltitude:2.5\n---\ntype=hover\nwait=1\n---\n"
    "type=waypoint\nx=1\ny=2\nz=2.5\n---\ntype=land\n");
  const auto result = validateLegacyMission(in);
  ASSERT_TRUE(result.valid()) << result.error;
  ASSERT_EQ(result.steps.size(), 4u);
  EXPECT_EQ(result.steps[0].climb_speed, 1.0);
  EXPECT_EQ(result.steps[2].max_speed, 2.0);
}
TEST(LegacyMission, BadLaterStepRejectsEntireMission) {
  for (const auto & suffix : {"type=search", "type=land\nunknown=1", "type=hover\nwait=nan",
      "type=hover\nwait=-1", "type=takeoff\naltitude=2m", "type=takeoff\naltitude=0",
      "type=waypoint\nx=1\ny=2", "type=hover\nwait=1\nwait=2", "steps:\n- type: land",
      "type=takeoff\naltitude=51", "type=takeoff\naltitude=1\nclimb_speed=21",
      "type=waypoint\nx=1e300\ny=0\nz=1", "type=hover\nwait=3601"})
  {
    std::istringstream in(std::string("type=land\n---\n") + suffix);
    const auto result = validateLegacyMission(in);
    EXPECT_FALSE(result.valid()) << suffix;
    EXPECT_TRUE(result.steps.empty()) << suffix;
    EXPECT_FALSE(result.error.empty());
  }
}
TEST(LegacyMission, SizeAndStepBudgets) {
  std::istringstream empty(" # empty\n\n");
  EXPECT_FALSE(validateLegacyMission(empty).valid());
  std::string many;
  for (int i = 0; i < 257; ++i) {
    many += "type=land\n---\n";
  }
  std::istringstream steps(many);
  EXPECT_FALSE(validateLegacyMission(steps).valid());
  std::istringstream large("#" + std::string(65536, 'x'));
  EXPECT_FALSE(validateLegacyMission(large).valid());
}
