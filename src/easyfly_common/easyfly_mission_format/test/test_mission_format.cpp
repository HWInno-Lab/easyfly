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
#include "easyfly_mission_format/mission_format.hpp"
using easyfly_mission_fsm::validateMissionText;
std::string prefix()
{
  return "schema_version: 1\nframe_id: odom\n"
         "units: {position: m, angle: rad, time: s}\nsteps:\n";
}
TEST(MissionFormat, AllSevenTypesAndLegacyCompatibility) {
  auto v = validateMissionText(
    prefix() +
    "- {type: takeoff, altitude: 1.5}\n- {type: hover, duration: 1}\n"
    "- {type: waypoint, x: 1, y: 0, z: 1.5}\n- {type: navigate, x: 0, y: 0, z: 1.5}\n"
    "- {type: track, target_id: -1, duration: 1}\n"
    "- {type: search, center_x: 0, center_y: 0, area_width: 4, "
    "area_height: 3, altitude: 1.5, pattern: boustrophedon}\n"
    "- {type: land}\n");
  ASSERT_TRUE(v.valid()) << v.error; ASSERT_EQ(v.steps.size(), 7u);
  EXPECT_EQ(v.steps[4].target_id, -1); EXPECT_EQ(v.steps[5].search_pattern, 1);
  EXPECT_TRUE(validateMissionText("---\n type = hover\n wait = 200\n").valid());
  EXPECT_TRUE(validateMissionText("altitude: 1\ntype: takeoff\n").valid());
  EXPECT_TRUE(validateMissionText("---\n" + prefix() + "- {type: land}\n").valid());
}
TEST(MissionFormat, RejectWholeMissionAndMalformedValues) {
  for (const auto & bad : {
    "- {type: takeoff, altitude: 1}\n- {type: unknown}\n", "- {type: land, retry: 2}\n",
    "- {type: navigate, x: .nan, y: 0, z: 1}\n", "- {type: takeoff, altitude: 1, altitude: 2}\n",
    "- {type: takeoff, altitude: '1'}\n", "- {type: track, target_id: -2, duration: 1}\n",
    "- {type: track, target_id: 1.5, duration: 1}\n",
    "- {type: track, target_id: 1, duration: 2, timeout: 1}\n",
    "- {type: hover, duration: 1, timeout: 0}\n", "- {type: navigate, x: 0, y: 0}\n",
    "- {type: search, center_x: 0, center_y: 0, area_width: 4, "
    "area_height: 3, altitude: 1.5, pattern: spiral}\n"})
  {
    auto result = validateMissionText(prefix() + bad);
    EXPECT_FALSE(result.valid()) << bad; EXPECT_TRUE(result.steps.empty());
  }
  for (const auto & bad : {"schema_version: 2\n", "schema_version: '1'\n", "schema_version: true\n",
      "schema_version: 1\nschema_version: 1\n", "schema_version: 1\nframe_id: map\n"})
  {
    EXPECT_FALSE(validateMissionText(std::string(bad) + "steps: [{type: land}]\n").valid());
  }
  EXPECT_FALSE(validateMissionText(prefix() + "- {type: land}\n---\nfoo: 2\n").valid());
  EXPECT_FALSE(validateMissionText("{\"schema_version\": 1}").valid());
  EXPECT_FALSE(validateMissionText(std::string(65537, 'x')).valid());
}
TEST(MissionFormat, BudgetsAndStepCountAreBounded) {
  auto text = prefix();
  for (int i = 0; i < 256; ++i) {
    text += "- {type: hover, duration: 0, timeout: 1}\n";
  }
  EXPECT_TRUE(validateMissionText(text).valid());
  EXPECT_FALSE(validateMissionText(text + "- {type: land}\n").valid());
  EXPECT_FALSE(
    validateMissionText(
      prefix() +
      "- {type: land, timeout: 3600}\n- {type: land, timeout: 3600}\n- {type: land}\n").valid());
}
TEST(MissionFormat, PrecisionLandingRequiresExplicitTagAndKeepsLegacyGrammar) {
  auto value = validateMissionText(prefix() + "- {type: precision_land, tag_id: 7, timeout: 30}\n");
  ASSERT_TRUE(value.valid()) << value.error;
  EXPECT_EQ(value.steps[0].tag_id, 7);
  EXPECT_EQ(value.steps[0].timeout, 30);
  for (const auto & bad : {"- {type: precision_land}\n",
      "- {type: precision_land, tag_id: -1}\n", "- {type: precision_land, tag_id: 1.5}\n",
      "- {type: precision_land, tag_id: 7, landing_speed: 1}\n"})
  {
    EXPECT_FALSE(validateMissionText(prefix() + bad).valid());
  }
  EXPECT_FALSE(validateMissionText("type=precision_land\ntag_id=7\n").valid());
}

TEST(MissionFormat, ExploreBoundsAndForbiddenBoxesAreStrict)
{
  const std::string fields = "type: explore, min_bound: [-2,-2,0.5], max_bound: [2,2,2.5], "
    "return_home: false";
  auto parsed = validateMissionText(
    prefix() + "- {" + fields +
    ", forbidden_boxes: [{min: [1,1,0.5], max: [1.5,1.5,2]}]}\n");
  ASSERT_TRUE(parsed.valid()) << parsed.error;
  EXPECT_FALSE(parsed.steps[0].return_home);
  ASSERT_EQ(parsed.steps[0].forbidden_boxes.size(), 1u);
  EXPECT_DOUBLE_EQ(parsed.steps[0].forbidden_boxes[0].max[2], 2);
  for (const auto & invalid : {"timeout: 4.99", "timeout: 901", "forbidden_boxes: true",
      "forbidden_boxes: [{min: [1,1,1], max: [1,2,2]}]",
      "forbidden_boxes: [{min: [1,1,1], max: [2,2,2], extra: 1}]"})
  {
    EXPECT_FALSE(validateMissionText(prefix() + "- {" + fields + ", " + invalid + "}\n").valid());
  }
  for (const auto & invalid : {
    "min_bound: [-2,-2,0], max_bound: [2,2,2.5], return_home: true",
    "min_bound: [-2,-2,0.5], max_bound: [2,2,2.6], return_home: true",
    "min_bound: [-2,-2,0.5], max_bound: [2,2,1.2], return_home: true",
    "min_bound: [-2,-2,0.5], max_bound: [2,2,2.5], return_home: 'true'",
    "min_bound: [-2,-2,0.5], max_bound: [2,2,2.5], return_home: 1",
    "min_bound: [-2,-2], max_bound: [2,2,2.5], return_home: true",
    "min_bound: [-2,-2,.nan], max_bound: [2,2,2.5], return_home: true"})
  {
    EXPECT_FALSE(validateMissionText(prefix() + "- {type: explore, " + invalid + "}\n").valid());
  }
  std::string boxes = "[";
  for (int i = 0; i < 17; ++i) {
    boxes += (i ? ", " : "") + std::string("{min: [0,0,0], max: [1,1,1]}");
  }
  EXPECT_FALSE(
    validateMissionText(
      prefix() + "- {" + fields + ", forbidden_boxes: " + boxes +
      "]}\n").valid());
  EXPECT_FALSE(validateMissionText("type=explore\n").valid());
}
