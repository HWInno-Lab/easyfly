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

#include "easyfly_ui_bridge/model.hpp"
namespace b = easyfly_ui_bridge;
namespace
{
using Response = easyfly_msgs::srv::GetMissionDetails::Response;
Response valid(const std::string & content = "type=hover\nwait=2\n")
{
  Response response; response.success = true; response.status = "valid_snapshot";
  response.mission_name = "demo.yaml"; response.content = content;
  response.content_sha256 = easyfly_mission_fsm::missionContentSha256(content); return response;
}
}
TEST(MissionDetailsModel, FingerprintFormatAndNormalizedDefaults)
{
  EXPECT_EQ(
    easyfly_mission_fsm::missionContentSha256("abc"),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(
    easyfly_mission_fsm::missionFormatName("# comment\n---\naltitude: 1\ntype: takeoff\n"),
    "legacy_flat");
  auto response = valid(
    "schema_version: 1\nframe_id: odom\n"
    "units: {position: m, angle: rad, time: s}\nsteps:\n"
    "- {type: takeoff, altitude: 1.5}\n- {type: waypoint, x: 1, y: 2, z: 1.5}\n"
    "- {type: hover, duration: 2}\n- {type: precision_land, tag_id: 4}\n");
  easyfly_mission_fsm::MissionValidation plan;
  ASSERT_EQ(
    b::detail::validate_mission_details(
      response, "demo.yaml",
      &plan), b::Quality::ValidAtObservation);
  ASSERT_EQ(plan.steps.size(), 4U); EXPECT_DOUBLE_EQ(plan.steps[0].climb_speed, 1);
  EXPECT_DOUBLE_EQ(plan.steps[1].acceptance_radius, .5);
  EXPECT_DOUBLE_EQ(plan.steps[2].wait, 2); EXPECT_EQ(plan.steps[3].tag_id, 4);
  EXPECT_EQ(easyfly_mission_fsm::missionFormatName(response.content), "yaml_v1");
}
TEST(MissionDetailsModel, RejectWrongIdentityDigestAndInvalidContent)
{
  auto response = valid(); response.mission_name = "other.yaml";
  EXPECT_EQ(b::detail::validate_mission_details(response, "demo.yaml"), b::Quality::Invalid);
  response = valid(); response.content_sha256[0] = response.content_sha256[0] == 'a' ? 'b' : 'a';
  EXPECT_EQ(b::detail::validate_mission_details(response, "demo.yaml"), b::Quality::Invalid);
  response = valid("type=hover\nwait=2\nextra=1\n");
  EXPECT_EQ(b::detail::validate_mission_details(response, "demo.yaml"), b::Quality::Invalid);
  response = valid("#\xff\n" "type=land\n");
  EXPECT_EQ(b::detail::validate_mission_details(response, "demo.yaml"), b::Quality::Invalid);
  response = valid(std::string(65537, '#'));
  EXPECT_EQ(b::detail::validate_mission_details(response, "demo.yaml"), b::Quality::BudgetExceeded);
  for (const auto & name : {"../demo.yaml", ".hidden.yaml", "/tmp/demo.yaml", "other.txt"}) {
    EXPECT_FALSE(b::detail::catalog_name(name));
  }
}
TEST(MissionDetailsModel, FailureCannotCarryAnOldExecutablePreview)
{
  auto response = valid(); response.success = false; response.status = "invalid";
  EXPECT_EQ(b::detail::validate_mission_details(response, "demo.yaml"), b::Quality::Invalid);
  response.content.clear(); response.content_sha256.clear(); response.message = "not in catalog";
  easyfly_mission_fsm::MissionValidation plan; plan.steps.push_back({});
  EXPECT_EQ(b::detail::validate_mission_details(response, "demo.yaml", &plan), b::Quality::Unknown);
  EXPECT_TRUE(plan.steps.empty());
}
