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

#include <sstream>
#include <easyfly_mission_fsm/legacy_mission.hpp>
#include <easyfly_mission_fsm/mission_format.hpp>
#include <easyfly_mission_format/mission_format.hpp>

int main()
{
  std::istringstream text("type=takeoff\naltitude=1\n---\ntype=land\n");
  const auto legacy = easyfly_mission_fsm::validateLegacyMission(text);
  const auto yaml = easyfly_mission_fsm::validateMissionText(
    "schema_version: 1\nframe_id: odom\nunits: {position: m, angle: rad, time: s}\n"
    "steps: [{type: precision_land, tag_id: 7}]\n");
  return legacy.valid() && legacy.steps.size() == 2 && yaml.valid() &&
         yaml.steps.front().tag_id == 7 &&
         easyfly_mission_fsm::missionContentSha256("abc") ==
         "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad" ? 0 : 1;
}
