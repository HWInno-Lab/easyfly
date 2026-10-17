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

#ifndef EASYFLY_MISSION_FORMAT__MISSION_FORMAT_HPP_
#define EASYFLY_MISSION_FORMAT__MISSION_FORMAT_HPP_
#include <string>
#include "easyfly_mission_format/legacy_mission.hpp"

namespace easyfly_mission_fsm
{
// Content detection also works with the facade's extensionless private snapshots.
// Static validation is not authorization to fly: live action admission still applies.
MissionValidation validateMissionText(const std::string & text);
MissionValidation validateMissionFile(const std::string & path);
// Classification is metadata only; callers must validate before using it.
std::string missionFormatName(const std::string & text);
// Identifies exact content bytes, never an execution or authentication identity.
std::string missionContentSha256(const std::string & text);
bool missionTextUtf8(const std::string & text);
}  // namespace easyfly_mission_fsm
#endif  // EASYFLY_MISSION_FORMAT__MISSION_FORMAT_HPP_
