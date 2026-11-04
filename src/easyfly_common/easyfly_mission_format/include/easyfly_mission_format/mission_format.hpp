// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_mission_format/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project

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
struct MissionDocument
{
  MissionValidation validation;
  std::string content;
};
// Retain exactly the bytes validated through one bounded, regular-file descriptor.
MissionDocument loadMissionDocument(const std::string & path);
// Classification is metadata only; callers must validate before using it.
std::string missionFormatName(const std::string & text);
// Identifies exact content bytes, never an execution or authentication identity.
std::string missionContentSha256(const std::string & text);
bool missionTextUtf8(const std::string & text);
}  // namespace easyfly_mission_fsm
#endif  // EASYFLY_MISSION_FORMAT__MISSION_FORMAT_HPP_
