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

#include <algorithm>
#include <utility>

#include "easyfly_ui_bridge/model.hpp"
namespace easyfly_ui_bridge
{
namespace
{
bool utf8(const std::string & text)
{
  return easyfly_mission_fsm::missionTextUtf8(text);
}
}  // namespace
namespace detail
{
bool catalog_name(const std::string & name)
{
  if (!basename(name) || name.size() > 128 || name.front() == '.' || !utf8(name)) {return false;}
  const auto suffix = name.substr(
    name.find_last_of('.') == std::string::npos ?
    name.size() : name.find_last_of('.'));
  return suffix == ".mission" || suffix == ".yaml" || suffix == ".yml";
}
Quality validate_mission_details(
  const easyfly_msgs::srv::GetMissionDetails::Response & response,
  const std::string & requested, easyfly_mission_fsm::MissionValidation * output)
{
  if (output) {*output = {};}
  if (response.content.size() > 65536 || response.message.size() > 2048 ||
    response.mission_name.size() > 128 || response.status.size() > 64 ||
    response.content_sha256.size() > 64) {return Quality::BudgetExceeded;}
  if (!catalog_name(requested) || response.mission_name != requested ||
    response.status.empty() || !utf8(response.status) || !utf8(response.message))
  {return Quality::Invalid;}
  if (!response.success) {
    return response.content.empty() && response.content_sha256.empty() &&
           response.status != "valid_snapshot" ? Quality::Unknown : Quality::Invalid;
  }
  if (response.status != "valid_snapshot" || !utf8(response.content) ||
    response.content_sha256.size() != 64 ||
    response.content_sha256 != easyfly_mission_fsm::missionContentSha256(response.content))
  {return Quality::Invalid;}
  auto plan = easyfly_mission_fsm::validateMissionText(response.content);
  if (!plan.valid()) {return Quality::Invalid;}
  if (output) {*output = std::move(plan);}
  return Quality::ValidAtObservation;
}
}  // namespace detail
}  // namespace easyfly_ui_bridge
