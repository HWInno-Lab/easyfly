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

#ifndef EASYFLY_MISSION_FORMAT__LEGACY_MISSION_HPP_
#define EASYFLY_MISSION_FORMAT__LEGACY_MISSION_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace easyfly_mission_fsm
{
struct ExplorationBox
{
  std::array<double, 3> min, max;
};
struct MissionStep
{
  std::string type;
  double altitude{0.0}, climb_speed{1.0};
  double x{0.0}, y{0.0}, z{0.0}, yaw{0.0}, max_speed{2.0}, wait{0.0};
  double timeout{120.0}, duration{0.0}, acceptance_radius{0.5};
  int target_id{-1}, search_pattern{1};
  std::array<double, 3> min_bound{{-2, -2, .5}}, max_bound{{2, 2, 2.5}};
  std::vector<ExplorationBox> forbidden_boxes;
  bool return_home{true};
  int tag_id{-1};  // Formal YAML only; legacy grammar is unchanged.
  double follow_distance{2.0}, follow_height{0.0};
  double center_x{0.0}, center_y{0.0}, area_width{0.0}, area_height{0.0};
};
struct MissionValidation
{
  std::vector<MissionStep> steps;
  std::string error;
  bool valid() const {return error.empty() && !steps.empty();}
};
inline std::string trimLegacy(std::string s)
{
  const auto first = s.find_first_not_of(" \t\r");
  if (first == std::string::npos) {return std::string{};}
  return s.substr(first, s.find_last_not_of(" \t\r") - first + 1);
}
// This is the existing flat key=value/key:value format, NOT general YAML/JSON.
// Validate every step before executing anything. Live flight admission remains
// the responsibility of the existing action servers and their configured limits.
inline MissionValidation validateLegacyMission(std::istream & input)
{
  MissionValidation result;
  MissionStep current;
  std::set<std::string> seen;
  auto reject = [&](const std::string & message) {
      result.steps.clear(); result.error = message;
    };
  auto finish = [&]() -> bool {
      if (seen.empty()) {return true;}
      const std::map<std::string, std::set<std::string>> fields{
        {"takeoff", {"type", "altitude", "climb_speed"}},
        {"waypoint", {"type", "x", "y", "z", "yaw", "max_speed"}},
        {"hover", {"type", "wait"}}, {"land", {"type"}}};
      const auto spec = fields.find(current.type);
      if (spec == fields.end()) {reject("unsupported or missing step type"); return false;}
      for (const auto & key : seen) {
        if (!spec->second.count(key)) {
          reject("field not supported for step: " + key); return false;
        }
      }
      // Static legacy bounds match the existing default altitude/speed caps.
      // Lower live overrides and relative waypoint distance still need backend
      // admission; this parser never promises that a valid file is flyable.
      if ((current.type == "takeoff" &&
        (!seen.count("altitude") || current.altitude <= 0 || current.altitude > 50 ||
        current.climb_speed < 0 || current.climb_speed > 20)) ||
        (current.type == "waypoint" && (!seen.count("x") || !seen.count("y") || !seen.count("z") ||
        std::abs(current.x) > 1000 || std::abs(current.y) > 1000 || current.z < 0 ||
        current.z > 50 ||
        std::abs(current.yaw) > 3.141592653589793 || current.max_speed < 0 ||
        current.max_speed > 20)) ||
        (current.type == "hover" && (current.wait < 0 || current.wait > 3600)))
      {
        reject("missing required field or invalid step range"); return false;
      }
      if (result.steps.size() >= 256) {reject("more than 256 steps"); return false;}
      if (current.type == "hover") {current.timeout = std::max(120.0, current.wait + 1.0);}
      result.steps.push_back(current);
      current = MissionStep{}; seen.clear(); return true;
    };
  size_t bytes = 0, line_number = 0;
  std::string line;
  while (std::getline(input, line)) {
    bytes += line.size() + 1; ++line_number;
    if (bytes > 65536) {reject("mission exceeds 64KiB"); return result;}
    line = trimLegacy(line);
    if (line.empty() || line[0] == '#') {continue;}
    if (line == "---") {if (!finish()) {return result;} continue;}
    auto separator = line.find('=');
    if (separator == std::string::npos) {separator = line.find(':');}
    if (separator == std::string::npos) {
      reject("invalid syntax at line " + std::to_string(line_number)); return result;
    }
    const auto key = trimLegacy(line.substr(0, separator));
    const auto value = trimLegacy(line.substr(separator + 1));
    if (!seen.insert(key).second) {reject("duplicate field: " + key); return result;}
    if (key == "type") {current.type = value; continue;}
    std::map<std::string, double *> numbers{
      {"altitude", &current.altitude}, {"climb_speed", &current.climb_speed},
      {"x", &current.x}, {"y", &current.y}, {"z", &current.z},
      {"yaw", &current.yaw}, {"max_speed", &current.max_speed}, {"wait", &current.wait}};
    const auto number = numbers.find(key);
    if (number == numbers.end()) {reject("unknown field: " + key); return result;}
    try {
      size_t consumed = 0;
      const double parsed = std::stod(value, &consumed);
      if (consumed != value.size() || !std::isfinite(parsed)) {
        throw std::invalid_argument("number");
      }
      *number->second = parsed;
    } catch (const std::exception &) {
      reject("invalid number at line " + std::to_string(line_number)); return result;
    }
  }
  if (input.bad()) {reject("mission read failed"); return result;}
  if (!finish()) {return result;}
  if (result.steps.empty()) {reject("empty mission");}
  return result;
}
inline MissionValidation validateLegacyMissionFile(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {return MissionValidation{{}, "cannot open mission file"};}
  std::string text(65537, '\0'); stream.read(text.data(), text.size());
  if (stream.bad() || stream.gcount() > 65536) {
    return MissionValidation{{}, "mission read failed or exceeds 64KiB"};
  }
  text.resize(static_cast<size_t>(stream.gcount()));
  std::istringstream content(text);
  return validateLegacyMission(content);
}
}  // namespace easyfly_mission_fsm
#endif  // EASYFLY_MISSION_FORMAT__LEGACY_MISSION_HPP_
