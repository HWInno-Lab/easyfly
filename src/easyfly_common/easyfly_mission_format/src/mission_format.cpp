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

#include "easyfly_mission_format/mission_format.hpp"
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cerrno>
#include <set>

namespace easyfly_mission_fsm
{
namespace
{
void require(bool condition, const std::string & error)
{
  if (!condition) {throw std::invalid_argument(error);}
}
void keys(const YAML::Node & map, const std::set<std::string> & allowed)
{
  require(map.IsMap(), "expected mapping");
  std::set<std::string> seen;
  for (const auto & item : map) {
    require(item.first.IsScalar(), "non-scalar key");
    auto key = item.first.as<std::string>();
    require(seen.insert(key).second, "duplicate key: " + key);
    require(allowed.count(key), "unsupported field: " + key);
  }
}
std::string word(const YAML::Node & node)
{
  require(node.IsScalar(), "expected scalar string");
  return node.as<std::string>();
}
double number(const YAML::Node & node, double low, double high)
{
  require(
    node.IsScalar() && node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str",
    "expected unquoted numeric scalar");
  const auto raw = node.Scalar();
  size_t consumed = 0;
  const auto value = std::stod(raw, &consumed);
  require(
    consumed == raw.size() && std::isfinite(value) && value >= low && value <= high,
    "numeric field out of range");
  return value;
}
double optional(
  const YAML::Node & node, const std::string & key, double value, double low,
  double high)
{
  return node[key] ? number(node[key], low, high) : value;
}
int integer(const YAML::Node & node, int low, int high)
{
  require(node.IsScalar(), "expected integer");
  const auto text = node.Scalar();
  require(!text.empty(), "empty integer");
  const auto first = text[0] == '-' ? 1u : 0u;
  require(
    first < text.size() && std::all_of(
      text.begin() + first, text.end(),
      [](char c) {return c >= '0' && c <= '9';}), "expected decimal integer");
  return static_cast<int>(number(node, low, high));
}
std::array<double, 3> point(const YAML::Node & node)
{
  require(node.IsSequence() && node.size() == 3, "expected XYZ triple");
  return {{number(node[0], -1000, 1000), number(node[1], -1000, 1000),
    number(node[2], -1000, 1000)}};
}
bool boolean(const YAML::Node & node)
{
  require(
    node.IsScalar() && node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str" &&
    (node.Scalar() == "true" || node.Scalar() == "false"), "expected true or false");
  return node.Scalar() == "true";
}
MissionValidation yaml(const std::string & text)
{
  const auto docs = YAML::LoadAll(text);
  require(docs.size() == 1, "exactly one YAML document required");
  const auto root = docs.front();
  keys(root, {"schema_version", "frame_id", "units", "failure_policy", "steps"});
  require(integer(root["schema_version"], 1, 1) == 1, "unsupported schema_version");
  require(word(root["frame_id"]) == "odom", "only odom frame supported");
  keys(root["units"], {"position", "angle", "time"});
  require(
    word(root["units"]["position"]) == "m" && word(root["units"]["angle"]) == "rad" &&
    word(root["units"]["time"]) == "s", "units must be m/rad/s");
  if (root["failure_policy"]) {
    require(word(root["failure_policy"]) == "abort", "only abort policy supported");
  }
  const auto steps = root["steps"];
  require(steps.IsSequence() && steps.size() > 0 && steps.size() <= 256, "expected 1..256 steps");
  MissionValidation result;
  double total_budget = 0;
  for (const auto & node : steps) {
    require(node.IsMap(), "step must be mapping");
    MissionStep step;
    step.type = word(node["type"]);
    std::set<std::string> allowed{"type", "timeout"};
    const std::map<std::string, std::set<std::string>> fields{
      {"takeoff", {"altitude", "climb_speed"}}, {"hover", {"duration"}},
      {"waypoint", {"x", "y", "z", "yaw", "max_speed", "acceptance_radius"}},
      {"navigate", {"x", "y", "z", "max_speed"}},
      {"track", {"target_id", "duration", "follow_distance", "follow_height"}},
      {"search",
        {"center_x", "center_y", "area_width", "area_height", "altitude", "max_speed",
          "pattern"}},
      {"explore", {"min_bound", "max_bound", "forbidden_boxes", "return_home"}},
      {"land", {}}, {"precision_land", {"tag_id"}}};
    const auto spec = fields.find(step.type);
    require(spec != fields.end(), "unsupported step type: " + step.type);
    allowed.insert(spec->second.begin(), spec->second.end()); keys(node, allowed);
    step.timeout = optional(node, "timeout", 120, .05, 3600);
    if (step.type == "takeoff") {
      step.altitude = number(node["altitude"], .01, 50);
      step.climb_speed = optional(node, "climb_speed", 1, .01, 20);
    } else if (step.type == "waypoint" || step.type == "navigate") {
      step.x = number(node["x"], -1000, 1000); step.y = number(node["y"], -1000, 1000);
      step.z = number(node["z"], 0, 50); step.max_speed = optional(node, "max_speed", 2, .01, 20);
      if (step.type == "waypoint") {
        step.yaw = optional(node, "yaw", 0, -3.141592653589793, 3.141592653589793);
        step.acceptance_radius = optional(node, "acceptance_radius", .5, .05, 5);
      }
    } else if (step.type == "explore") {
      step.min_bound = point(node["min_bound"]); step.max_bound = point(node["max_bound"]);
      step.return_home = boolean(node["return_home"]);
      require(step.timeout >= 5 && step.timeout <= 900, "explore timeout must be 5..900s");
      for (size_t axis = 0; axis < 3; ++axis) {
        require(
          step.max_bound[axis] - step.min_bound[axis] >= .8,
          "explore extent must be at least 0.8m per axis");
      }
      require(
        step.min_bound[2] >= .5 && step.max_bound[2] <= 2.5,
        "explore vertical bounds must be within 0.5..2.5m");
      if (node["forbidden_boxes"]) {
        const auto boxes = node["forbidden_boxes"];
        require(boxes.IsSequence() && boxes.size() <= 16, "expected at most 16 forbidden boxes");
        for (const auto & box : boxes) {
          keys(box, {"min", "max"});
          ExplorationBox value{point(box["min"]), point(box["max"])};
          for (size_t axis = 0; axis < 3; ++axis) {
            require(value.min[axis] < value.max[axis], "forbidden box requires min < max");
          }
          step.forbidden_boxes.push_back(value);
        }
      }
    } else if (step.type == "precision_land") {
      step.tag_id = integer(node["tag_id"], 0, 2147483647);
    } else if (step.type == "hover") {
      step.wait = number(node["duration"], 0, 3600);
      require(step.wait <= step.timeout, "hover duration exceeds timeout");
    } else if (step.type == "track") {
      step.target_id = integer(node["target_id"], -1, 2147483647);
      step.duration = number(node["duration"], .01, 3600);
      step.follow_distance = optional(node, "follow_distance", 2, .1, 30);
      step.follow_height = optional(node, "follow_height", 0, -10, 10);
      require(step.duration <= step.timeout, "track duration exceeds timeout");
    } else if (step.type == "search") {
      step.center_x = number(node["center_x"], -1000, 1000);
      step.center_y = number(node["center_y"], -1000, 1000);
      step.area_width = number(node["area_width"], .1, 100);
      step.area_height = number(node["area_height"], .1, 100);
      step.altitude = number(node["altitude"], .5, 50);
      step.max_speed = optional(node, "max_speed", 1, .01, 20);
      require(word(node["pattern"]) == "boustrophedon", "only boustrophedon search supported");
    }
    total_budget += step.timeout;
    require(total_budget <= 7200, "mission step budgets exceed 7200s");
    result.steps.push_back(step);
  }
  return result;
}
}  // namespace
MissionValidation validateMissionText(const std::string & text)
{
  try {
    require(
      text.size() <= 65536 && text.find(
        '\0') == std::string::npos, "invalid mission size/content");
    std::istringstream lines(text); std::string first;
    while (std::getline(lines, first)) {
      first = trimLegacy(first);
      if (!first.empty() && first[0] != '#' && first != "---") {break;}
    }
    require(first.empty() || (first[0] != '{' && first[0] != '['), "JSON/flow root not supported");
    if (missionFormatName(text) == "legacy_flat") {
      std::istringstream legacy(text); return validateLegacyMission(legacy);
    }
    return yaml(text);
  } catch (const std::exception & exc) {
    return MissionValidation{{}, exc.what()};
  }
}
MissionValidation validateMissionFile(const std::string & path)
{
  const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {return MissionValidation{{}, "cannot open mission file"};}
  struct Guard {int fd; ~Guard() {close(fd);}} guard{fd};
  struct stat info {};
  if (fstat(fd, &info) || !S_ISREG(info.st_mode) || info.st_size > 65536) {
    return MissionValidation{{}, "mission must be a regular file <=64KiB"};
  }
  std::string text(65537, '\0'); size_t count = 0;
  while (count < text.size()) {
    const auto n = read(fd, text.data() + count, text.size() - count);
    if (n < 0 && errno == EINTR) {continue;}
    if (n < 0) {return MissionValidation{{}, "mission read failed"};}
    if (!n) {break;}
    count += static_cast<size_t>(n);
  }
  if (count > 65536) {return MissionValidation{{}, "mission exceeds 64KiB"};}
  text.resize(count);
  return validateMissionText(text);
}
}  // namespace easyfly_mission_fsm
