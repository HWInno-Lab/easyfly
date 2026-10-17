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

#include <openssl/sha.h>

#include <array>
#include <cstdint>
#include <set>
#include <sstream>
#include <stdexcept>

#include "easyfly_mission_format/mission_format.hpp"
namespace easyfly_mission_fsm
{
bool missionTextUtf8(const std::string & text)
{
  for (std::size_t i = 0; i < text.size(); ) {
    const auto first = static_cast<unsigned char>(text[i++]);
    if (first < 128) {if (first == 0) {return false;} continue;}
    int extra = 0; uint32_t code = 0, minimum = 0;
    if (first >= 0xc2 && first <= 0xdf) {
      extra = 1; code = first & 31; minimum = 128;
    } else if (first >= 0xe0 && first <= 0xef) {
      extra = 2; code = first & 15; minimum = 2048;
    } else if (first >= 0xf0 && first <= 0xf4) {
      extra = 3; code = first & 7; minimum = 65536;
    } else {return false;}
    while (extra--) {
      if (i == text.size()) {return false;}
      const auto next = static_cast<unsigned char>(text[i++]);
      if ((next & 0xc0) != 0x80) {return false;} code = (code << 6) | (next & 63);
    }
    if (code < minimum || code > 0x10ffff || (code >= 0xd800 && code <= 0xdfff)) {return false;}
  }
  return true;
}
std::string missionFormatName(const std::string & text)
{
  std::istringstream lines(text); std::string first;
  while (std::getline(lines, first)) {
    first = trimLegacy(first);
    if (!first.empty() && first[0] != '#' && first != "---") {break;}
  }
  auto separator = first.find('=');
  if (separator == std::string::npos) {separator = first.find(':');}
  const std::set<std::string> legacy_keys{"type", "altitude", "climb_speed",
    "x", "y", "z", "yaw", "max_speed", "wait"};
  return separator != std::string::npos &&
         legacy_keys.count(trimLegacy(first.substr(0, separator))) ? "legacy_flat" : "yaml_v1";
}
std::string missionContentSha256(const std::string & text)
{
  std::array<unsigned char, SHA256_DIGEST_LENGTH> bytes{};
  if (!SHA256(reinterpret_cast<const unsigned char *>(text.data()), text.size(), bytes.data())) {
    throw std::runtime_error("mission content digest failed");
  }
  const char digits[] = "0123456789abcdef"; std::string result;
  result.reserve(64);
  for (auto byte : bytes) {
    result += digits[byte >> 4]; result += digits[byte & 15];
  }
  return result;
}
}  // namespace easyfly_mission_fsm
