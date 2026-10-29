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
#ifndef EASYFLY_DESKTOP__PRESENTATION_HPP_
#define EASYFLY_DESKTOP__PRESENTATION_HPP_
#include <QString>
#include <optional>
#include <string>
#include "easyfly_desktop/source.hpp"
namespace easyfly_desktop
{
QString text(const std::string & value, std::size_t limit = 4096);
QString quality_text(bridge::Quality quality);
QString number(double value, int precision = 2);
QString missing_mission_backends(const bridge::UiSnapshot & snapshot, double now);
QString mission_step_parameters(const easyfly_mission_fsm::MissionStep & step);
struct FlightInstruments
{
  bool attitude_valid{false}, position_valid{false};
  double roll_degrees{0}, pitch_degrees{0}, heading_degrees{0};
};
// ENU yaw=0 is +X/east; compass heading is clockwise from local +Y/north.
std::optional<double> enu_heading_degrees(double yaw_radians);
FlightInstruments flight_instruments(const bridge::UiSnapshot & snapshot, double now);
double snapshot_delay(const bridge::UiSnapshot & snapshot, double now);
bridge::Quality display_quality(
  const bridge::Evidence & evidence,
  const bridge::UiSnapshot & snapshot, double now, double ttl, bool clock_required);
QString ages(const bridge::Evidence & evidence, const bridge::UiSnapshot & snapshot, double now);
QString phase_event(const bridge::UiSnapshot & snapshot, double now);
QString mission_result(
  const bridge::Observation<easyfly_msgs::srv::ManageMission::Response> & result,
  const bridge::UiSnapshot & snapshot, double now, const std::string & requested_id);
bool geometry_current(const bridge::UiSnapshot & snapshot, bridge::Key key, double now);
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__PRESENTATION_HPP_
