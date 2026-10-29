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

#ifndef EASYFLY_UI_BRIDGE__SPECIAL_OBSERVATIONS_HPP_
#define EASYFLY_UI_BRIDGE__SPECIAL_OBSERVATIONS_HPP_
#include <array>
#include <string>
#include "easyfly_ui_bridge/model.hpp"
namespace easyfly_ui_bridge
{
namespace special
{
bool valid(const easyfly_msgs::msg::TagObservationArray &, const std::string & optical);
bool valid(const easyfly_msgs::msg::LandingTarget &, const std::string & optical);
bool valid(const easyfly_msgs::msg::LandingGuidanceTarget &, const std::string & optical);
bool valid(const easyfly_msgs::msg::MissionStepFeedback &, const std::string & optical = {});
bool valid(const easyfly_msgs::msg::TargetPredictionArray &, const std::string & optical = {});
bool valid(const easyfly_msgs::msg::ClassifiedObstacleArray &, const std::string & optical = {});
bool class_current(const easyfly_msgs::msg::ClassifiedObstacle &, int64_t now_ns);
}  // namespace special
// One worker owner; at most one bounded typed value per input, never a control publisher.
class SpecialObservationBuffer
{
public:
  explicit SpecialObservationBuffer(std::string optical);
  void clear();
  void receive(easyfly_msgs::msg::TagObservationArray::ConstSharedPtr, double now, uint64_t epoch);
  void receive(easyfly_msgs::msg::LandingTarget::ConstSharedPtr, double now, uint64_t epoch);
  void receive(easyfly_msgs::msg::LandingGuidanceTarget::ConstSharedPtr, double now,
    uint64_t epoch);
  void receive(easyfly_msgs::msg::MissionStepFeedback::ConstSharedPtr, double now, uint64_t epoch);
  void receive(easyfly_msgs::msg::TargetPredictionArray::ConstSharedPtr, double now,
    uint64_t epoch);
  void receive(easyfly_msgs::msg::ClassifiedObstacleArray::ConstSharedPtr, double now,
    uint64_t epoch);
  SpecialObservations snapshot(const ClockState &, double now) const;

private:
  std::string optical_;
  SpecialObservations data_;
  std::array<int64_t, 6> watermarks_{};
};
}  // namespace easyfly_ui_bridge
#endif  // EASYFLY_UI_BRIDGE__SPECIAL_OBSERVATIONS_HPP_
