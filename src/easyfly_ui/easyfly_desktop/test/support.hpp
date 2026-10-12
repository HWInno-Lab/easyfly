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

#ifndef SUPPORT_HPP_
#define SUPPORT_HPP_
#include <QApplication>
#include <QFont>
#include <QTest>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "easyfly_desktop/main_window.hpp"
namespace desktop_test
{
namespace ui = easyfly_desktop;
namespace b = easyfly_ui_bridge;
inline void application()
{
  static int argc = 1;
  static char name[] = "easyfly_desktop_test";
  static char * argv[] = {name, nullptr};
  static QApplication app(argc, argv);
  app.setQuitOnLastWindowClosed(false); app.setFont(QFont("Noto Sans CJK SC", 10));
}
template<typename Predicate>
bool until(Predicate predicate, int milliseconds = 3000)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
  while (std::chrono::steady_clock::now() < deadline) {
    QCoreApplication::processEvents(); if (predicate()) {return true;} QTest::qWait(5);
  }
  QCoreApplication::processEvents(); return predicate();
}
inline b::Evidence evidence(b::Quality quality = b::Quality::ValidAtObservation)
{
  b::Evidence e; e.quality = quality;
  e.received_steady = e.first_source_steady = b::detail::steady_seconds();
  e.source_age_available = true;
  return e;
}
inline std::shared_ptr<b::UiSnapshot> sample()
{
  auto s = std::make_shared<b::UiSnapshot>(); s->observation_session_id = 101; s->running = true;
  s->produced_steady = b::detail::steady_seconds(); s->selected_profile = "navigation_dev";
  s->clock = {b::Quality::ValidAtObservation, 10000000000LL, 0};
  s->state.evidence = evidence(); s->state.sdk.received = s->state.sdk.fresh = true;
  s->state.sdk.source_time_valid = true;
  auto & v = s->state.sdk.value; v.header.frame_id = "odom"; v.header.stamp.sec = 10;
  v.connected = v.pose_valid = v.velocity_valid = v.battery_valid = true;
  v.flight_mode_name = "OFFBOARD"; v.position_x = 1.25; v.position_y = -.5; v.position_z = 1.5;
  v.battery_remaining = .6; v.battery_voltage = 12;
  s->state.position_valid = s->state.velocity_valid = s->state.battery_valid = true;
  s->state.speed_mps = 0.2; s->state.battery_percent = 60;
  for (auto key :
    {b::Key::FlightCapabilities, b::Key::MissionCapabilities, b::Key::PerceptionCapabilities,
      b::Key::LandingCapabilities})
  {
    auto caps = std::make_shared<easyfly_msgs::srv::GetCapabilities::Response>();
    caps->header = v.header; caps->names = {"takeoff"}; caps->implemented = {true};
    caps->ready = {false};
    caps->details = {"{\"command_uncertain\":false}"};
    s->capabilities[key] = {caps, evidence()};
  }
  for (auto key : {b::Key::MissionList, b::Key::MissionValidation, b::Key::MissionQuery}) {
    s->missions[key] = {};
  }
  for (auto key :
    {b::Key::Profile, b::Key::UseSimTime, b::Key::OperatorNote, b::Key::ExportPretty})
  {
    s->parameters[key] = {};
  }
  for (auto key : {b::Key::Targets, b::Key::Obstacles}) {
    auto p = std::make_shared<easyfly_msgs::srv::GetPerception::Response>();
    p->status = p->VALID; p->observed_at = v.header.stamp;
    if (key == b::Key::Targets) {p->targets.header = v.header;} else {
      p->obstacles.header = v.header;
    }
    s->perception[key] = {p, evidence()};
  }
  for (int i = 0; i < 30; ++i) {
    s->trajectory.push_back({i, s->produced_steady, i * .04, i * -.02, 1.5});
  }
  return s;
}
struct FakeState
{
  std::shared_ptr<b::UiSnapshot> data{sample()};
  uint64_t sequence{0}; int stop_calls{0}, list_calls{0}, validate_calls{0}, query_calls{0};
  bool answer{true}, wrong_id{false}, unknown_id{false}, frozen_snapshot{false};
  int stop_delay_ms{0}; double stop_requested{0};
  std::string last_validation, last_query;
};
class FakeSource final : public ui::Source
{
public:
  explicit FakeSource(std::shared_ptr<FakeState> state)
  : state_(std::move(state)) {}
  std::shared_ptr<const b::UiSnapshot> snapshot() const override
  {
    auto result = std::make_shared<b::UiSnapshot>(*state_->data);
    const double now = b::detail::steady_seconds();
    if (!state_->frozen_snapshot) {result->produced_steady = now;}
    if (state_->stop_requested > 0 &&
      (now - state_->stop_requested) * 1000 >= state_->stop_delay_ms)
    {
      result->running = false; result->stopped = true;
      state_->data->running = false; state_->data->stopped = true;
    }
    const auto age = [now](b::Evidence & e) {
        if (e.received_steady > 0) {
          e.receipt_age = now - e.received_steady;
        }
      };
    age(result->state.evidence); for (auto & v : result->images) {age(v.evidence);}
    for (auto & v : result->perception) {age(v.second.evidence);}
    for (auto & v : result->capabilities) {age(v.second.evidence);}
    for (auto & v : result->missions) {age(v.second.evidence);}
    for (auto & v : result->parameters) {age(v.second.evidence);}
    return result;
  }
  b::RequestReceipt capabilities(b::CapabilitySource) override
  {
    return {++state_->sequence, b::RequestStatus::Queued};
  }
  b::RequestReceipt list_missions() override
  {
    ++state_->list_calls; const auto id = ++state_->sequence;
    if (state_->answer) {
      auto r = std::make_shared<easyfly_msgs::srv::ManageMission::Response>(); r->success = true;
      r->status = "listed"; r->missions = {"demo.yaml"};
      state_->data->missions[b::Key::MissionList] = {r, evidence()};
      state_->data->missions[b::Key::MissionList].evidence.request_id = id;
    }
    return {id, b::RequestStatus::Queued};
  }
  b::RequestReceipt validate_mission(const std::string & name) override
  {
    ++state_->validate_calls; state_->last_validation = name; const auto id = ++state_->sequence;
    auto r = std::make_shared<easyfly_msgs::srv::ManageMission::Response>(); r->success = true;
    r->status = "valid";
    state_->data->missions[b::Key::MissionValidation] = {r, evidence()};
    state_->data->missions[b::Key::MissionValidation].evidence.request_id = id;
    return {id, b::RequestStatus::Queued};
  }
  b::RequestReceipt query_mission(const std::string & execution) override
  {
    ++state_->query_calls; state_->last_query = execution; const auto id = ++state_->sequence;
    if (state_->answer) {
      auto r = std::make_shared<easyfly_msgs::srv::ManageMission::Response>();
      r->success = !state_->unknown_id;
      r->status = state_->unknown_id ? "not_found" : "failed"; r->execution_pending = true;
      r->execution_id = state_->wrong_id ? std::string(32, 'b') : execution;
      r->message = "<b>失败原因</b> & plain text";
      state_->data->missions[b::Key::MissionQuery] = {r, evidence(
          state_->unknown_id ? b::Quality::Unknown : b::Quality::ValidAtObservation)};
      state_->data->missions[b::Key::MissionQuery].evidence.request_id = id;
    }
    return {id, b::RequestStatus::Queued};
  }
  b::RequestReceipt read_parameter(b::Parameter p) override
  {
    const auto id = ++state_->sequence;
    const auto key = p == b::Parameter::Profile ? b::Key::Profile : p ==
      b::Parameter::UseSimTime ? b::Key::UseSimTime : p ==
      b::Parameter::OperatorNote ? b::Key::OperatorNote : b::Key::ExportPretty;
    auto r = std::make_shared<easyfly_msgs::srv::QueryParams::Response>(); r->success = true;
    r->current_value = p == b::Parameter::Profile ? "\"navigation_dev\"" : p ==
      b::Parameter::OperatorNote ? "\"<script>plain</script>\"" : "false";
    state_->data->parameters[key] = {r, evidence()};
    state_->data->parameters[key].evidence.request_id = id;
    return {id, b::RequestStatus::Queued};
  }
  b::RequestReceipt perception(b::PerceptionKind) override
  {
    return {++state_->sequence, b::RequestStatus::Queued};
  }
  void stop() override {++state_->stop_calls; state_->stop_requested = b::detail::steady_seconds();}

private:
  std::shared_ptr<FakeState> state_;
};
inline ui::SourceFactory factory(std::shared_ptr<FakeState> state)
{
  return [state](const b::BridgeConfig &, std::shared_ptr<const b::UiSnapshot> history) {
           if (state->data->stopped) {
             const auto next = state->data->observation_session_id + 1; state->data = sample();
             state->data->observation_session_id = next;
             if (history) {
               state->data->command_uncertain_observed = history->command_uncertain_observed;
               state->data->events = history->events;
             }
             state->stop_requested = 0;
           }
           return std::make_unique<FakeSource>(state);
         };
}
}  // namespace desktop_test
#endif  // SUPPORT_HPP_
