// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_ui_control/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#ifndef EASYFLY_UI_CONTROL__PERIPHERALS_HPP_
#define EASYFLY_UI_CONTROL__PERIPHERALS_HPP_
#include <memory>
#include <string>
#include <easyfly_msgs/msg/single_track_state.hpp>
#include <easyfly_msgs/msg/single_track_state_v2.hpp>
#include <easyfly_msgs/msg/gimbal_state.hpp>
#include <easyfly_msgs/srv/manage_single_track.hpp>
#include <easyfly_msgs/srv/set_gimbal_angle.hpp>
#include <easyfly_msgs/srv/set_gimbal_tracking.hpp>
namespace easyfly_ui_control
{
struct PeripheralConfig {bool use_sim_time{true}; bool commands_enabled{false};
  double response_timeout{2}; int perception_version{1};};
struct PeripheralSnapshot
{
  bool running{false}, stopped{false}, pending{false}, uncertain{false};
  bool single_ready{false}, angle_ready{false}, tracking_ready{false};
  bool single_fresh{false}, gimbal_fresh{false}, reply_received{false}, accepted{false};
  uint64_t sequence{0};
  double produced_steady{0}, single_receipt{0}, gimbal_receipt{0};
  int64_t ros_now_ns{0};
  std::string operation, message;
  std::shared_ptr<const easyfly_msgs::msg::SingleTrackState> single;
  std::shared_ptr<const easyfly_msgs::msg::SingleTrackStateV2> native_single;
  std::shared_ptr<const easyfly_msgs::msg::GimbalState> gimbal;
};
// Explicit opt-in, fixed peripheral endpoints only. One in-flight request, no retries.
// A timeout is unknown and blocks further submissions until the actual reply arrives.
// Replies acknowledge admission only; independent telemetry reports device/tracker state.
class PeripheralControl
{
public:
  explicit PeripheralControl(const PeripheralConfig & config = {});
  ~PeripheralControl();
  std::shared_ptr<const PeripheralSnapshot> snapshot() const;
  bool single_confirmed(const easyfly_msgs::srv::ManageSingleTrack::Request & request);
  bool angle_confirmed(const easyfly_msgs::srv::SetGimbalAngle::Request & request);
  bool tracking_confirmed(const easyfly_msgs::srv::SetGimbalTracking::Request & request);
  void inhibit(bool inhibited);
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_ui_control
#endif  // EASYFLY_UI_CONTROL__PERIPHERALS_HPP_
