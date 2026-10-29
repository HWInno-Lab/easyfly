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

#include <gtest/gtest.h>
#include "easyfly_flight_control/flight_controller.hpp"

namespace easyfly_flight_control
{
class NavigationSessionTestPeer
{
public:
  static void waitForSession(FlightController & node, const std::string & session)
  {
    node.active_ = FlightController::TRAJECTORY_WAIT;
    node.traj_session_id_ = session;
    node.traj_goal_pending_ = false;
    node.traj_accept_time_ = node.now();
    node.traj_deadline_ = node.now() + rclcpp::Duration::from_seconds(120.0);
    node.traj_baseline_id_ = node.traj_seen_max_id_ = 10;
  }
  static void receive(FlightController & node, const std::string & session, uint32_t id)
  {
    auto command = std::make_shared<quadrotor_msgs::msg::PositionCommand>();
    command->header.frame_id = "odom";
    command->header.stamp = node.now();
    command->position.z = 1.0;
    command->navigation_session_id = session;
    command->trajectory_id = id;
    command->trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    node.cb_position_cmd(command);
  }
  static uint32_t watermark(FlightController & node) {return node.traj_seen_max_id_;}
  static bool received(FlightController & node) {return node.traj_cmd_received_.load();}
  static rclcpp_action::GoalResponse admit(FlightController & node, const std::string & session)
  {
    node.active_ = FlightController::HOVER;
    node.uav_.connected = node.uav_.armed = node.uav_.pose_valid = node.uav_.attitude_valid = true;
    node.uav_.flight_mode = 3;
    node.uav_.header.stamp = node.now();
    node.cb_uav_state(std::make_shared<easyfly_msgs::msg::UAVState>(node.uav_));
    auto goal = std::make_shared<FlightController::FollowTrajectory::Goal>();
    goal->target_z = 1.0;
    goal->navigation_session_id = session;
    return node.handle_traj_goal(rclcpp_action::GoalUUID{}, goal);
  }
};

TEST(NavigationSession, ForeignAndEmptyFramesCannotAdvanceWatermarkOrStart) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  FlightController node;
  NavigationSessionTestPeer::waitForSession(node, "new-session");
  NavigationSessionTestPeer::receive(node, "old-session", 999);
  NavigationSessionTestPeer::receive(node, "", 1000);
  NavigationSessionTestPeer::receive(node, std::string(129, 'x'), 1001);
  EXPECT_EQ(NavigationSessionTestPeer::watermark(node), 10U);
  EXPECT_FALSE(NavigationSessionTestPeer::received(node));
  NavigationSessionTestPeer::receive(node, "new-session", 11);
  EXPECT_EQ(NavigationSessionTestPeer::watermark(node), 11U);
  EXPECT_TRUE(NavigationSessionTestPeer::received(node));
  NavigationSessionTestPeer::receive(node, "old-session", 2000);
  EXPECT_EQ(NavigationSessionTestPeer::watermark(node), 11U);
}

TEST(NavigationSession, GoalSessionLengthIsBoundedAndEmptyLegacyGoalRemainsValid) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  FlightController node;
  EXPECT_EQ(
    NavigationSessionTestPeer::admit(node, std::string(129, 'x')),
    rclcpp_action::GoalResponse::REJECT);
  EXPECT_EQ(
    NavigationSessionTestPeer::admit(node, ""),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

TEST(NavigationSession, ReadOnlyStateExposesActualWatermarkAndStableControllerIdentity) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto controller = std::make_shared<FlightController>();
  auto observer = std::make_shared<rclcpp::Node>("trajectory_state_observer");
  auto client = observer->create_client<easyfly_msgs::srv::GetTrajectoryState>(
    "/flight_control/trajectory_state");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(observer);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  NavigationSessionTestPeer::waitForSession(*controller, "owned-stream");
  NavigationSessionTestPeer::receive(*controller, "owned-stream", 11);
  auto request = std::make_shared<easyfly_msgs::srv::GetTrajectoryState::Request>();
  auto first = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(first, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto state = first.get();
  EXPECT_EQ(state->highest_trajectory_id, 11U);
  EXPECT_FALSE(state->idle);
  EXPECT_EQ(state->controller_session_id.size(), 32U);
  auto second = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(second, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(second.get()->controller_session_id, state->controller_session_id);
  EXPECT_EQ(NavigationSessionTestPeer::watermark(*controller), 11U);
}
}  // namespace easyfly_flight_control
