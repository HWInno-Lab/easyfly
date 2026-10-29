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
#include <rcl/time.h>
#include <future>
#include <limits>
#include <thread>
#include "easyfly_mavlink_bridge/mavlink_bridge.hpp"

using namespace std::chrono_literals;

namespace easyfly_mavlink_bridge
{
class BridgeFreshnessTestPeer
{
public:
  static void freeze(MavlinkBridge & node, int64_t seconds)
  {
    node.timer_state_->cancel();
    node.timer_setpoint_->cancel();
    ASSERT_EQ(rcl_enable_ros_time_override(node.get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(
      rcl_set_ros_time_override(
        node.get_clock()->get_clock_handle(), seconds * 1000000000LL), RCL_RET_OK);
  }
  static void telemetry(MavlinkBridge & node)
  {
    auto state = std::make_shared<mavros_msgs::msg::State>();
    state->connected = true;
    state->mode = "OFFBOARD";
    node.cb_state(state);
    auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose->pose.position.z = 1.0;
    node.cb_local_pose(pose);
    node.cb_local_velocity(std::make_shared<geometry_msgs::msg::TwistStamped>());
    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu->orientation.w = 1.0;
    node.cb_imu(imu);
    auto battery = std::make_shared<sensor_msgs::msg::BatteryState>();
    battery->voltage = 12.0;
    battery->percentage = 0.8;
    node.cb_battery(battery);
    node.cb_global_position(std::make_shared<sensor_msgs::msg::NavSatFix>());
    auto rc = std::make_shared<mavros_msgs::msg::RCIn>();
    rc->channels.push_back(1500);
    node.cb_rc_in(rc);
  }
  static void status(MavlinkBridge & node, int64_t stamp, const std::string & mode = "OFFBOARD")
  {
    auto message = std::make_shared<mavros_msgs::msg::State>();
    message->header.stamp = rclcpp::Time(stamp);
    message->connected = message->armed = true;
    message->mode = mode;
    node.cb_state(message);
  }
  static void pose(MavlinkBridge & node, double x)
  {
    auto message = std::make_shared<geometry_msgs::msg::PoseStamped>();
    message->pose.position.x = x;
    node.cb_local_pose(message);
  }
  static easyfly_msgs::msg::UAVState snapshot(MavlinkBridge & node)
  {
    node.publish_state();
    std::lock_guard<std::mutex> lock(node.state_mtx_);
    return node.uav_;
  }
  static void command(MavlinkBridge & node)
  {
    auto command = std::make_shared<easyfly_msgs::msg::UAVCommand>();
    command->header.frame_id = "odom";
    command->header.stamp = node.now();
    command->control_mode = command->POSITION;
    command->position_z = 1.0;
    node.cb_uav_command(command);
  }
  static void tick(MavlinkBridge & node) {node.publish_setpoint();}
  static bool modeReady(MavlinkBridge & node) {return node.client_set_mode_->service_is_ready();}
  static bool fault(MavlinkBridge & node) {return node.command_fault_;}
  static easyfly_msgs::srv::GetCapabilities::Response capabilities(MavlinkBridge & node)
  {
    return node.command_capabilities();
  }
  static size_t pending(MavlinkBridge & node)
  {
    return node.client_set_mode_->prune_pending_requests();
  }
  static easyfly_msgs::srv::SetFlightMode::Response mode(MavlinkBridge & node)
  {
    auto request = std::make_shared<easyfly_msgs::srv::SetFlightMode::Request>();
    request->flight_mode = 3;
    auto response = std::make_shared<easyfly_msgs::srv::SetFlightMode::Response>();
    node.cb_set_flight_mode(request, response);
    return *response;
  }
  static easyfly_msgs::srv::ArmMotors::Response arm(MavlinkBridge & node)
  {
    auto request = std::make_shared<easyfly_msgs::srv::ArmMotors::Request>();
    request->arm = true;
    auto response = std::make_shared<easyfly_msgs::srv::ArmMotors::Response>();
    node.cb_arm_motors(request, response);
    return *response;
  }
};

class BridgeFreshness : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_).parameter_overrides(
      {
        rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("setpoint_timeout", 0.05),
        rclcpp::Parameter("motion_freshness_s", 0.05),
        rclcpp::Parameter("slow_sensor_freshness_s", 0.05),
      });
    node_ = std::make_shared<MavlinkBridge>(options);
    BridgeFreshnessTestPeer::freeze(*node_, 10);
    rclcpp::NodeOptions peer_options;
    peer_options.context(context_);
    peer_ = std::make_shared<rclcpp::Node>("bridge_freshness_peer", peer_options);
    subscription_ = peer_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr) {++setpoints_;});
    status_subscription_ = peer_->create_subscription<easyfly_msgs::msg::FlightStatus>(
      "/flight_status", 10, [this](easyfly_msgs::msg::FlightStatus::ConstSharedPtr message) {
        status_ = *message;
        ++status_count_;
      });
    state_subscription_ = peer_->create_subscription<easyfly_msgs::msg::UAVState>(
      "/uav_state", 10, [this](easyfly_msgs::msg::UAVState::ConstSharedPtr message) {
        state_ = *message;
        ++state_count_;
      });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    executor_->add_node(peer_);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ((subscription_->get_publisher_count() == 0 ||
      status_subscription_->get_publisher_count() == 0 ||
      state_subscription_->get_publisher_count() == 0) &&
      std::chrono::steady_clock::now() < deadline) {std::this_thread::sleep_for(5ms);}
    ASSERT_GT(subscription_->get_publisher_count(), 0U);
    ASSERT_GT(status_subscription_->get_publisher_count(), 0U);
    ASSERT_GT(state_subscription_->get_publisher_count(), 0U);
  }
  void TearDown() override
  {
    executor_->cancel();
    if (context_->is_valid()) {context_->shutdown("test complete");}
  }
  void spin()
  {
    for (int i = 0; i < 15; ++i) {
      executor_->spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }
  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<MavlinkBridge> node_;
  rclcpp::Node::SharedPtr peer_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Subscription<easyfly_msgs::msg::FlightStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr state_subscription_;
  easyfly_msgs::msg::UAVState state_;
  int state_count_{0};
  easyfly_msgs::msg::FlightStatus status_;
  int status_count_{0};
  int setpoints_{0};
};

TEST_F(BridgeFreshness, UninitializedClockDoesNotPublishAnAggregateIdentity)
{
  BridgeFreshnessTestPeer::freeze(*node_, 0);
  BridgeFreshnessTestPeer::telemetry(*node_);
  BridgeFreshnessTestPeer::snapshot(*node_); spin();
  EXPECT_EQ(state_count_, 0); EXPECT_GT(status_count_, 0);
  BridgeFreshnessTestPeer::freeze(*node_, 10);
  BridgeFreshnessTestPeer::snapshot(*node_); spin();
  EXPECT_EQ(state_count_, 1);
  EXPECT_EQ(rclcpp::Time(state_.header.stamp).nanoseconds(), 10000000000LL);
  EXPECT_EQ(setpoints_, 0);
}

TEST_F(BridgeFreshness, AggregateIdentityIsUniqueAndFrozenClockDoesNotRenewReceipts)
{
  BridgeFreshnessTestPeer::pose(*node_, 1);
  BridgeFreshnessTestPeer::snapshot(*node_); spin();
  ASSERT_EQ(state_count_, 1); EXPECT_EQ(state_.position_x, 1);
  BridgeFreshnessTestPeer::pose(*node_, 2);
  for (int i = 0; i < 5; ++i) {
    BridgeFreshnessTestPeer::snapshot(*node_);
  }
  spin(); EXPECT_EQ(state_count_, 1); EXPECT_EQ(state_.position_x, 1);
  EXPECT_GE(status_count_, 6);  // Independent status receipt-age reporting remains active.
  std::this_thread::sleep_for(80ms);
  BridgeFreshnessTestPeer::snapshot(*node_); spin(); EXPECT_EQ(state_count_, 1);
  BridgeFreshnessTestPeer::freeze(*node_, 11);
  BridgeFreshnessTestPeer::pose(*node_, 3);
  BridgeFreshnessTestPeer::snapshot(*node_); spin();
  ASSERT_EQ(state_count_, 2); EXPECT_EQ(state_.position_x, 3);
  EXPECT_EQ(rclcpp::Time(state_.header.stamp).nanoseconds(), 11000000000LL);
  BridgeFreshnessTestPeer::freeze(*node_, 9);
  BridgeFreshnessTestPeer::snapshot(*node_); spin();
  ASSERT_EQ(state_count_, 3);
  EXPECT_EQ(rclcpp::Time(state_.header.stamp).nanoseconds(), 9000000000LL);
  EXPECT_FALSE(state_.pose_valid);
  EXPECT_EQ(setpoints_, 0);
}

TEST_F(BridgeFreshness, CapabilityQueriesAreReadOnlyAndCannotClearACommandFault)
{
  const auto initial = BridgeFreshnessTestPeer::capabilities(*node_);
  ASSERT_EQ(initial.names, std::vector<std::string>{"flight_command_transport"});
  ASSERT_EQ(initial.implemented, std::vector<bool>{true});
  EXPECT_EQ(initial.ready, std::vector<bool>{false});
  EXPECT_NE(initial.details.front().find("\"command_fault\":false"), std::string::npos);
  EXPECT_FALSE(BridgeFreshnessTestPeer::fault(*node_));
  // The real service callback uses the same read-only snapshot. Exercise it
  // repeatedly before and after a real unanswered request sets the fault.
  auto query = peer_->create_client<easyfly_msgs::srv::GetCapabilities>(
    "/platform/mavlink_bridge/capabilities");
  ASSERT_TRUE(query->wait_for_service(2s));
  for (int i = 0; i < 3; ++i) {
    auto future = query->async_send_request(
      std::make_shared<easyfly_msgs::srv::GetCapabilities::Request>());
    spin();
    ASSERT_EQ(future.wait_for(0ms), std::future_status::ready);
    EXPECT_EQ(future.get()->details, initial.details);
  }
  EXPECT_EQ(setpoints_, 0);
}

TEST_F(BridgeFreshness, ModeSourceDoesNotBecomeAggregatePublicationTime)
{
  BridgeFreshnessTestPeer::status(*node_, 10000000000LL);
  BridgeFreshnessTestPeer::snapshot(*node_);
  spin();
  ASSERT_GT(status_count_, 0);
  EXPECT_TRUE(status_.valid);
  EXPECT_EQ(rclcpp::Time(status_.header.stamp).nanoseconds(), 10000000000LL);
  BridgeFreshnessTestPeer::freeze(*node_, 11);
  const auto aggregate = BridgeFreshnessTestPeer::snapshot(*node_);
  spin();
  EXPECT_EQ(rclcpp::Time(aggregate.header.stamp).nanoseconds(), 11000000000LL);
  EXPECT_EQ(rclcpp::Time(status_.header.stamp).nanoseconds(), 10000000000LL);
  EXPECT_FALSE(status_.valid);
}

TEST_F(BridgeFreshness, DuplicateModeCannotRenewReceiptAndNewSourceRestoresIt)
{
  BridgeFreshnessTestPeer::status(*node_, 10000000000LL);
  std::this_thread::sleep_for(80ms);
  BridgeFreshnessTestPeer::status(*node_, 10000000000LL);
  BridgeFreshnessTestPeer::snapshot(*node_);
  spin();
  EXPECT_FALSE(status_.valid);
  EXPECT_GE(status_.receipt_age_s, .08);
  BridgeFreshnessTestPeer::status(*node_, 10000000001LL, "AUTO.LAND");
  BridgeFreshnessTestPeer::snapshot(*node_);
  spin();
  EXPECT_TRUE(status_.valid);
  EXPECT_EQ(status_.flight_mode_name, "AUTO.LAND");
  EXPECT_EQ(status_.flight_mode, 7);
}

TEST_F(BridgeFreshness, FrozenClockTelemetryExpiresAndOnlyNewInputRestoresValidity)
{
  BridgeFreshnessTestPeer::telemetry(*node_);
  auto state = BridgeFreshnessTestPeer::snapshot(*node_);
  EXPECT_TRUE(state.connected && state.pose_valid && state.velocity_valid && state.attitude_valid);
  EXPECT_TRUE(state.battery_valid && state.gps_valid && state.rc_connected);
  std::this_thread::sleep_for(80ms);
  state = BridgeFreshnessTestPeer::snapshot(*node_);
  EXPECT_FALSE(state.connected || state.pose_valid || state.velocity_valid || state.attitude_valid);
  EXPECT_FALSE(state.battery_valid || state.gps_valid);
  // RC has its own unchanged 1s timeout; expire its steady receipt too.
  std::this_thread::sleep_for(950ms);
  EXPECT_FALSE(BridgeFreshnessTestPeer::snapshot(*node_).rc_connected);
  EXPECT_EQ(node_->now().nanoseconds(), 10000000000LL);
  BridgeFreshnessTestPeer::telemetry(*node_);
  state = BridgeFreshnessTestPeer::snapshot(*node_);
  EXPECT_TRUE(state.connected && state.pose_valid && state.attitude_valid && state.rc_connected);
}

TEST_F(BridgeFreshness, FrozenClockOldSetpointStopsAndNewCommandIsRequired)
{
  BridgeFreshnessTestPeer::command(*node_);
  spin();
  ASSERT_GT(setpoints_, 0);
  std::this_thread::sleep_for(80ms);
  const int count = setpoints_;
  for (int i = 0; i < 5; ++i) {
    BridgeFreshnessTestPeer::tick(*node_);
  }
  spin();
  EXPECT_EQ(setpoints_, count);
  BridgeFreshnessTestPeer::command(*node_);
  spin();
  EXPECT_GT(setpoints_, count);
}

TEST_F(BridgeFreshness, BackwardClockInvalidatesTelemetryAndOldCommand)
{
  BridgeFreshnessTestPeer::telemetry(*node_);
  BridgeFreshnessTestPeer::command(*node_);
  spin();
  ASSERT_GT(setpoints_, 0);
  const int count = setpoints_;
  BridgeFreshnessTestPeer::freeze(*node_, 9);
  EXPECT_FALSE(BridgeFreshnessTestPeer::snapshot(*node_).pose_valid);
  BridgeFreshnessTestPeer::tick(*node_);
  spin();
  EXPECT_EQ(setpoints_, count);
}

TEST_F(BridgeFreshness, ZeroClockAndInvalidTimeoutCannotAuthorizeCommands)
{
  BridgeFreshnessTestPeer::freeze(*node_, 0);
  BridgeFreshnessTestPeer::telemetry(*node_);
  EXPECT_FALSE(BridgeFreshnessTestPeer::snapshot(*node_).connected);
  BridgeFreshnessTestPeer::command(*node_);
  spin();
  EXPECT_EQ(setpoints_, 0);
  for (const double value : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    rclcpp::NodeOptions options;
    options.context(context_).parameter_overrides({rclcpp::Parameter("setpoint_timeout", value)});
    EXPECT_THROW(MavlinkBridge invalid(options), std::invalid_argument);
  }
}

TEST_F(BridgeFreshness, ExplicitMavrosRejectionIsNotAnUnknownResult)
{
  auto mode_service = peer_->create_service<mavros_msgs::srv::SetMode>(
    "/mavros/set_mode", [](mavros_msgs::srv::SetMode::Request::SharedPtr,
    mavros_msgs::srv::SetMode::Response::SharedPtr response) {response->mode_sent = false;});
  auto arm_service = peer_->create_service<mavros_msgs::srv::CommandBool>(
    "/mavros/cmd/arming", [](mavros_msgs::srv::CommandBool::Request::SharedPtr,
    mavros_msgs::srv::CommandBool::Response::SharedPtr response) {response->success = false;});
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!BridgeFreshnessTestPeer::modeReady(*node_) &&
    std::chrono::steady_clock::now() < deadline) {std::this_thread::sleep_for(5ms);}
  ASSERT_TRUE(BridgeFreshnessTestPeer::modeReady(*node_));
  std::thread spinning([this]() {executor_->spin();});
  const auto mode = BridgeFreshnessTestPeer::mode(*node_);
  const auto arm = BridgeFreshnessTestPeer::arm(*node_);
  EXPECT_FALSE(mode.success || arm.success);
  EXPECT_NE(mode.message.find("command_result_unknown:"), 0U);
  EXPECT_NE(arm.message.find("command_result_unknown:"), 0U);
  EXPECT_FALSE(BridgeFreshnessTestPeer::fault(*node_));
  EXPECT_EQ(BridgeFreshnessTestPeer::pending(*node_), 0U);
  executor_->cancel();
  spinning.join();
}

TEST_F(BridgeFreshness, UnavailableMavrosNeverSentAndDoesNotFaultLock)
{
  const auto start = std::chrono::steady_clock::now();
  const auto response = BridgeFreshnessTestPeer::mode(*node_);
  EXPECT_FALSE(response.success);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 3500ms);
  EXPECT_NE(response.message.find("command_result_unknown:"), 0U);
  EXPECT_FALSE(BridgeFreshnessTestPeer::fault(*node_));
  EXPECT_EQ(BridgeFreshnessTestPeer::pending(*node_), 0U);
}

TEST_F(BridgeFreshness, UnansweredMavrosModeLocksBridgeAndClearsPending)
{
  // A discovered real service with an idle executor never answers the request.
  auto service = peer_->create_service<mavros_msgs::srv::SetMode>(
    "/mavros/set_mode", [](mavros_msgs::srv::SetMode::Request::SharedPtr,
    mavros_msgs::srv::SetMode::Response::SharedPtr) {});
  const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (!BridgeFreshnessTestPeer::modeReady(*node_) &&
    std::chrono::steady_clock::now() < discovery_deadline) {std::this_thread::sleep_for(5ms);}
  ASSERT_TRUE(BridgeFreshnessTestPeer::modeReady(*node_));
  auto response = BridgeFreshnessTestPeer::mode(*node_);
  EXPECT_FALSE(response.success);
  EXPECT_EQ(response.message.find("command_result_unknown:"), 0U);
  EXPECT_TRUE(BridgeFreshnessTestPeer::fault(*node_));
  EXPECT_EQ(BridgeFreshnessTestPeer::pending(*node_), 0U);
  for (int i = 0; i < 3; ++i) {
    const auto capability = BridgeFreshnessTestPeer::capabilities(*node_);
    EXPECT_EQ(capability.ready, std::vector<bool>{false});
    EXPECT_NE(capability.details.front().find("\"command_fault\":true"), std::string::npos);
    EXPECT_NE(capability.details.front().find("command_fault_locked"), std::string::npos);
  }
  EXPECT_FALSE(BridgeFreshnessTestPeer::arm(*node_).success);
  EXPECT_EQ(BridgeFreshnessTestPeer::mode(*node_).message.find("command_result_unknown:"), 0U);
  BridgeFreshnessTestPeer::command(*node_);
  const auto first_rejection = BridgeFreshnessTestPeer::capabilities(*node_);
  EXPECT_NE(
    first_rejection.details.front().find("\"fault_rejected_command_count\":1"),
    std::string::npos);
  EXPECT_EQ(
    first_rejection.details.front().find("\"last_fault_rejected_source_ns\":0"),
    std::string::npos);
  BridgeFreshnessTestPeer::command(*node_);
  const auto second_rejection = BridgeFreshnessTestPeer::capabilities(*node_);
  EXPECT_NE(
    second_rejection.details.front().find("\"fault_rejected_command_count\":2"),
    std::string::npos);
  spin();
  EXPECT_EQ(setpoints_, 0);
}

TEST_F(BridgeFreshness, ShutdownBoundsPendingMavrosWaitWithoutFurtherCommands)
{
  auto service = peer_->create_service<mavros_msgs::srv::SetMode>(
    "/mavros/set_mode", [](mavros_msgs::srv::SetMode::Request::SharedPtr,
    mavros_msgs::srv::SetMode::Response::SharedPtr) {});
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!BridgeFreshnessTestPeer::modeReady(*node_) &&
    std::chrono::steady_clock::now() < deadline) {std::this_thread::sleep_for(5ms);}
  ASSERT_TRUE(BridgeFreshnessTestPeer::modeReady(*node_));
  auto response = std::async(
    std::launch::async, [this]() {
      return BridgeFreshnessTestPeer::mode(*node_);
    });
  EXPECT_EQ(response.wait_for(80ms), std::future_status::timeout);
  context_->shutdown("pending service test shutdown");
  ASSERT_EQ(response.wait_for(500ms), std::future_status::ready);
  EXPECT_FALSE(response.get().success);
  EXPECT_TRUE(BridgeFreshnessTestPeer::fault(*node_));
  EXPECT_EQ(BridgeFreshnessTestPeer::pending(*node_), 0U);
  EXPECT_FALSE(BridgeFreshnessTestPeer::arm(*node_).success);
}
}  // namespace easyfly_mavlink_bridge
