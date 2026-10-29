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
#include <atomic>
#include <thread>
#include <chrono>
#include <easyfly_core/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <easyfly_msgs/srv/get_action_ownership.hpp>
using namespace std::chrono_literals;
using easyfly_core::Status;
using easyfly_core::Admission;
class SdkTest : public ::testing::Test
{
protected:
  using Action = easyfly_msgs::action::NavigateTo;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread thread;
  rclcpp_action::Server<Action>::SharedPtr server;
  std::vector<std::shared_ptr<void>> extra_servers;
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  std::atomic<int> mode{0}, accepted{0}, cancels{0};
  rclcpp::Publisher<easyfly_msgs::msg::UAVState>::SharedPtr publisher;
  void SetUp() override
  {
    setenv("ROS_DOMAIN_ID", "91", 1);
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions o; o.context(context);
    node = std::make_shared<rclcpp::Node>("sdk_mock_server", o);
    publisher = node->create_publisher<easyfly_msgs::msg::UAVState>("/uav_state", 10);
    server = rclcpp_action::create_server<Action>(
      node, "/navigate",
      [this](const auto &, auto) {
        if (mode == 4) {
          std::this_thread::sleep_for(
            250ms);
        }
        return mode == 1 ? rclcpp_action::GoalResponse::REJECT :
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [this](auto) {++cancels; return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto handle) {
        ++accepted;
        const auto start = std::chrono::steady_clock::now();
        auto canceled_at = std::make_shared<std::chrono::steady_clock::time_point>();
        auto timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
        *timer = node->create_wall_timer(
          10ms, [this, handle, start, canceled_at, timer] {
            auto feedback = std::make_shared<Action::Feedback>();
            feedback->distance_remaining = 4.0;
            handle->publish_feedback(feedback);
            auto result = std::make_shared<Action::Result>();
            const auto elapsed = std::chrono::steady_clock::now() - start;
            if (handle->is_canceling()) {
              if (*canceled_at == std::chrono::steady_clock::time_point{}) {
                *canceled_at = std::chrono::steady_clock::now();
              }
              if (std::chrono::steady_clock::now() - *canceled_at < 120ms) {return;}
              result->success = false; result->message = "mock_canceled"; handle->canceled(result);
            } else if (mode == 4 || mode == 5 || elapsed < 120ms) {
              return;
            } else if (mode == 2) {
              result->success = false; result->message = "false_success"; handle->succeed(result);
            } else if (mode == 3) {
              result->success = false; result->message = "mock_abort"; handle->abort(result);
            } else {result->success = true; result->message = "arrived"; handle->succeed(result);}
            (*timer)->cancel();
            timer->reset();
          });
        timers.push_back(*timer);
      });
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node);
    thread = std::thread([this] {executor->spin();});
  }
  void TearDown() override
  {
    executor->cancel(); if (thread.joinable()) {thread.join();}
    timers.clear(); extra_servers.clear(); server.reset(); publisher.reset();
    executor->remove_node(node); executor.reset(); node.reset(); context->shutdown(
      "mock test completed");
  }
  void ready(easyfly_core::Client & client)
  {
    for (int i = 0; i < 300 && !client.server_ready("navigate"); ++i) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(client.server_ready("navigate"));
  }
  template<class A> void simple(const std::string & topic)
  {
    extra_servers.push_back(
      rclcpp_action::create_server<A>(
        node, topic,
        [](const auto &, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
        [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
        [](auto handle) {
          auto result = std::make_shared<typename A::Result>();
          result->success = true; result->message = "typed_mock_success"; handle->succeed(result);
        }));
  }
};
TEST_F(SdkTest, AllSevenTypedMethodsReachTheirRealActionEndpoints) {
  simple<easyfly_msgs::action::Takeoff>("/takeoff");
  simple<easyfly_msgs::action::Land>("/land");
  simple<easyfly_msgs::action::GoToWaypoint>("/goto_waypoint");
  simple<easyfly_msgs::action::FollowTrajectory>("/follow_trajectory");
  simple<easyfly_msgs::action::TrackTarget>("/track_target");
  simple<easyfly_msgs::action::ExecuteMission>("/execute_mission");
  easyfly_core::Client client;
  for (const auto * name :
    {"takeoff", "land", "goto", "navigate", "follow", "track", "execute_mission"})
  {
    for (int i = 0; i < 300 && !client.server_ready(name); ++i) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(client.server_ready(name));
  }
  std::vector<std::shared_ptr<easyfly_core::Operation>> operations{
    client.takeoff(easyfly_msgs::action::Takeoff::Goal{}), client.land(
      easyfly_msgs::action::Land::Goal{}),
    client.go_to(easyfly_msgs::action::GoToWaypoint::Goal{}), client.navigate(Action::Goal{}),
    client.follow(easyfly_msgs::action::FollowTrajectory::Goal{}), client.track(
      easyfly_msgs::action::TrackTarget::Goal{}),
    client.execute_mission(easyfly_msgs::action::ExecuteMission::Goal{})};
  for (const auto & op : operations) {
    ASSERT_TRUE(op->wait(3)); EXPECT_TRUE(op->snapshot().success);
  }
  EXPECT_TRUE(client.close(3));
}
TEST_F(SdkTest, SearchUsesRealEndpointAndReportsTruthfulResult) {
  using Search = easyfly_msgs::action::SearchArea;
  extra_servers.push_back(
    rclcpp_action::create_server<Search>(
      node, "/search_area",
      [](const auto &, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto handle) {
        auto ticks = std::make_shared<int>(0);
        auto timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
        *timer = node->create_wall_timer(
          20ms, [handle, ticks, timer] {
            auto feedback = std::make_shared<Search::Feedback>();
            feedback->area_covered = 2; feedback->progress = .5; feedback->targets_found = 3;
            handle->publish_feedback(feedback);
            if (++*ticks < 10) {return;}
            auto result = std::make_shared<Search::Result>(); result->success = true;
            result->message = "searched";
            handle->succeed(result); (*timer)->cancel(); timer->reset();
          });
        timers.push_back(*timer);
      }));
  easyfly_core::Client client;
  for (int i = 0; i < 300 && !client.server_ready("search"); ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(client.server_ready("search"));
  easyfly_msgs::action::SearchArea::Goal goal;
  goal.area_width = goal.area_height = 2; goal.search_altitude = 1;
  auto operation = client.search(goal);
  ASSERT_TRUE(operation->wait(3));
  EXPECT_EQ(operation->snapshot().admission, Admission::Accepted);
  EXPECT_EQ(operation->snapshot().status, Status::Succeeded);
  EXPECT_DOUBLE_EQ(operation->snapshot().feedback.numbers.at("area_covered"), 2);
  EXPECT_DOUBLE_EQ(operation->snapshot().feedback.numbers.at("progress"), .5);
  EXPECT_DOUBLE_EQ(operation->snapshot().feedback.numbers.at("targets_found"), 3);
  EXPECT_TRUE(client.close(3));
}
TEST_F(SdkTest, PrecisionLandingWaitsForExactRecursiveRelease) {
  using Precision = easyfly_msgs::action::PrecisionLand;
  using Ownership = easyfly_msgs::srv::GetActionOwnership;
  std::atomic<bool> released{false};
  auto service = node->create_service<Ownership>(
    "/precision_landing/ownership",
    [&released](Ownership::Request::SharedPtr request, Ownership::Response::SharedPtr response) {
      response->goal_id = request->goal_id;
      response->state = released ? 2 : 1;
    });
  extra_servers.push_back(
    rclcpp_action::create_server<Precision>(
      node, "/precision_land",
      [](const auto &, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto handle) {
        auto ticks = std::make_shared<int>(0);
        auto timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
        *timer = node->create_wall_timer(
          20ms, [handle, ticks, timer] {
            auto f = std::make_shared<Precision::Feedback>();
            f->phase = "land_handoff"; f->target_valid = true;
            f->horizontal_error_m = .05; f->height_above_target_m = .3;
            handle->publish_feedback(f);
            if (++*ticks < 5) {return;}
            auto result = std::make_shared<Precision::Result>();
            result->success = false; result->message = "public_failure_child_pending";
            handle->abort(result); (*timer)->cancel(); timer->reset();
          });
        timers.push_back(*timer);
      }));
  easyfly_core::ClientOptions options; options.require_managed_ownership = true;
  easyfly_core::Client client(options);
  for (int i = 0; i < 300 && !client.server_ready("precision_land"); ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(client.server_ready("precision_land"));
  Precision::Goal goal; goal.tag_id = 7; goal.timeout = 5;
  auto op = client.precision_land(goal);
  for (int i = 0; i < 300 && !op->snapshot().action_terminal_received; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(op->snapshot().action_terminal_received);
  EXPECT_EQ(op->snapshot().status, Status::Failed);
  EXPECT_FALSE(op->snapshot().success);
  EXPECT_TRUE(op->snapshot().remote_execution_pending);
  EXPECT_EQ(op->snapshot().feedback.strings.at("phase"), "land_handoff");
  EXPECT_TRUE(op->snapshot().feedback.flags.at("target_valid"));
  EXPECT_FALSE(op->wait(.05));
  released = true;
  ASSERT_TRUE(op->wait(3));
  EXPECT_FALSE(op->snapshot().remote_execution_pending);
  EXPECT_TRUE(client.close(3));
}
TEST_F(SdkTest, ManagedOwnershipRequiresExactReleaseAfterRealActionTerminal) {
  using Ownership = easyfly_msgs::srv::GetActionOwnership;
  std::atomic<bool> released{false}, wrong_echo{true};
  auto service = node->create_service<Ownership>(
    "/navigation/ownership",
    [&released, &wrong_echo](Ownership::Request::SharedPtr request,
    Ownership::Response::SharedPtr response) {
      response->goal_id = wrong_echo ? std::string(32, 'f') : request->goal_id;
      response->state = released ? 2 : 1;
    });
  easyfly_core::ClientOptions options; options.require_managed_ownership = true;
  options.max_pending_operations = 1;
  easyfly_core::Client client(options);ready(client);
  auto operation = client.navigate(Action::Goal{});
  for (int i = 0; i < 300 && !operation->snapshot().action_terminal_received; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(operation->snapshot().action_terminal_received);
  EXPECT_EQ(operation->snapshot().status, Status::Succeeded);
  EXPECT_TRUE(operation->snapshot().success);
  EXPECT_TRUE(operation->snapshot().remote_execution_pending);
  EXPECT_EQ(operation->snapshot().goal_id.size(), 32u);
  EXPECT_GT(operation->snapshot().result_received_seconds, 0);
  EXPECT_FALSE(operation->wait(.01));
  EXPECT_EQ(client.navigate(Action::Goal{})->snapshot().status, Status::CapacityExceeded);
  released = true;
  EXPECT_FALSE(operation->wait(.4));  // RELEASED for another UUID is not ours
  wrong_echo = false;
  EXPECT_FALSE(client.close(0));  // retains operation while managed drain still owns DDS clients
  ASSERT_TRUE(operation->wait(3));
  EXPECT_FALSE(operation->snapshot().remote_execution_pending);
  EXPECT_TRUE(client.close(0));  // cleanup barrier includes the ownership client
  EXPECT_TRUE(operation->wait(0));
}
TEST_F(SdkTest, RequiredOwnershipMissingMeansNotReadyAndNoGoalSent) {
  easyfly_core::ClientOptions options;options.require_managed_ownership = true;
  easyfly_core::Client client(options);
  std::this_thread::sleep_for(100ms);
  EXPECT_FALSE(client.server_ready("navigate"));
  auto operation = client.navigate(Action::Goal{});
  ASSERT_TRUE(operation->wait(3));
  EXPECT_EQ(operation->snapshot().status, Status::Unavailable);
  EXPECT_EQ(accepted.load(), 0);
  EXPECT_TRUE(client.close(3));
}
TEST_F(SdkTest, SuccessFalseSuccessRejectAbortAndWaitAreDistinct) {
  easyfly_core::Client client; ready(client);
  Action::Goal goal; goal.target_x = 1; goal.target_z = 1;
  auto op = client.navigate(goal);
  EXPECT_FALSE(op->wait(.001)); EXPECT_EQ(cancels.load(), 0);
  ASSERT_TRUE(op->wait(3)); EXPECT_TRUE(op->snapshot().success);
  EXPECT_EQ(op->snapshot().admission, Admission::Accepted);
  EXPECT_DOUBLE_EQ(op->snapshot().feedback.numbers.at("distance_remaining"), 4.0);
  mode = 2; op = client.navigate(goal); ASSERT_TRUE(op->wait(3));
  EXPECT_EQ(op->snapshot().status, Status::Failed); EXPECT_FALSE(op->snapshot().success);
  mode = 1; op = client.navigate(goal); ASSERT_TRUE(op->wait(3));
  EXPECT_EQ(op->snapshot().status, Status::Rejected);
  mode = 3; op = client.navigate(goal); ASSERT_TRUE(op->wait(3));
  EXPECT_EQ(op->snapshot().status, Status::Failed);
  EXPECT_EQ(client.unsupported("precision_landing")->snapshot().status, Status::Unsupported);
  easyfly_msgs::action::Land::Goal land;
  op = client.land(land); ASSERT_TRUE(op->wait(3)); EXPECT_EQ(
    op->snapshot().status, Status::Unavailable);
  EXPECT_TRUE(client.close(3));
}
TEST_F(SdkTest, CancelAcknowledgmentIsNotTerminalAndCapacityIsBounded) {
  easyfly_core::ClientOptions options; options.max_pending_operations = 1;
  easyfly_core::Client client(options); ready(client); mode = 5;
  auto op = client.navigate(Action::Goal{});
  auto rejected = client.navigate(Action::Goal{});
  EXPECT_EQ(rejected->snapshot().status, Status::CapacityExceeded);
  for (int i = 0; i < 300 && op->snapshot().admission != Admission::Accepted; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(op->snapshot().admission, Admission::Accepted);
  EXPECT_TRUE(op->cancel());
  for (int i = 0; i < 100 && op->snapshot().cancel_status != easyfly_core::CancelStatus::Accepted;
    ++i)
  {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(op->snapshot().cancel_status, easyfly_core::CancelStatus::Accepted);
  EXPECT_TRUE(op->snapshot().remote_execution_pending); EXPECT_FALSE(op->snapshot().success);
  ASSERT_TRUE(op->wait(3)); EXPECT_EQ(op->snapshot().status, Status::Canceled);
  EXPECT_EQ(cancels.load(), 1); EXPECT_TRUE(client.close(3));
}
TEST_F(SdkTest, CloseAndDestructorRetainLateAcceptanceUntilExactCancellationCompletes) {
  auto client = std::make_unique<easyfly_core::Client>(); ready(*client); mode = 4;
  auto op = client->navigate(Action::Goal{});
  std::this_thread::sleep_for(50ms);
  EXPECT_FALSE(client->close(.001));
  EXPECT_EQ(op->snapshot().local_status, easyfly_core::LocalStatus::Closed);
  EXPECT_TRUE(op->snapshot().remote_execution_pending);
  client.reset();  // detached self-owned drain contains no facade pointer
  ASSERT_TRUE(op->wait(3)); EXPECT_EQ(op->snapshot().status, Status::Canceled);
  EXPECT_EQ(accepted.load(), 1); EXPECT_EQ(cancels.load(), 1);
}
TEST_F(SdkTest, ClosedLastOperationWaitIncludesCleanupWithoutInventingTerminalStatus) {
  easyfly_core::Client client; ready(client); mode = 4;
  auto op = client.navigate(Action::Goal{});
  std::this_thread::sleep_for(50ms);
  EXPECT_FALSE(client.close(0));
  EXPECT_FALSE(op->wait(0));
  EXPECT_EQ(op->snapshot().status, Status::Pending);
  EXPECT_TRUE(op->snapshot().remote_execution_pending);
  ASSERT_TRUE(op->wait(3));
  EXPECT_EQ(op->snapshot().status, Status::Canceled);
  EXPECT_FALSE(op->snapshot().success);
  EXPECT_FALSE(op->snapshot().remote_execution_pending);
  // wait has synchronized final DDS cleanup, not merely cancel/result receipt.
  EXPECT_TRUE(client.close(0));
  EXPECT_TRUE(op->wait(0));
}
TEST_F(SdkTest, AdmissionTimeoutIsLocalAndStateFreezesAtClose) {
  easyfly_core::ClientOptions options; options.admission_timeout_seconds = .02;
  options.cancel_on_admission_timeout = true;
  easyfly_core::Client client(options); ready(client);
  easyfly_msgs::msg::UAVState state; state.connected = true; state.pose_valid = true;
  state.position_x = 7;
  state.header.frame_id = "odom";
  for (int i = 0; i < 100 && !client.state().received; ++i) {
    state.header.stamp = node->now(); publisher->publish(state); std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(client.state().received); EXPECT_TRUE(client.state().fresh);
  mode = 4; auto op = client.navigate(Action::Goal{});
  std::this_thread::sleep_for(80ms);
  EXPECT_EQ(op->snapshot().local_status, easyfly_core::LocalStatus::AdmissionTimedOut);
  EXPECT_TRUE(op->snapshot().remote_execution_pending);
  ASSERT_TRUE(op->wait(3)); EXPECT_EQ(op->snapshot().status, Status::Canceled);
  EXPECT_TRUE(client.close(3));
  state.position_x = 9; publisher->publish(state); std::this_thread::sleep_for(50ms);
  EXPECT_DOUBLE_EQ(client.state().value.position_x, 7); EXPECT_FALSE(client.state().fresh);
}
