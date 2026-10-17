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
#include <unistd.h>
#include <sys/stat.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <easyfly_msgs/srv/manage_reviewed_mission.hpp>
#include <easyfly_mission_format/mission_format.hpp>
#include "easyfly_ui_control/controller.hpp"
#include "journal.hpp"
using namespace easyfly_ui_control;  // NOLINT(build/namespaces)
using Api = easyfly_msgs::srv::ManageReviewedMission;
namespace fs = std::filesystem;
namespace
{
template<typename P>
bool until(P predicate, double seconds = 5)
{
  const auto end = steady_seconds() + seconds;
  while (steady_seconds() < end) {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}
}  // namespace
class ControlTest : public ::testing::Test
{
protected:
  fs::path root;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Service<Api>::SharedPtr service;
  std::thread thread;
  std::unique_ptr<MissionControl> control;
  std::mutex mutex;
  std::map<std::string, Api::Response> records;
  std::string instance = std::string(32, 'a');
  std::atomic<int> starts{0}, cancels{0}, reads{0}, terminal_mode{0}, malformed{0};
  std::atomic<bool> drop_start{false}, drop_cancel{false};
  std::atomic<bool> durable_start{false}, durable_cancel{false};
  std::atomic<int> refusal{0};
  std::vector<std::string> canceled_executions;

  void SetUp() override
  {
    ASSERT_NE(std::getenv("ROS_DOMAIN_ID"), nullptr);
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "113");
    char directory[] = "/tmp/easyfly-control-ros-XXXXXX";
    const auto result = mkdtemp(directory); ASSERT_NE(result, nullptr); root = result;
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("control_test_backend", options);
    service = node->create_service<Api>(
      "/platform/mission/reviewed",
      [this](std::shared_ptr<rclcpp::Service<Api>> server, std::shared_ptr<rmw_request_id_t> header,
      Api::Request::SharedPtr req) {handle(server, header, *req);});
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node); thread = std::thread([this] {executor->spin();});
  }
  void TearDown() override
  {
    control.reset();
    if (context && context->is_valid()) {context->shutdown("mock fixture complete");}
    if (thread.joinable()) {thread.join();}
    service.reset(); executor.reset(); node.reset(); context.reset();
    if (!root.empty()) {fs::remove_all(root);}
  }
  std::string journalText()
  {
    std::ifstream stream(root / "intent.json");
    return std::string((std::istreambuf_iterator<char>(stream)), {});
  }
  void handle(
    const std::shared_ptr<rclcpp::Service<Api>> & server,
    const std::shared_ptr<rmw_request_id_t> & header, const Api::Request & req)
  {
    std::lock_guard<std::mutex> lock(mutex);
    Api::Response r; r.server_instance_id = instance; r.request_capacity = 256;
    r.client_request_id = req.client_request_id;
    if (req.operation == "session") {
      ++reads; r.success = true; r.status = "session";
    } else if (req.server_instance_id != instance) {
      ++reads; r.status = "instance_mismatch";
    } else {
      if (req.operation == "start") {
        ++starts;
        durable_start = journalText().find(req.client_request_id) != std::string::npos;
        r.success = true; r.status = "accepted"; r.admission_resolved = true;
        r.admission_success = true; r.admission_status = "accepted";
        r.mission_name = req.mission_name; r.content_sha256 = req.expected_content_sha256;
        r.execution_id = new_id(); r.action_goal_id = new_id(); r.execution_pending = true;
        if (refusal) {
          r.success = false; r.admission_success = false; r.execution_pending = false;
          r.status = refusal == 1 ? "content_changed" : "rejected";
          r.admission_status = r.status; r.action_goal_id.clear();
          if (refusal == 1) {r.execution_id.clear();}
        }
        records[req.client_request_id] = r;
        if (drop_start) {return;}
      } else if (!records.count(req.client_request_id)) {
        ++reads; r.status = "not_found";
      } else {
        r = records.at(req.client_request_id);
        r.status = "running";
        if (req.operation == "cancel") {
          ++cancels; canceled_executions.push_back(req.execution_id);
          durable_cancel = journalText().find("\"cancel_dispatched\":true") != std::string::npos;
          if (req.execution_id != r.execution_id) {
            r.success = false; r.status = "execution_mismatch";
          } else {r.status = "cancel_requested";}
          if (drop_cancel) {return;}
        } else {++reads;}
        if (terminal_mode) {
          r.status = terminal_mode == 3 ? "canceled" : "succeeded";
          r.execution_pending = terminal_mode == 1;
        }
        if (malformed == 1) {r.action_goal_id = std::string(32, 'f');}
        if (malformed == 2) {r.execution_id.clear();}
        if (malformed == 3) {r.execution_pending = false; r.status = "running";}
        if (malformed == 4) {r.content_sha256 = std::string(64, 'f');}
      }
    }
    r.retained_requests = records.size();
    server->send_response(*header, r);
  }
  void create()
  {
    control = std::make_unique<MissionControl>(ControlConfig{root.string(), .15});
  }
  bool ready()
  {
    return until(
      [this] {
        const auto s = control->snapshot(); return s->session_ready && !s->request_pending;
      });
  }
  easyfly_msgs::srv::GetMissionDetails::Response review()
  {
    easyfly_msgs::srv::GetMissionDetails::Response r;
    r.success = true; r.status = "valid_snapshot"; r.mission_name = "demo.mission";
    r.content = "type=hover\nwait=0.1\n";
    r.content_sha256 = easyfly_mission_fsm::missionContentSha256(r.content); return r;
  }
  bool start()
  {
    return ready() && control->start_confirmed(
      review(), steady_seconds(), control->snapshot()->server_instance);
  }
  bool running()
  {
    return until([this] {return control->snapshot()->phase == Phase::Running;});
  }
  bool stopped()
  {
    control->stop(); return until([this] {return control->snapshot()->stopped;});
  }
};
TEST_F(ControlTest, ConstructionAndStopAreReadOnly)
{
  create(); ASSERT_TRUE(ready()); EXPECT_GT(reads, 0); EXPECT_EQ(starts, 0); EXPECT_EQ(cancels, 0);
  EXPECT_FALSE(control->cancel_confirmed());
  ASSERT_TRUE(stopped()); EXPECT_EQ(starts, 0); EXPECT_EQ(cancels, 0);
  EXPECT_FALSE(fs::exists(root / "intent.json"));
}
TEST_F(ControlTest, GuiConfirmationIsOneShotEvenAfterCompletion)
{
  refusal = 1; create(); ASSERT_TRUE(ready());
  const auto confirmation = new_id();
  ASSERT_TRUE(control->start_confirmed(review(), steady_seconds(), instance, confirmation));
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Completed;}));
  ASSERT_TRUE(ready()); const auto before = journalText();
  EXPECT_FALSE(control->start_confirmed(review(), steady_seconds(), instance, confirmation));
  EXPECT_FALSE(control->start_confirmed(review(), steady_seconds(), instance, "bad\nid"));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(starts, 1); EXPECT_EQ(cancels, 0); EXPECT_EQ(journalText(), before);
  ASSERT_TRUE(control->start_confirmed(review(), steady_seconds(), instance, new_id()));
  ASSERT_TRUE(until([this] {return starts == 2;}));
}
TEST_F(ControlTest, ExplicitOnceDurableBeforeSendAndNoAutomaticCancelOnStop)
{
  create(); ASSERT_TRUE(start()); ASSERT_TRUE(running());
  EXPECT_TRUE(durable_start); EXPECT_EQ(starts, 1); EXPECT_FALSE(
    control->snapshot()->intent->resolved);
  EXPECT_FALSE(control->start_confirmed(review(), steady_seconds(), instance));
  ASSERT_TRUE(stopped()); EXPECT_EQ(cancels, 0);
  const auto text = journalText(); EXPECT_NE(text.find("\"resolved\":false"), std::string::npos);
}
TEST_F(ControlTest, LostStartReplyRestoresByLookupWithoutReplay)
{
  drop_start = true; create(); ASSERT_TRUE(start());
  ASSERT_TRUE(until([this] {return starts == 1;})); ASSERT_TRUE(stopped()); control.reset();
  create(); ASSERT_TRUE(running());
  EXPECT_EQ(starts, 1); EXPECT_EQ(cancels, 0);
  EXPECT_FALSE(control->snapshot()->intent->execution.empty());
}
TEST_F(ControlTest, ReplyTimeoutThenLateStateRecoveryDoesNotResend)
{
  drop_start = true; create(); ASSERT_TRUE(start());
  ASSERT_TRUE(running()); EXPECT_EQ(starts, 1); EXPECT_EQ(cancels, 0);
  EXPECT_FALSE(control->snapshot()->intent->resolved);
}
TEST_F(ControlTest, CancelIsExactDurableAckNotReleaseAndStopDoesNotRetry)
{
  create(); ASSERT_TRUE(start()); ASSERT_TRUE(running());
  ASSERT_TRUE(until([this] {return !control->snapshot()->request_pending;}));
  const auto execution = control->snapshot()->intent->execution;
  ASSERT_TRUE(control->cancel_confirmed());
  ASSERT_TRUE(until([this] {return cancels == 1;}));
  EXPECT_TRUE(durable_cancel); EXPECT_FALSE(control->cancel_confirmed());
  EXPECT_FALSE(control->snapshot()->intent->resolved);
  ASSERT_TRUE(stopped()); control.reset(); create();
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::CancelPending;}));
  EXPECT_EQ(cancels, 1);
  {std::lock_guard<std::mutex> lock(mutex); ASSERT_EQ(canceled_executions.size(), 1U);
    EXPECT_EQ(canceled_executions.front(), execution);}
  terminal_mode = 3;
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Completed;}));
  EXPECT_TRUE(control->snapshot()->intent->resolved);
}
TEST_F(ControlTest, LostCancelReplyDoesNotRetryAcrossRestart)
{
  create(); ASSERT_TRUE(start()); ASSERT_TRUE(running()); drop_cancel = true;
  ASSERT_TRUE(until([this] {return !control->snapshot()->request_pending;}));
  ASSERT_TRUE(control->cancel_confirmed()); ASSERT_TRUE(until([this] {return cancels == 1;}));
  ASSERT_TRUE(stopped()); control.reset(); create();
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::CancelPending;}));
  EXPECT_EQ(cancels, 1); EXPECT_EQ(starts, 1); EXPECT_FALSE(control->cancel_confirmed());
}
TEST_F(ControlTest, PublicTerminalWaitsOwnershipAndPersistsCompletion)
{
  create(); ASSERT_TRUE(start()); ASSERT_TRUE(running()); terminal_mode = 1;
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::DrainPending;}));
  EXPECT_FALSE(control->snapshot()->intent->resolved);
  EXPECT_FALSE(control->start_confirmed(review(), steady_seconds(), instance));
  terminal_mode = 2;
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Completed;}));
  EXPECT_TRUE(control->snapshot()->intent->resolved);
  ASSERT_TRUE(stopped()); control.reset(); create(); ASSERT_TRUE(ready());
  EXPECT_EQ(control->snapshot()->phase, Phase::Completed);
  EXPECT_EQ(starts, 1);
}
TEST_F(ControlTest, ChangedInstanceAndUnknownRequestRetainFaultLock)
{
  create(); ASSERT_TRUE(start()); ASSERT_TRUE(running()); ASSERT_TRUE(stopped()); control.reset();
  {std::lock_guard<std::mutex> lock(mutex); instance = std::string(32, 'd');}
  create();
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Blocked;}));
  EXPECT_FALSE(control->snapshot()->intent->resolved); EXPECT_EQ(starts, 1);
  EXPECT_FALSE(control->cancel_confirmed()); ASSERT_TRUE(stopped()); control.reset();
  {std::lock_guard<std::mutex> lock(mutex); instance = std::string(32, 'a'); records.clear();}
  create(); ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Blocked;}));
  EXPECT_EQ(starts, 1); EXPECT_EQ(cancels, 0); EXPECT_FALSE(control->snapshot()->intent->resolved);
}
TEST_F(ControlTest, MalformedIdentityOrTerminalCannotRelease)
{
  for (int mode = 1; mode <= 4; ++mode) {
    if (mode != 1) {fs::remove(root / "intent.json");}
    malformed = 0; create(); ASSERT_TRUE(start()); ASSERT_TRUE(running()); malformed = mode;
    ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Blocked;}));
    EXPECT_FALSE(control->snapshot()->intent->resolved);
    ASSERT_TRUE(stopped()); control.reset();
  }
  EXPECT_EQ(starts, 4); EXPECT_EQ(cancels, 0);
}
TEST_F(ControlTest, InvalidOrStaleReviewCannotDispatch)
{
  create(); ASSERT_TRUE(ready()); auto r = review();
  EXPECT_FALSE(control->start_confirmed(r, steady_seconds() - 3, instance));
  EXPECT_FALSE(control->start_confirmed(r, steady_seconds(), std::string(32, 'd')));
  r.content_sha256 = std::string(64, 'f');
  ASSERT_TRUE(control->start_confirmed(r, steady_seconds(), instance));
  ASSERT_TRUE(
    until(
      [this] {
        return control->snapshot()->message.find("invalid") != std::string::npos;
      }));
  EXPECT_EQ(starts, 0); EXPECT_FALSE(fs::exists(root / "intent.json"));
}
TEST_F(ControlTest, CorruptOccupiedOrScopeChangedJournalRefusesControl)
{
  std::ofstream(root / "intent.json") << "{}"; chmod((root / "intent.json").c_str(), 0600);
  create(); ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Blocked;}));
  EXPECT_EQ(starts, 0); EXPECT_EQ(cancels, 0); control.reset(); fs::remove(root / "intent.json");
  {
    Journal owner(root.string());
    EXPECT_THROW(create(), std::runtime_error);
    owner.save(
      {"domain=114;localhost=1", instance, new_id(), "demo.mission", std::string(64, 'c'),
        "", "", false, false});
  }
  create(); ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Blocked;}));
  EXPECT_EQ(starts, 0); EXPECT_EQ(cancels, 0); EXPECT_FALSE(control->snapshot()->intent->resolved);
}
TEST_F(ControlTest, UnsafeDirectoryThrowsBeforeAnyRemoteRequestAndPreservesRecord)
{
  const Intent original{"domain=113;localhost=1", instance, new_id(), "demo.mission",
    std::string(64, 'c'), "", "", false, false};
  {Journal owner(root.string()); owner.save(original);}
  const auto before = journalText();
  ASSERT_EQ(chmod(root.c_str(), 0755), 0);
  EXPECT_THROW(create(), std::runtime_error);
  EXPECT_FALSE(control); EXPECT_EQ(starts, 0); EXPECT_EQ(cancels, 0); EXPECT_EQ(reads, 0);
  EXPECT_EQ(journalText(), before);
  ASSERT_EQ(chmod(root.c_str(), 0700), 0);
  Journal owner(root.string()); ASSERT_TRUE(owner.load());
  EXPECT_EQ(owner.load()->request, original.request); EXPECT_FALSE(owner.load()->resolved);
}
TEST_F(ControlTest, PersistFailureRefusesStartBeforeRemoteWrite)
{
  create(); ASSERT_TRUE(ready()); fs::create_directory(root / "intent.json");
  ASSERT_TRUE(control->start_confirmed(review(), steady_seconds(), instance));
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Blocked;}));
  EXPECT_EQ(starts, 0); EXPECT_EQ(cancels, 0);
}
TEST_F(ControlTest, ConcurrentConfirmationQueuesOnlyOneIntent)
{
  create(); ASSERT_TRUE(ready()); const auto r = review();
  const auto confirmed = control->snapshot()->server_instance;
  std::atomic<int> queued_count{0}; std::vector<std::thread> callers;
  for (int i = 0; i < 8; ++i) {
    callers.emplace_back(
      [&] {
        if (control->start_confirmed(r, steady_seconds(), confirmed)) {++queued_count;}
      });
  }
  for (auto & caller : callers) {
    caller.join();
  }
  EXPECT_EQ(queued_count, 1); ASSERT_TRUE(running()); EXPECT_EQ(starts, 1);
}
TEST_F(ControlTest, PersistFailureRefusesCancelBeforeRemoteWrite)
{
  create(); ASSERT_TRUE(start()); ASSERT_TRUE(running());
  ASSERT_TRUE(until([this] {return !control->snapshot()->request_pending;}));
  fs::rename(root / "intent.json", root / "original.json");
  fs::create_directory(root / "intent.json");
  ASSERT_TRUE(control->cancel_confirmed());
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Blocked;}));
  EXPECT_EQ(starts, 1); EXPECT_EQ(cancels, 0);
  EXPECT_TRUE(fs::exists(root / "original.json"));
}
TEST_F(ControlTest, DefiniteNoExecutionAndRejectedAdmissionCanResolve)
{
  refusal = 1; create(); ASSERT_TRUE(start());
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Completed;}));
  EXPECT_TRUE(control->snapshot()->intent->resolved);
  EXPECT_TRUE(control->snapshot()->intent->execution.empty());
  const auto first_request = control->snapshot()->intent->request;
  ASSERT_TRUE(stopped()); control.reset(); refusal = 2;
  create(); ASSERT_TRUE(start());
  ASSERT_TRUE(until([this] {return starts == 2;}));
  ASSERT_TRUE(
    until(
      [this, first_request] {
        const auto s = control->snapshot();
        return s->intent && s->intent->request != first_request && s->phase == Phase::Completed;
      }));
  EXPECT_TRUE(control->snapshot()->intent->resolved);
  EXPECT_FALSE(control->snapshot()->intent->execution.empty());
  EXPECT_TRUE(control->snapshot()->intent->goal.empty());
  EXPECT_EQ(starts, 2); EXPECT_EQ(cancels, 0);
}
TEST_F(ControlTest, CompletionCommitFailureCannotPublishResolvedIntent)
{
  create(); ASSERT_TRUE(start()); ASSERT_TRUE(running());
  ASSERT_TRUE(until([this] {return !control->snapshot()->request_pending;}));
  fs::rename(root / "intent.json", root / "original.json");
  fs::create_directory(root / "intent.json"); terminal_mode = 2;
  ASSERT_TRUE(until([this] {return control->snapshot()->phase == Phase::Blocked;}));
  EXPECT_EQ(control->snapshot()->status, "succeeded");
  EXPECT_FALSE(control->snapshot()->intent->resolved);
  EXPECT_EQ(starts, 1); EXPECT_EQ(cancels, 0);
}
