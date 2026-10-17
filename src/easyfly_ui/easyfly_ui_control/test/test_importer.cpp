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
#include <sys/stat.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <easyfly_msgs/srv/import_mission.hpp>
#include <easyfly_msgs/srv/get_mission_details.hpp>
#include <easyfly_mission_format/mission_format.hpp>
#include "easyfly_ui_control/importer.hpp"
#include "import_journal.hpp"
namespace c = easyfly_ui_control;
using Api = easyfly_msgs::srv::ImportMission;
using Details = easyfly_msgs::srv::GetMissionDetails;
namespace fs = std::filesystem;
namespace
{
constexpr char body[] = "# exact draft\nschema_version: 1\nframe_id: odom\n"
  "units: {position: m, angle: rad, time: s}\nsteps:\n- type: hover\n  duration: 2\n";
template<typename P> bool until(P predicate, double seconds = 5)
{
  const auto end = c::steady_seconds() + seconds;
  while (c::steady_seconds() < end) {
    if (predicate()) {return true;} std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}
}  // namespace
class Importer : public ::testing::Test
{
protected:
  fs::path root;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Service<Api>::SharedPtr service;
  rclcpp::Service<Details>::SharedPtr details;
  std::thread thread;
  std::unique_ptr<c::MissionImporter> control;
  std::mutex mutex;
  std::map<std::string, Api::Response> records;
  std::vector<std::pair<std::shared_ptr<rmw_request_id_t>, Api::Response>> held;
  std::string instance = std::string(32, 'a'), catalog = std::string(32, 'b');
  std::atomic<int> creates{0}, lookups{0}, sessions{0}, reads{0};
  std::atomic<bool> drop_create{false}, drop_lookup{false}, hold_session{false},
  refuse{false}, mismatch{false}, durable{false};
  void SetUp() override
  {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "128");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
    char path[] = "/tmp/easyfly-importer-test-XXXXXX";
    const auto directory = mkdtemp(path); ASSERT_NE(directory, nullptr); root = directory;
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("import_only_mock", options);
    service = node->create_service<Api>(
      "/platform/mission/import",
      [this](std::shared_ptr<rclcpp::Service<Api>> server, std::shared_ptr<rmw_request_id_t> header,
      Api::Request::SharedPtr request) {
        std::lock_guard<std::mutex> lock(
          mutex);
        Api::Response r; r.server_instance_id = instance; r.catalog_id = catalog;
        r.catalog_directory = "/tmp/mock-import-catalog";
        r.enabled = true; r.request_capacity = 256;
        r.client_request_id = request->client_request_id;
        if (request->operation == "session") {
          ++sessions; r.status = "session"; r.success = true;
          if (hold_session) {held.emplace_back(header, r); return;}
        } else if (request->server_instance_id != instance || request->catalog_id != catalog) {
          r.status = "instance_mismatch";
        } else if (request->operation == "create") {
          ++creates;
          std::ifstream file(root / "import.json");
          const std::string bytes((std::istreambuf_iterator<char>(file)), {});
          durable = bytes.find(request->client_request_id) != std::string::npos &&
          bytes.find(
            request->expected_content_sha256) != std::string::npos;
          r.mission_name = request->mission_name;
          r.content_sha256 = request->expected_content_sha256;
          r.resolved = true; r.success = r.published = !refuse;
          r.status = refuse ? "exists" : "imported"; records[r.client_request_id] = r;
          if (drop_create) {return;}
        } else {
          ++lookups;
          if (drop_lookup) {return;}
          if (records.count(
            request->client_request_id)) {r = records[request->client_request_id];} else {
            r.status = "not_found";
          }
        }
        server->send_response(*header, r);
      });
    details = node->create_service<Details>(
      "/platform/mission/details",
      [this](Details::Request::SharedPtr request, Details::Response::SharedPtr response) {
        ++reads; response->success = true; response->status = "valid_snapshot";
        response->mission_name = request->mission_name;
        response->content = mismatch ? std::string(body) +
        "# externally changed\n" : std::string(body);
        response->content_sha256 = easyfly_mission_fsm::missionContentSha256(response->content);
      });
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo); executor->add_node(
      node);
    thread = std::thread([this] {executor->spin();}); start();
  }
  void start()
  {
    control = std::make_unique<c::MissionImporter>(c::ImportConfig{root.string(), .2});
    ASSERT_TRUE(until([&] {return control->snapshot()->ready || control->snapshot()->fault;}));
  }
  void stop()
  {
    if (control) {
      control->stop(); EXPECT_TRUE(until([&] {return control->snapshot()->stopped;}));
      control.reset();
    }
  }
  void TearDown() override
  {
    stop();
    if (executor) {executor->cancel();}
    if (thread.joinable()) {thread.join();}
    details.reset(); service.reset(); node.reset(); executor.reset();
    if (context && context->is_valid()) {context->shutdown("test end");}
    if (!root.empty()) {fs::remove_all(root);}
  }
  c::ImportDestination destination()
  {
    EXPECT_TRUE(
      until(
        [&] {
          const auto s = control->snapshot();
          if (s->destination && !s->pending) {return true;}
          if (!s->pending) {control->read_destination();} return false;
        }));
    return control->snapshot()->destination.value_or(c::ImportDestination{});
  }
  c::ImportDocument document()
  {return {"new.yaml", body, easyfly_mission_fsm::missionContentSha256(body)};}
};
TEST_F(Importer, SingleCreateIsDurableBeforeSendAndIndependentlyVerified)
{
  EXPECT_EQ(creates, 0); const auto d = destination();
  ASSERT_TRUE(control->import_confirmed(document(), d, std::string(32, 'a')));
  EXPECT_FALSE(control->import_confirmed(document(), d, std::string(32, 'b')));
  ASSERT_TRUE(until([&] {return control->snapshot()->phase == c::ImportPhase::Completed;}));
  EXPECT_TRUE(durable); EXPECT_EQ(creates, 1); EXPECT_EQ(reads, 1);
  EXPECT_TRUE(control->snapshot()->intent->resolved); EXPECT_GE(sessions, 3);
  EXPECT_FALSE(
    control->import_confirmed(
      document(), *control->snapshot()->destination,
      std::string(32, 'a')));
  stop(); c::ImportJournal j(root.string()); ASSERT_TRUE(j.load());
  EXPECT_TRUE(j.load()->resolved && j.load()->success);
}
TEST_F(Importer, ChangedDestinationAndInvalidCandidateNeverCreate)
{
  const auto d = destination(); auto bad = document(); bad.hash = std::string(64, '0');
  EXPECT_FALSE(control->import_confirmed(bad, d, std::string(32, 'a')));
  EXPECT_FALSE(control->import_confirmed(document(), d, ""));
  {std::lock_guard<std::mutex> lock(mutex); instance = std::string(32, 'f');}
  ASSERT_TRUE(control->import_confirmed(document(), d, std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_EQ(creates, 0); EXPECT_FALSE(fs::exists(root / "import.json"));
}
TEST_F(Importer, InhibitUnfinishedPrecheckPreventsCreate)
{
  const auto d = destination(); const int before = sessions; hold_session = true;
  ASSERT_TRUE(control->import_confirmed(document(), d, std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return sessions > before;})); control->inhibit(true);
  {std::lock_guard<std::mutex> lock(mutex);
    for (auto & reply : held) {
      service->send_response(*reply.first, reply.second);
    }
    held.clear();}
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_EQ(creates, 0); EXPECT_FALSE(fs::exists(root / "import.json"));
}
TEST_F(Importer, LostCreateReplyRecoversByLookupWithoutReissuingCreate)
{
  const auto d = destination(); drop_create = true;
  ASSERT_TRUE(control->import_confirmed(document(), d, std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return control->snapshot()->phase == c::ImportPhase::Completed;}));
  EXPECT_EQ(creates, 1); EXPECT_GE(lookups, 1); EXPECT_EQ(reads, 1);
}
TEST_F(Importer, RestartRestoresOnlyOriginalLookupAndNoNewSessionOrCreate)
{
  const auto d = destination(); drop_create = drop_lookup = true;
  ASSERT_TRUE(control->import_confirmed(document(), d, std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return control->snapshot()->phase == c::ImportPhase::Uncertain;}));
  const auto original = *control->snapshot()->intent; stop();
  drop_lookup = false; const int before = sessions; start();
  ASSERT_TRUE(until([&] {return control->snapshot()->phase == c::ImportPhase::Completed;}));
  EXPECT_EQ(creates, 1); EXPECT_EQ(control->snapshot()->intent->request, original.request);
  EXPECT_EQ(sessions, before + 1);  // Only final identity check, never a startup session/adoption.
}
TEST_F(Importer, NewServerInstanceKeepsOriginalFaultAndRecord)
{
  const auto d = destination(); drop_create = drop_lookup = true;
  ASSERT_TRUE(control->import_confirmed(document(), d, std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return control->snapshot()->phase == c::ImportPhase::Uncertain;}));
  const auto original = *control->snapshot()->intent; stop();
  {std::lock_guard<std::mutex> lock(mutex); instance = std::string(32, 'f');}
  drop_lookup = false; start();
  ASSERT_TRUE(until([&] {return control->snapshot()->fault;}));
  EXPECT_EQ(control->snapshot()->intent->instance, original.instance);
  EXPECT_FALSE(control->read_destination()); EXPECT_EQ(creates, 1);
  stop(); c::ImportJournal j(root.string()); EXPECT_FALSE(j.load()->resolved);
}
TEST_F(Importer, ChangedFileReadbackCannotClaimSuccessOrRollback)
{
  const auto d = destination(); mismatch = true;
  ASSERT_TRUE(control->import_confirmed(document(), d, std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return control->snapshot()->fault;}));
  EXPECT_EQ(creates, 1); EXPECT_FALSE(control->snapshot()->intent->resolved);
  EXPECT_FALSE(control->import_confirmed(document(), d, std::string(32, 'b')));
}
TEST_F(Importer, OriginalRefusalIsResolvedButNeverSuccess)
{
  const auto d = destination(); refuse = true;
  ASSERT_TRUE(control->import_confirmed(document(), d, std::string(32, 'a')));
  ASSERT_TRUE(
    until(
      [&] {
        return control->snapshot()->intent && control->snapshot()->intent->resolved;
      }));
  EXPECT_FALSE(control->snapshot()->intent->success); EXPECT_EQ(creates, 1); EXPECT_EQ(reads, 0);
}
TEST_F(Importer, CorruptJournalBlocksWithoutChangingBytesOrSendingCalls)
{
  stop(); std::ofstream(root / "import.json") << "{bad}";
  chmod((root / "import.json").c_str(), 0600);
  start(); ASSERT_TRUE(control->snapshot()->fault); EXPECT_EQ(creates + lookups + sessions, 0);
  std::ifstream file(root / "import.json");
  std::string bytes((std::istreambuf_iterator<char>(file)), {});
  EXPECT_EQ(bytes, "{bad}");
}
TEST_F(Importer, ImportJournalIsExclusiveTypedAndSeparateFromFlightJournal)
{
  stop();
  {
    c::ImportJournal j(root.string()); EXPECT_THROW(
      c::ImportJournal another(
        root.string()), std::runtime_error);
    c::ImportIntent i{"domain=128;localhost=1", instance, catalog, std::string(32, 'c'),
      document().name, document().hash, "submitted", false, false};
    j.save(i); ASSERT_TRUE(j.load()); EXPECT_EQ(j.load()->catalog, catalog);
    i.hash = "bad"; EXPECT_THROW(j.save(i), std::runtime_error);
  }
  std::ofstream(root / "intent.json") << "flight record stays";
  chmod((root / "intent.json").c_str(), 0600);
  EXPECT_THROW(c::ImportJournal invalid(root.string()), std::runtime_error);
  std::ifstream file(root / "intent.json");
  std::string bytes((std::istreambuf_iterator<char>(file)), {});
  EXPECT_EQ(bytes, "flight record stays");
}
TEST_F(Importer, ScopeChangeBlocksRestoreBeforeAnyRemoteCalls)
{
  const auto d = destination(); drop_create = drop_lookup = true;
  ASSERT_TRUE(control->import_confirmed(document(), d, std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return control->snapshot()->phase == c::ImportPhase::Uncertain;}));
  stop();
  {c::ImportJournal j(root.string()); auto i = *j.load(); i.scope = "domain=129;localhost=1";
    j.save(i);}
  const int before = creates + sessions + lookups;
  start(); ASSERT_TRUE(control->snapshot()->fault);
  EXPECT_EQ(creates + sessions + lookups, before);
  EXPECT_FALSE(control->read_destination());
}
