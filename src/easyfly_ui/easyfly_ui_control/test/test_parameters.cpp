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
#include <chrono>
#include <cstdlib>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <easyfly_msgs/srv/query_params.hpp>
#include "easyfly_ui_control/parameters.hpp"
namespace p = easyfly_ui_control;
using Api = easyfly_msgs::srv::QueryParams;
namespace
{
template<typename Predicate> bool until(Predicate predicate, double seconds = 4)
{
  const auto end = p::parameter_steady_seconds() + seconds;
  while (p::parameter_steady_seconds() < end) {
    if (predicate()) {return true;} std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}
}  // namespace
TEST(ParameterValues, TypedJsonBoundsAndEmptyStringAreUnambiguous)
{
  EXPECT_EQ(p::parameter_json(p::LocalParameter::OperatorNote, std::string{}), "\"\"");
  EXPECT_EQ(p::parameter_json(p::LocalParameter::ExportPretty, true), "true");
  EXPECT_TRUE(p::parameter_value(p::LocalParameter::OperatorNote, "\"你好\""));
  EXPECT_FALSE(p::parameter_value(p::LocalParameter::OperatorNote, "true"));
  for (const auto * bad :
    {"1", "null", "[]", "{}", "\"false\"", "true trailing", "/*x*/true", "NaN"})
  {
    EXPECT_FALSE(p::parameter_value(p::LocalParameter::ExportPretty, bad)) << bad;
  }
  EXPECT_NO_THROW(p::parameter_json(p::LocalParameter::OperatorNote, std::string(512, 'a')));
  std::string multibyte;
  for (int i = 0; i < 170; ++i) {
    multibyte += "中";
  }
  EXPECT_NO_THROW(p::parameter_json(p::LocalParameter::OperatorNote, multibyte + "ab"));
  EXPECT_THROW(
    p::parameter_json(p::LocalParameter::OperatorNote, multibyte + "中"),
    std::invalid_argument);
  EXPECT_THROW(
    p::parameter_json(
      p::LocalParameter::OperatorNote, std::string(
        513,
        'a')),
    std::invalid_argument);
  EXPECT_THROW(
    p::parameter_json(
      p::LocalParameter::OperatorNote, std::string(
        1,
        '\0')),
    std::invalid_argument);
  EXPECT_THROW(p::parameter_json(p::LocalParameter::OperatorNote, false), std::invalid_argument);
  EXPECT_THROW(p::parameter_json(static_cast<p::LocalParameter>(99), false), std::invalid_argument);
  EXPECT_THROW(p::ParameterControl({.001}), std::invalid_argument);
  EXPECT_THROW(
    p::ParameterControl(
      {std::numeric_limits<double>::quiet_NaN()}), std::invalid_argument);
}
class Parameters : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Service<Api>::SharedPtr service;
  std::unique_ptr<p::ParameterControl> control;
  std::thread thread;
  std::mutex mutex;
  std::map<std::string, std::string> values{{"operator_note", "\"old\""},
    {"export_pretty", "false"}};
  std::vector<std::pair<std::shared_ptr<rmw_request_id_t>, Api::Response>> held;
  std::atomic<int> reads{0}, writes{0}, foreign{0};
  std::atomic<bool> hold_reads{false}, hold_write{false}, reject{false}, mismatch{false},
  invalid_read{false};
  void SetUp() override
  {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "123"); ASSERT_STREQ(
      std::getenv(
        "ROS_LOCALHOST_ONLY"), "1");
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("parameter_only_fixture", options);
    service = node->create_service<Api>(
      "/platform/params/query",
      [this](std::shared_ptr<rclcpp::Service<Api>>, std::shared_ptr<rmw_request_id_t> header,
      Api::Request::SharedPtr req) {
        Api::Response result; std::lock_guard<std::mutex> lock(mutex);
        if (!values.count(req->param_name)) {
          ++foreign; result.success = false;
        } else if (req->param_value.empty()) {
          ++reads; result.success = true;
          result.current_value = invalid_read ? "wrong JSON" : values[req->param_name];
        } else {
          ++writes; result.success = !reject;
          if (!reject) {values[req->param_name] = mismatch ? "\"concurrent\"" : req->param_value;}
          result.current_value = "not readback; ignore write payload";
        }
        if ((req->param_value.empty() && hold_reads) || (!req->param_value.empty() && hold_write)) {
          held.emplace_back(header, result); return;
        }
        service->send_response(*header, result);
      });
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo); executor->add_node(
      node);
    thread = std::thread([this] {executor->spin();});
    control = std::make_unique<p::ParameterControl>(p::ParameterConfig{.3});
    ASSERT_TRUE(until([&] {return control->snapshot()->ready;}));
  }
  void TearDown() override
  {
    if (control) {
      control->stop(); EXPECT_TRUE(
        until(
          [&] {
            return control->snapshot()->stopped;
          })); control.reset();
    }
    if (executor) {executor->cancel();}
    if (thread.joinable()) {thread.join();}
    service.reset(); node.reset(); executor.reset(); if (context) {context->shutdown("test end");}
    EXPECT_EQ(foreign, 0);
  }
  p::ParameterReview read(p::LocalParameter parameter = p::LocalParameter::OperatorNote)
  {
    EXPECT_TRUE(
      until(
        [&] {
          auto s = control->snapshot();
          if (s->review && s->review->parameter == parameter && !s->pending) {return true;}
          if (!s->pending) {control->read(parameter);} return false;
        }));
    const auto s = control->snapshot(); return s->review.value_or(p::ParameterReview{});
  }
  void release()
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto & item : held) {service->send_response(*item.first, item.second);} held.clear();
  }
};
TEST_F(Parameters, SingleTypedWriteIndependentReadbackAndConsumedConfirmation)
{
  EXPECT_EQ(writes, 0); const auto review = read();
  ASSERT_TRUE(control->write_confirmed(review, std::string("新备注"), std::string(32, 'a')));
  EXPECT_FALSE(control->write_confirmed(review, std::string("duplicate"), std::string(32, 'b')));
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_EQ(control->snapshot()->phase, p::ParameterPhase::Verified); EXPECT_EQ(writes, 1);
  EXPECT_GE(reads, 3);
  EXPECT_TRUE(control->snapshot()->acknowledged); EXPECT_FALSE(control->snapshot()->fault);
  EXPECT_FALSE(
    control->write_confirmed(
      *control->snapshot()->review, std::string("repeat"),
      std::string(32, 'a')));
  const auto boolean = read(p::LocalParameter::ExportPretty);
  ASSERT_TRUE(control->write_confirmed(boolean, true, std::string(32, 'c')));
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;})); EXPECT_EQ(writes, 2);
  EXPECT_EQ(control->snapshot()->phase, p::ParameterPhase::Verified);
  EXPECT_EQ(control->snapshot()->review->json, "true");
}
TEST_F(Parameters, ChangedOldValueRejectsBeforeWrite)
{
  const auto review = read();
  {std::lock_guard<std::mutex> lock(mutex); values["operator_note"] = "\"external\"";}
  ASSERT_TRUE(control->write_confirmed(review, std::string("proposal"), std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_EQ(writes, 0); EXPECT_EQ(control->snapshot()->phase, p::ParameterPhase::Rejected);
  EXPECT_EQ(control->snapshot()->review->json, "\"external\""); EXPECT_FALSE(
    control->snapshot()->dispatched);
}
TEST_F(Parameters, EmptyNoteIsOneWriteNotARead)
{
  const auto review = read();
  ASSERT_TRUE(control->write_confirmed(review, std::string{}, std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_EQ(writes, 1);
  EXPECT_EQ(control->snapshot()->review->json, "\"\"");
  EXPECT_EQ(control->snapshot()->phase, p::ParameterPhase::Verified);
}
TEST_F(Parameters, WriteTimeoutDoesNotReplayOrUnlockOnMatchingRead)
{
  const auto review = read(); hold_write = true;
  ASSERT_TRUE(control->write_confirmed(review, std::string("proposal"), std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return control->snapshot()->fault;})); EXPECT_EQ(writes, 1);
  EXPECT_FALSE(control->snapshot()->acknowledged);
  ASSERT_TRUE(control->read(p::LocalParameter::OperatorNote));
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_EQ(control->snapshot()->review->json, "\"proposal\""); EXPECT_TRUE(
    control->snapshot()->fault);
  EXPECT_FALSE(
    control->write_confirmed(
      *control->snapshot()->review, std::string("retry"),
      std::string(32, 'b')));
  release(); std::this_thread::sleep_for(std::chrono::milliseconds(30)); EXPECT_TRUE(
    control->snapshot()->fault); EXPECT_EQ(writes, 1);
}
TEST_F(Parameters, SuccessReplyWithDifferentReadbackRemainsFaultLocked)
{
  const auto review = read(); mismatch = true;
  ASSERT_TRUE(control->write_confirmed(review, std::string("proposal"), std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return control->snapshot()->fault;}));
  EXPECT_TRUE(control->snapshot()->acknowledged); EXPECT_EQ(
    control->snapshot()->phase, p::ParameterPhase::Uncertain);
  EXPECT_EQ(writes, 1);
}
TEST_F(Parameters, DisconnectAfterDispatchKeepsLockEvenWithSuccessfulReadback)
{
  const auto review = read(); hold_write = true;
  ASSERT_TRUE(control->write_confirmed(review, std::string("proposal"), std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return writes == 1;}));
  control->inhibit(true);
  EXPECT_TRUE(control->snapshot()->fault);
  control->inhibit(false); release();
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_TRUE(control->snapshot()->acknowledged);
  EXPECT_EQ(control->snapshot()->review->json, "\"proposal\"");
  EXPECT_EQ(control->snapshot()->phase, p::ParameterPhase::Uncertain);
  EXPECT_FALSE(
    control->write_confirmed(
      *control->snapshot()->review, std::string("retry"), std::string(32, 'b')));
  EXPECT_EQ(writes, 1);
}
TEST_F(Parameters, BackendRejectionIsNotSuccessEvenIfCurrentValueMatches)
{
  const auto review = read(); reject = true;
  ASSERT_TRUE(control->write_confirmed(review, std::string("old"), std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_EQ(control->snapshot()->phase, p::ParameterPhase::Rejected); EXPECT_EQ(writes, 1);
  EXPECT_FALSE(control->snapshot()->fault); EXPECT_GE(reads, 3);
}
TEST_F(Parameters, InhibitDuringPrecheckAndInvalidTypesNeverDispatch)
{
  const auto review = read(); const int before = reads; hold_reads = true;
  EXPECT_FALSE(control->write_confirmed(review, true, std::string(32, 'a')));
  EXPECT_FALSE(control->write_confirmed(review, std::string("x"), ""));
  EXPECT_FALSE(control->read(static_cast<p::LocalParameter>(99)));
  ASSERT_TRUE(control->write_confirmed(review, std::string("proposal"), std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return reads > before;})); control->inhibit(true); release();
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_EQ(writes, 0); EXPECT_FALSE(control->snapshot()->fault);
  control->inhibit(false); invalid_read = true; hold_reads = false;
  ASSERT_TRUE(control->read(p::LocalParameter::OperatorNote));
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;})); EXPECT_FALSE(
    control->snapshot()->review);
}
TEST_F(Parameters, StopWhileWriteUnconfirmedDoesNotCompensate)
{
  const auto review = read(); hold_write = true;
  ASSERT_TRUE(control->write_confirmed(review, std::string("proposal"), std::string(32, 'a')));
  ASSERT_TRUE(until([&] {return writes == 1;})); control->stop();
  ASSERT_TRUE(until([&] {return control->snapshot()->stopped;}));
  EXPECT_TRUE(control->snapshot()->fault); EXPECT_EQ(writes, 1);
  EXPECT_FALSE(control->read(p::LocalParameter::OperatorNote));
}
