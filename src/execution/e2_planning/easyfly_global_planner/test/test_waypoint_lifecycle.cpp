// Copyright 2026 EasyFly Project
//
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
#include <future>
#include <stdexcept>
#include <vector>
#include "easyfly_global_planner/waypoint_lifecycle.hpp"
namespace easyfly_global_planner
{
namespace
{
TEST(WaypointLifecycleTest, PendingSendCannotBeReplaced)
{
  WaypointLifecycle lifecycle;
  EXPECT_TRUE(lifecycle.beginSend());
  EXPECT_FALSE(lifecycle.mayIssue());
  EXPECT_TRUE(lifecycle.requestCancel());
  EXPECT_EQ(lifecycle.state(), WaypointLifecycleState::kSending);
  EXPECT_TRUE(lifecycle.resolveSend(true));
  EXPECT_EQ(lifecycle.state(), WaypointLifecycleState::kCanceling);
  EXPECT_FALSE(lifecycle.mayIssue());
  EXPECT_TRUE(lifecycle.confirmTerminal());
  EXPECT_TRUE(lifecycle.mayIssue());
}
TEST(WaypointLifecycleTest, RejectedSendReturnsIdle)
{
  WaypointLifecycle lifecycle;
  EXPECT_TRUE(lifecycle.beginSend());
  EXPECT_TRUE(lifecycle.resolveSend(false));
  EXPECT_EQ(lifecycle.state(), WaypointLifecycleState::kIdle);
  EXPECT_TRUE(lifecycle.beginSend());
}
TEST(WaypointLifecycleTest, FaultNeverLooksIdle)
{
  WaypointLifecycle lifecycle;
  EXPECT_TRUE(lifecycle.beginSend());
  lifecycle.markFault();
  EXPECT_EQ(lifecycle.state(), WaypointLifecycleState::kFault);
  EXPECT_FALSE(lifecycle.mayIssue());
  EXPECT_TRUE(lifecycle.enterRecovery());
  EXPECT_EQ(lifecycle.state(), WaypointLifecycleState::kRecovering);
  EXPECT_TRUE(lifecycle.confirmTerminal());
}
TEST(WaypointLifecycleTest, RecoveryRetainsOwnershipAfterNormalTimeout)
{
  WaypointLifecycle lifecycle;
  EXPECT_TRUE(lifecycle.beginSend());
  EXPECT_TRUE(lifecycle.resolveSend(true));
  EXPECT_TRUE(lifecycle.requestCancel());
  // The bounded normal cancellation phase has expired.  Recovery is still
  // non-issuable and only a confirmed child terminal state can release it.
  EXPECT_TRUE(lifecycle.enterRecovery());
  EXPECT_EQ(lifecycle.state(), WaypointLifecycleState::kRecovering);
  EXPECT_FALSE(lifecycle.mayIssue());
  EXPECT_TRUE(lifecycle.confirmTerminal());
  EXPECT_EQ(lifecycle.state(), WaypointLifecycleState::kIdle);
  EXPECT_TRUE(lifecycle.mayIssue());
}
TEST(WaypointLifecycleTest, LateSendResolutionIsOwnedByRecovery)
{
  WaypointLifecycle lifecycle;
  EXPECT_TRUE(lifecycle.beginSend());
  EXPECT_TRUE(lifecycle.enterRecovery());
  EXPECT_TRUE(lifecycle.resolveSend(true));
  EXPECT_EQ(lifecycle.state(), WaypointLifecycleState::kRecovering);
  EXPECT_FALSE(lifecycle.mayIssue());
  EXPECT_TRUE(lifecycle.confirmTerminal());
}
TEST(WaypointLifecycleTest, ExceptionalResultCanRetryWithoutReleasingChild)
{
  WaypointLifecycle lifecycle;
  EXPECT_TRUE(lifecycle.beginSend());
  EXPECT_TRUE(lifecycle.resolveSend(true));
  lifecycle.markFault();
  EXPECT_TRUE(lifecycle.canRetryResult());
  EXPECT_FALSE(lifecycle.mayIssue());
  EXPECT_TRUE(lifecycle.enterRecovery());
  EXPECT_TRUE(lifecycle.canRetryResult());
  EXPECT_TRUE(lifecycle.confirmTerminal());
}
TEST(WaypointLifecycleTest, ExceptionalReadyFutureIsClearedForRetry)
{
  WaypointLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.beginSend());
  ASSERT_TRUE(lifecycle.resolveSend(true));
  ASSERT_TRUE(lifecycle.requestCancel());
  std::promise<int> failed_promise;
  auto result_future = failed_promise.get_future().share();
  bool cancel_sent = true;
  failed_promise.set_exception(std::make_exception_ptr(std::runtime_error("transport")));
  EXPECT_FALSE(lifecycle.consumeReadyResult(result_future, cancel_sent));
  EXPECT_FALSE(result_future.valid());
  EXPECT_FALSE(cancel_sent);
  EXPECT_FALSE(lifecycle.mayIssue());
  // A new async_get_result equivalent is allowed; the old exceptional future
  // is not read again, and only terminal confirmation releases ownership.
  std::promise<int> terminal_promise;
  result_future = terminal_promise.get_future().share();
  terminal_promise.set_value(0);
  EXPECT_TRUE(lifecycle.consumeReadyResult(result_future, cancel_sent));
  EXPECT_TRUE(lifecycle.confirmTerminal());
  EXPECT_TRUE(lifecycle.mayIssue());
}
TEST(WaypointLifecycleTest, PrepareCancelGetsResultBeforeCancel)
{
  WaypointLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.beginSend());
  ASSERT_TRUE(lifecycle.resolveSend(true));
  ASSERT_TRUE(lifecycle.requestCancel());
  std::promise<int> terminal_promise;
  std::shared_future<int> result_future;
  bool cancel_sent = false;
  std::vector<int> order;
  EXPECT_TRUE(
    lifecycle.prepareCancel(
      result_future, cancel_sent,
      [&]() {
        order.push_back(1);
        return terminal_promise.get_future().share();
      },
      [&]() {order.push_back(2);}));
  EXPECT_EQ(order, (std::vector<int>{1, 2}));
  EXPECT_TRUE(cancel_sent);
  terminal_promise.set_value(0);
  ASSERT_TRUE(lifecycle.consumeReadyResult(result_future, cancel_sent));
  EXPECT_TRUE(lifecycle.confirmTerminal());
}
TEST(WaypointLifecycleTest, CancelFailureStillConsumesResult)
{
  WaypointLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.beginSend());
  ASSERT_TRUE(lifecycle.resolveSend(true));
  ASSERT_TRUE(lifecycle.requestCancel());
  std::promise<int> terminal_promise;
  auto result_future = terminal_promise.get_future().share();
  bool cancel_sent = false;
  EXPECT_TRUE(
    lifecycle.prepareCancel(
      result_future, cancel_sent,
      [&]() {return result_future;},
      [&]() {throw std::runtime_error("cancel transport");}));
  EXPECT_FALSE(cancel_sent);
  terminal_promise.set_value(0);
  ASSERT_TRUE(lifecycle.consumeReadyResult(result_future, cancel_sent));
  EXPECT_TRUE(lifecycle.confirmTerminal());
}
TEST(WaypointLifecycleTest, BothCancelTransportFailuresEnterRecovery)
{
  WaypointLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.beginSend());
  ASSERT_TRUE(lifecycle.resolveSend(true));
  ASSERT_TRUE(lifecycle.requestCancel());
  std::shared_future<int> result_future;
  bool cancel_sent = false;
  EXPECT_FALSE(
    lifecycle.prepareCancel(
      result_future, cancel_sent,
      [&]() -> std::shared_future<int> {
        throw std::runtime_error("result transport");
      },
      [&]() {throw std::runtime_error("cancel transport");}));
  EXPECT_EQ(lifecycle.state(), WaypointLifecycleState::kRecovering);
  EXPECT_FALSE(lifecycle.mayIssue());
}
}  // namespace
}  // namespace easyfly_global_planner
