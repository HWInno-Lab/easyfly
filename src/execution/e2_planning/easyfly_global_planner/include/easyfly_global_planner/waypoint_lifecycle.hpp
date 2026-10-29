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
#pragma once
#include <future>
#include <utility>
namespace easyfly_global_planner
{
// Pure child-goal state machine used by GlobalPlanner.  In particular, a
// pending send is not considered idle: a late accepted goal must still pass
// through cancellation before the parent can finish or another child starts.
enum class WaypointLifecycleState
{
  kIdle,
  kSending,
  kActive,
  kCanceling,
  // The normal cancellation window expired.  The owner must keep the child
  // handle/futures and continue recovery; this state is never issuable.
  kRecovering,
  kFault,
};
class WaypointLifecycle
{
public:
  bool beginSend()
  {
    if (state_ != WaypointLifecycleState::kIdle) {return false;}
    state_ = WaypointLifecycleState::kSending;
    cancel_requested_ = false;
    return true;
  }
  bool resolveSend(const bool accepted)
  {
    if (state_ != WaypointLifecycleState::kSending &&
      state_ != WaypointLifecycleState::kRecovering)
    {
      return false;
    }
    if (!accepted) {
      state_ = WaypointLifecycleState::kIdle;
      cancel_requested_ = false;
    } else {
      state_ = cancel_requested_ ?
        (state_ == WaypointLifecycleState::kRecovering ?
        WaypointLifecycleState::kRecovering : WaypointLifecycleState::kCanceling) :
        WaypointLifecycleState::kActive;
    }
    return true;
  }
  bool requestCancel()
  {
    if (state_ == WaypointLifecycleState::kIdle) {return true;}
    cancel_requested_ = true;
    if (state_ == WaypointLifecycleState::kActive) {
      state_ = WaypointLifecycleState::kCanceling;
    }
    return true;
  }
  bool confirmTerminal()
  {
    if (state_ != WaypointLifecycleState::kActive &&
      state_ != WaypointLifecycleState::kCanceling &&
      state_ != WaypointLifecycleState::kRecovering &&
      state_ != WaypointLifecycleState::kFault)
    {
      return false;
    }
    state_ = WaypointLifecycleState::kIdle;
    cancel_requested_ = false;
    return true;
  }
  void markFault()
  {
    state_ = WaypointLifecycleState::kFault;
    cancel_requested_ = true;
  }
  // Leave the bounded normal-cancel phase and enter a persistent recovery
  // phase.  The caller must retain all child context until confirmTerminal().
  bool enterRecovery()
  {
    if (state_ == WaypointLifecycleState::kIdle) {return false;}
    state_ = WaypointLifecycleState::kRecovering;
    cancel_requested_ = true;
    return true;
  }
  WaypointLifecycleState state() const {return state_;}
  bool cancelRequested() const {return cancel_requested_;}
  bool mayIssue() const {return state_ == WaypointLifecycleState::kIdle;}
  // A result future may throw after becoming ready (for example when the
  // action handle was invalidated).  The owner must clear that future and
  // retry async_get_result/cancel while retaining the child ownership.
  bool canRetryResult() const
  {
    return state_ == WaypointLifecycleState::kActive ||
           state_ == WaypointLifecycleState::kCanceling ||
           state_ == WaypointLifecycleState::kRecovering ||
           state_ == WaypointLifecycleState::kFault;
  }
  // Consume a ready result future when the caller only needs terminal
  // confirmation.  An exceptional shared_future is one-shot: retaining it
  // would make every recovery pass throw before a fresh async_get_result can
  // be requested.  Clear it and the sent flag while retaining child
  // ownership in the non-idle lifecycle.
  template<typename Future>
  bool consumeReadyResult(Future & result_fut, bool & cancel_sent)
  {
    if (!result_fut.valid()) {return false;}
    try {
      result_fut.get();
      return true;
    } catch (...) {
      result_fut = Future{};
      cancel_sent = false;
      markFault();
      return false;
    }
  }
  // Ensure that cancellation has a result future to consume.  The action
  // transport is ordered deliberately: when result_fut is invalid, request
  // the result first and only then send cancel.  A cancel exception must not
  // skip an already-available result; only the combination of both failures
  // enters parent-held recovery without inferring a terminal state.
  template<typename Future, typename GetResult, typename SendCancel>
  bool prepareCancel(
    Future & result_fut, bool & cancel_sent,
    GetResult && get_result, SendCancel && send_cancel)
  {
    bool get_failed = false;
    bool cancel_failed = false;
    if (!result_fut.valid()) {
      try {
        result_fut = std::forward<GetResult>(get_result)();
      } catch (...) {
        result_fut = Future{};
        get_failed = true;
        markFault();
      }
      if (!result_fut.valid()) {
        get_failed = true;
        markFault();
      }
    }
    if (!cancel_sent) {
      try {
        std::forward<SendCancel>(send_cancel)();
        cancel_sent = true;
      } catch (...) {
        cancel_failed = true;
        markFault();
      }
    }
    if (!result_fut.valid()) {
      if (get_failed && cancel_failed) {
        enterRecovery();
      }
      return false;
    }
    return true;
  }

private:
  WaypointLifecycleState state_{WaypointLifecycleState::kIdle};
  bool cancel_requested_{false};
};
}  // namespace easyfly_global_planner
