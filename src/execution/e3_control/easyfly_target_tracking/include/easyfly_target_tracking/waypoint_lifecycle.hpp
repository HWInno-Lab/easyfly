// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_target_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#pragma once
#include <future>
#include <utility>
namespace easyfly_target_tracking
{
enum class WaypointLifecycleState
{
  kIdle,
  kSending,
  kActive,
  kCanceling,
  kRecovering,
  kFault,
};
// Pure child-goal state machine.  A timed-out normal cancellation transitions
// to kRecovering; the owner must retain the handle and futures until a later
// confirmed terminal result.  Only kIdle permits a new child goal.
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
  bool enterRecovery()
  {
    if (state_ == WaypointLifecycleState::kIdle) {return false;}
    state_ = WaypointLifecycleState::kRecovering;
    cancel_requested_ = true;
    return true;
  }
  WaypointLifecycleState state() const {return state_;}
  bool mayIssue() const {return state_ == WaypointLifecycleState::kIdle;}
  // A ready result future may still throw.  The owner must clear it and retry
  // result/cancel requests while retaining the child ownership.
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
}  // namespace easyfly_target_tracking
