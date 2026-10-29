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
//
// EasyFly E2: 全局规划节点 (M4.2)
//
// 数据流:
//   /occupancy_grid (OccupancyGrid, semantic_map 产出, 已含膨胀)
//   /uav_state      (起点)
//     → goal (/navigate: NavigateTo)
//     → 2D A* + 视线剪枝 (astar.hpp)
//     → 逐段 GoToWaypoint 执行 (mission_fsm 同款轮询模式)
//
// 失败语义:
//   - 目标在膨胀障碍内 / 越界 / 无可行动作: goal REJECT 或 abort (message 说明)
//   - 段执行 ABORTED 带 "takeover" (低电量 RTL 等) → 传播 abort 让位
//   - 用户 cancel → 取消当前段 + 悬停 + CANCELED
//   - V1 不做动态重规划 (静态世界假设, 动态避障归 M4.4 obstacle_avoidance)
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <easyfly_msgs/action/navigate_to.hpp>
#include <easyfly_msgs/action/go_to_waypoint.hpp>
#include <easyfly_msgs/msg/uav_state.hpp>
#include "easyfly_global_planner/adapters/planner_factory.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "easyfly_global_planner/goal_validation.hpp"
#include "easyfly_global_planner/map_validity.hpp"
#include "easyfly_global_planner/segment_acceptance.hpp"
#include "easyfly_global_planner/waypoint_lifecycle.hpp"
namespace easyfly_global_planner
{
namespace gp = easyfly::grid_planning;
using NavigateTo = easyfly_msgs::action::NavigateTo;
using GoToWaypoint = easyfly_msgs::action::GoToWaypoint;
using GoalHandleWp = rclcpp_action::ClientGoalHandle<GoToWaypoint>;
using namespace std::chrono_literals;
using namespace std::placeholders;
class GlobalPlannerNode : public rclcpp::Node
{
public:
  explicit GlobalPlannerNode(const rclcpp::NodeOptions & options)
  : Node("easyfly_global_planner", options)
  {
    loadParams();
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    AStarParams algorithm_params;
    const auto max_cells = declare_parameter<std::int64_t>("max_grid_cells", 4000000, descriptor);
    const auto expansions = declare_parameter<std::int64_t>(
      "planning_max_expansions", 4000000,
      descriptor);
    planning_timeout_ = declare_parameter<double>("planning_timeout", 5.0, descriptor);
    if (max_cells < 1 || expansions < 0 || !std::isfinite(planning_timeout_) ||
      planning_timeout_ <= 0 || planning_timeout_ > 300)
    {
      throw std::invalid_argument("invalid planning budget");
    }
    algorithm_params.max_cells = static_cast<std::size_t>(max_cells);
    planning_max_expansions_ = static_cast<std::size_t>(expansions);
    planner_ = make_planner(
      declare_parameter<std::string>(
        "planner_backend", "astar", descriptor), algorithm_params);
    auto qos = rclcpp::QoS(10);
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      params_.grid_topic, qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> send_lock(send_mtx_);
        std::lock_guard<std::mutex> lk(snap_mtx_);
        if (!occupancyGridReady(*msg)) {
          ++map_invalidations_;
          search_stop_ = true;
        }
        ++grid_revision_;
        latest_grid_ = std::move(msg);
        grid_arrival_ = std::chrono::steady_clock::now();
        grid_received_ = true;
      });
    state_sub_ = create_subscription<easyfly_msgs::msg::UAVState>(
      "/uav_state", qos,
      [this](easyfly_msgs::msg::UAVState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(snap_mtx_);
        latest_state_ = *msg;
        state_received_ = true;
        state_arrival_ = std::chrono::steady_clock::now();
      });
    cli_wp_ = rclcpp_action::create_client<GoToWaypoint>(this, "/goto_waypoint");
    as_nav_ = rclcpp_action::create_server<NavigateTo>(
      this, declare_parameter<std::string>("action_name", "/navigate_astar"),
      std::bind(&GlobalPlannerNode::handle_goal, this, _1, _2),
      std::bind(&GlobalPlannerNode::handle_cancel, this, _1),
      std::bind(&GlobalPlannerNode::handle_accepted, this, _1));
    RCLCPP_INFO(
      get_logger(),
      "GlobalPlanner 启动: %s → %s (栅格超时 %.0fs, 段到位半径 %.2fm)",
      params_.grid_topic.c_str(), get_parameter(
        "action_name").as_string().c_str(), params_.grid_stale_timeout,
      params_.segment_acceptance);
  }
  ~GlobalPlannerNode() override
  {
    stopping_ = true;
    cancel_req_ = true;
    search_stop_ = true;
    if (worker_.joinable()) {worker_.join();}
  }

private:
  struct NodeParams
  {
    std::string grid_topic{"/occupancy_grid"};
    double grid_stale_timeout{5.0};       // 栅格数据过期判定 [s]
    double segment_acceptance{kDefaultSegmentAcceptance};      // 段到位半径 [m]
    double max_speed_default{2.0};
    double z_min{0.5};
    double z_max{2.5};                    // SLAM 盲区约束 (Q22)
    double state_timeout{1.0};            // /uav_state 新鲜度 [s]
    double child_cancel_timeout{5.0};
    double child_cancel_max_wait{30.0};
  };
  void loadParams()
  {
    declare_parameter<std::string>("config_file", "");
    std::string cfg = get_parameter("config_file").as_string();
    if (cfg.empty()) {
      cfg = ament_index_cpp::get_package_share_directory("easyfly_global_planner") +
        "/config/navigate.yaml";
    }
    YAML::Node root = YAML::LoadFile(cfg);
    const auto & p = root["global_planner"];
    params_.grid_topic = p["grid_topic"].as<std::string>(params_.grid_topic);
    params_.grid_stale_timeout =
      p["grid_stale_timeout"].as<double>(params_.grid_stale_timeout);
    params_.segment_acceptance =
      p["segment_acceptance"].as<double>(params_.segment_acceptance);
    const double configured_segment_acceptance = params_.segment_acceptance;
    params_.segment_acceptance =
      normalizeSegmentAcceptance(configured_segment_acceptance);
    if (!std::isfinite(configured_segment_acceptance) ||
      configured_segment_acceptance <= 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "segment_acceptance=%.3f 无效，使用安全默认值 %.2fm",
        configured_segment_acceptance, params_.segment_acceptance);
    } else if (configured_segment_acceptance > kMaxSegmentAcceptance) {
      RCLCPP_WARN(
        get_logger(),
        "segment_acceptance=%.3f 超过安全上限 %.2fm，截断为上限",
        configured_segment_acceptance, kMaxSegmentAcceptance);
    }
    params_.max_speed_default =
      p["max_speed_default"].as<double>(params_.max_speed_default);
    params_.z_min = p["z_min"].as<double>(params_.z_min);
    params_.z_max = p["z_max"].as<double>(params_.z_max);
    params_.state_timeout = p["state_timeout"].as<double>(params_.state_timeout);
    params_.child_cancel_timeout =
      p["child_cancel_timeout"].as<double>(params_.child_cancel_timeout);
    params_.child_cancel_max_wait =
      p["child_cancel_max_wait"].as<double>(params_.child_cancel_max_wait);
  }
  // ------------------------------------------------ 快照
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr snapGrid(
    double & age_s, std::uint64_t * revision = nullptr,
    std::uint64_t * invalidations = nullptr)
  {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - grid_arrival_).count();
    if (revision) {*revision = grid_revision_;}
    if (invalidations) {*invalidations = map_invalidations_;}
    return latest_grid_;
  }
  bool waitForFirstGrid()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(std::max(0.1, params_.grid_stale_timeout));
    while ((!stopping_ && rclcpp::ok()) && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lk(snap_mtx_);
        if (grid_received_) {return true;}
      }
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }
  struct StateSnapshot
  {
    easyfly_msgs::msg::UAVState state;
    bool received{false};
    double age_s{std::numeric_limits<double>::infinity()};
  };
  StateSnapshot snapState()
  {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    StateSnapshot snapshot;
    snapshot.state = latest_state_;
    snapshot.received = state_received_;
    if (state_received_) {
      snapshot.age_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - state_arrival_).count();
    }
    return snapshot;
  }
  bool stateValid(const StateSnapshot & snapshot) const
  {
    const auto & state = snapshot.state;
    return snapshot.received && std::isfinite(snapshot.age_s) &&
           snapshot.age_s <= params_.state_timeout && state.connected &&
           state.pose_valid && std::isfinite(state.position_x) &&
           std::isfinite(state.position_y) && std::isfinite(state.position_z) &&
           std::isfinite(state.yaw);
  }
  // ------------------------------------------------ Action 服务端
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateTo::Goal> goal)
  {
    if (!goal || !validNavigateGoal(*goal)) {
      RCLCPP_WARN(
        get_logger(),
        "拒绝非有限导航 goal (x/y 必须 finite, z 仅允许 NaN 哨兵, "
        "max_speed 必须 finite)");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lk(goal_mtx_);
    // Reserve admission in handle_goal, before handle_accepted runs on a
    // different executor path.  Otherwise two simultaneous requests can
    // both observe active=false and create competing child waypoints.
    if (active_ || goal_pending_) {
      RCLCPP_WARN(get_logger(), "已有导航在执行, 拒绝新 goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_pending_ = true;
    pending_goal_uuid_ = uuid;
    pending_cancel_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>> gh)
  {
    std::lock_guard<std::mutex> lk(goal_mtx_);
    if (auto current = active_goal_.lock(); current && current == gh) {
      std::lock_guard<std::mutex> send_lock(send_mtx_);
      search_stop_ = true;
      cancel_req_ = true;        // 仅取消当前 parent goal 的控制线程
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    if (goal_pending_ && gh && gh->get_goal_id() == pending_goal_uuid_) {
      pending_cancel_ = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    return rclcpp_action::CancelResponse::REJECT;
  }
  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>> gh)
  {
    {
      std::lock_guard<std::mutex> lk(goal_mtx_);
      if (!goal_pending_ || gh->get_goal_id() != pending_goal_uuid_) {
        if (goal_pending_ && gh->get_goal_id() == pending_goal_uuid_) {
          goal_pending_ = false;
          pending_cancel_ = false;
        }
        auto res = std::make_shared<NavigateTo::Result>();
        res->success = false;
        res->message = "goal admission expired";
        try {gh->abort(res);} catch (...) {}
        return;
      }
      goal_pending_ = false;
      if (pending_cancel_) {
        pending_cancel_ = false;
        auto res = std::make_shared<NavigateTo::Result>();
        res->success = false;
        res->message = "cancelled before start";
        try {gh->canceled(res);} catch (...) {}
        return;
      }
      active_ = true;
      active_goal_ = gh;
      cancel_req_ = false;
      search_stop_ = false;
      child_fault_ = false;
    }
    if (worker_.joinable()) {worker_.join();}
    worker_ = std::thread(
      [this, gh]() {
        try {
          navigateLoop(gh);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "导航线程异常: %s", e.what());
          if (!child_fault_) {
            try {
              auto res = std::make_shared<NavigateTo::Result>();
              res->success = false;
              res->message = std::string("internal error: ") + e.what();
              gh->abort(res);
            } catch (...) {}
          } else {
            RCLCPP_ERROR(
              get_logger(),
              "子航点故障未确认终态，保持 NavigateTo parent active");
          }
          // navigateLoop's RAII cleanup owns admission release; if a
          // child is unresolved it deliberately keeps the parent slot
          // occupied rather than creating an orphan race.
        }
      });
  }
  // ------------------------------------------------ 执行线程
  /// 轮询等待 future, 期间响应取消. 返回 0=ready, 1=timeout, 2=中断
  template<typename F>
  int pollFuture(
    F & fut, double timeout_s,
    const std::function<bool()> & safety_check = {})
  {
    if (!fut.valid()) {return 1;}
    const auto t0 = std::chrono::steady_clock::now();
    while (true) {
      if (stopping_ || cancel_req_) {return 2;}
      if (safety_check && !safety_check()) {return 3;}
      if (fut.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        return 0;
      }
      if (std::chrono::steady_clock::now() - t0 >
        std::chrono::duration<double>(timeout_s))
      {
        return 1;
      }
    }
  }
  void clearActiveGoal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>> & gh)
  {
    std::lock_guard<std::mutex> lk(goal_mtx_);
    if (auto current = active_goal_.lock(); current && current == gh) {
      active_goal_.reset();
      active_ = false;
      cancel_req_ = false;
      search_stop_ = false;
    }
  }
  bool issueWaypoint(
    const GoToWaypoint::Goal & goal,
    std::shared_ptr<GoalHandleWp> & gh,
    std::shared_future<GoalHandleWp::SharedPtr> & goal_fut,
    WaypointLifecycle & lifecycle,
    const std::function<bool()> & safety_check = {})
  {
    if (!lifecycle.beginSend()) {return false;}
    const auto server_deadline = std::chrono::steady_clock::now() + 3s;
    while (!cli_wp_->wait_for_action_server(100ms)) {
      if (stopping_ || cancel_req_ || (safety_check && !safety_check())) {
        lifecycle.resolveSend(false);
        return false;
      }
      if (std::chrono::steady_clock::now() >= server_deadline) {
        lifecycle.resolveSend(false);
        RCLCPP_ERROR(get_logger(), "GoToWaypoint action server 不可用");
        return false;
      }
    }
    if (safety_check && !safety_check()) {
      lifecycle.resolveSend(false);
      return false;
    }
    try {
      // Serialize the final admission check and send with map invalidation
      // and parent cancellation. Release this lock before polling futures.
      std::lock_guard<std::mutex> send_lock(send_mtx_);
      if (stopping_ || cancel_req_ || (safety_check && !safety_check())) {
        lifecycle.resolveSend(false);
        return false;
      }
      goal_fut = cli_wp_->async_send_goal(goal);
    } catch (const std::exception & e) {
      lifecycle.resolveSend(false);
      RCLCPP_ERROR(get_logger(), "发送子航点异常: %s", e.what());
      return false;
    }
    if (!goal_fut.valid()) {
      lifecycle.resolveSend(false);
      RCLCPP_ERROR(get_logger(), "发送子航点返回无效 future");
      return false;
    }
    const int status = pollFuture(goal_fut, 10.0, safety_check);
    if (status != 0) {
      // Keep the sending state and future.  A late accepted goal must
      // be resolved and canceled instead of becoming an orphan.
      RCLCPP_WARN(get_logger(), "子航点发送 future 未在期限内完成 (%d)", status);
      return false;
    }
    try {
      gh = goal_fut.get();
    } catch (const std::exception & e) {
      // An exceptional send future cannot produce a handle on a later
      // read. Clear it and keep the parent in unknown-handle recovery
      // without dropping ownership or cancelling other owners' goals.
      goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
      lifecycle.markFault();
      RCLCPP_ERROR(get_logger(), "解析子航点 future 异常: %s", e.what());
      return false;
    } catch (...) {
      goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
      lifecycle.markFault();
      RCLCPP_ERROR(get_logger(), "解析子航点 future 未知异常");
      return false;
    }
    if (!gh) {
      lifecycle.resolveSend(false);
      goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
      return false;
    }
    lifecycle.resolveSend(true);
    return true;
  }
  bool cancelWaypoint(
    std::shared_ptr<GoalHandleWp> & gh,
    std::shared_future<GoalHandleWp::SharedPtr> & goal_fut,
    std::shared_future<rclcpp_action::Client<GoToWaypoint>::WrappedResult> & res_fut,
    WaypointLifecycle & lifecycle, bool & cancel_sent, double timeout_s)
  {
    try {
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(std::max(0.1, timeout_s));
      if (lifecycle.state() == WaypointLifecycleState::kIdle) {return true;}
      lifecycle.requestCancel();
      if (lifecycle.state() == WaypointLifecycleState::kSending ||
        (lifecycle.state() == WaypointLifecycleState::kRecovering && !gh))
      {
        if (!goal_fut.valid()) {return false;}
        while (goal_fut.wait_for(100ms) != std::future_status::ready) {
          if ((stopping_ || !rclcpp::ok()) || std::chrono::steady_clock::now() >= deadline) {
            return false;
          }
        }
        try {
          gh = goal_fut.get();
        } catch (const std::exception & e) {
          // This pending future is permanently exceptional.  Clear
          // it and keep the parent in unknown-handle recovery so a
          // later pass never reads the same future again.
          goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
          lifecycle.markFault();
          RCLCPP_WARN(get_logger(), "解析 pending 子航点异常: %s", e.what());
          return false;
        } catch (...) {
          goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
          lifecycle.markFault();
          RCLCPP_WARN(get_logger(), "解析 pending 子航点未知异常");
          return false;
        }
        if (!gh) {
          lifecycle.resolveSend(false);
          goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
          return true;
        }
        lifecycle.resolveSend(true);
      }
      if (!gh) {return false;}
      // A child may have reached a terminal state just before the
      // parent cancellation request.  Consume that result first;
      // canceling an already-terminal handle can throw and strand
      // recovery forever.
      if (res_fut.valid() &&
        res_fut.wait_for(0s) == std::future_status::ready)
      {
        if (!lifecycle.consumeReadyResult(res_fut, cancel_sent)) {
          lifecycle.markFault();
          RCLCPP_WARN(
            get_logger(),
            "读取已完成子航点终态异常，清空 future 后重试 cancel/result");
          return false;
        }
        if (!lifecycle.confirmTerminal()) {return false;}
        gh.reset();
        goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
        res_fut = std::shared_future<
          rclcpp_action::Client<GoToWaypoint>::WrappedResult>();
        cancel_sent = false;
        return true;
      }
      // With no result future, ask for the result before cancellation.
      // A cancel transport exception must not skip a result future that
      // was already obtained (or becomes available while cancel fails).
      if (!lifecycle.prepareCancel(
          res_fut, cancel_sent,
          [this, &gh]() {return cli_wp_->async_get_result(gh);},
          [this, &gh]() {cli_wp_->async_cancel_goal(gh);}))
      {
        return false;
      }
      while (res_fut.wait_for(100ms) != std::future_status::ready) {
        if ((stopping_ || !rclcpp::ok()) || std::chrono::steady_clock::now() >= deadline) {
          return false;
        }
      }
      if (!lifecycle.consumeReadyResult(res_fut, cancel_sent)) {
        lifecycle.markFault();
        RCLCPP_WARN(
          get_logger(),
          "读取子航点终态异常，清空 future 后重试 cancel/result");
        return false;
      }
      if (!lifecycle.confirmTerminal()) {return false;}
      gh.reset();
      goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
      res_fut = std::shared_future<
        rclcpp_action::Client<GoToWaypoint>::WrappedResult>();
      cancel_sent = false;
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "子航点取消状态机异常: %s", e.what());
      return false;
    } catch (...) {
      RCLCPP_WARN(get_logger(), "子航点取消状态机未知异常");
      return false;
    }
  }
  bool cancelWaypointUntilTerminal(
    std::shared_ptr<GoalHandleWp> & gh,
    std::shared_future<GoalHandleWp::SharedPtr> & goal_fut,
    std::shared_future<rclcpp_action::Client<GoToWaypoint>::WrappedResult> & res_fut,
    WaypointLifecycle & lifecycle, bool & cancel_sent)
  {
    if (lifecycle.state() == WaypointLifecycleState::kIdle) {return true;}
    bool unknown_handle = !gh && !goal_fut.valid() &&
      lifecycle.state() == WaypointLifecycleState::kFault;
    if (unknown_handle) {
      // A send future that threw cannot be resolved into a handle.  Do
      // not read it again; retain the parent and enter fault recovery.
      lifecycle.enterRecovery();
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(std::max(1.0, params_.child_cancel_max_wait));
    while (!unknown_handle && (!stopping_ && rclcpp::ok()) &&
      lifecycle.state() != WaypointLifecycleState::kRecovering &&
      lifecycle.state() != WaypointLifecycleState::kIdle)
    {
      const double remaining = std::chrono::duration<double>(
        deadline - std::chrono::steady_clock::now()).count();
      if (remaining <= 0.0) {break;}
      if (cancelWaypoint(
          gh, goal_fut, res_fut, lifecycle, cancel_sent,
          std::min(params_.child_cancel_timeout, remaining)) &&
        lifecycle.state() == WaypointLifecycleState::kIdle)
      {
        return true;
      }
      if (!unknown_handle && !gh && !goal_fut.valid() &&
        lifecycle.state() == WaypointLifecycleState::kFault)
      {
        unknown_handle = true;
        lifecycle.enterRecovery();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "等待子航点取消终态，父 goal 暂不结束");
      std::this_thread::sleep_for(100ms);
    }
    if ((stopping_ || !rclcpp::ok())) {return false;}
    if (lifecycle.state() != WaypointLifecycleState::kIdle) {
      lifecycle.enterRecovery();
      RCLCPP_ERROR(
        get_logger(),
        "子航点取消超过 %.1fs，进入持续 recovery；保持 child 所有权",
        params_.child_cancel_max_wait);
      int recovery_attempts = 0;
      while ((!stopping_ && rclcpp::ok()) && lifecycle.state() != WaypointLifecycleState::kIdle) {
        if (cancelWaypoint(
            gh, goal_fut, res_fut, lifecycle, cancel_sent, 1.0) &&
          lifecycle.state() == WaypointLifecycleState::kIdle)
        {
          return true;
        }
        // Retry the per-goal cancellation at a low rate.  A timeout
        // never means terminal; ownership remains in this thread.
        cancel_sent = false;
        ++recovery_attempts;
        if ((!gh && !goal_fut.valid() &&
          (recovery_attempts == 1 || recovery_attempts % 10 == 0)) ||
          recovery_attempts % 10 == 0)
        {
          // Unknown ownership must never cancel another caller's goal.
          RCLCPP_ERROR(get_logger(), "子航点所有权未确认，保持故障锁；不取消其他所有者");
        }
        std::this_thread::sleep_for(1s);
      }
    }
    return lifecycle.state() == WaypointLifecycleState::kIdle;
  }
  void navigateLoop(std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>> gh)
  {
    std::shared_ptr<GoalHandleWp> wp_gh;
    std::shared_future<GoalHandleWp::SharedPtr> wp_goal_fut;
    std::shared_future<rclcpp_action::Client<GoToWaypoint>::WrappedResult> wp_res_fut;
    WaypointLifecycle child_lifecycle;
    bool cancel_sent = false;
    struct Cleanup
    {
      GlobalPlannerNode * node;
      std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>> goal;
      std::shared_ptr<GoalHandleWp> * child;
      std::shared_future<GoalHandleWp::SharedPtr> * goal_fut;
      std::shared_future<rclcpp_action::Client<GoToWaypoint>::WrappedResult> * res_fut;
      WaypointLifecycle * lifecycle;
      bool * cancel_sent;
      ~Cleanup() noexcept
      {
        try {
          if (lifecycle->state() != WaypointLifecycleState::kIdle) {
            node->cancelWaypointUntilTerminal(
              *child, *goal_fut, *res_fut, *lifecycle, *cancel_sent);
          }
        } catch (...) {
          RCLCPP_ERROR(
            node->get_logger(), "导航 RAII 子航点清理异常，保持 parent active");
        }
        if (lifecycle->state() == WaypointLifecycleState::kIdle) {
          node->child_fault_ = false;
          try {node->clearActiveGoal(goal);} catch (...) {}
        } else {
          node->child_fault_ = true;
          RCLCPP_ERROR(
            node->get_logger(), "导航 parent 保持 active：子航点终态未确认");
        }
      }
    } cleanup{this, gh, &wp_gh, &wp_goal_fut, &wp_res_fut,
      &child_lifecycle, &cancel_sent};
    const auto goal = gh->get_goal();
    const double max_speed =
      goal->max_speed > 0 ? goal->max_speed : params_.max_speed_default;
    // ---- 栅格新鲜度 ----
    // Do not classify the default empty snapshot as a bad frame.  Once a
    // grid has actually arrived, occupancyGridReady enforces the odom
    // frame and finite geometry fail-closed.
    if (!waitForFirstGrid()) {
      auto res = std::make_shared<NavigateTo::Result>();
      res->success = false;
      res->message = "occupancy grid not received";
      gh->abort(res);
      return;
    }
    double grid_age = 0;
    std::uint64_t revision = 0, map_generation = 0;
    const auto grid_msg = snapGrid(grid_age, &revision, &map_generation);
    if (!grid_msg || grid_age > params_.grid_stale_timeout ||
      !occupancyGridReady(*grid_msg))
    {
      auto res = std::make_shared<NavigateTo::Result>();
      res->success = false;
      res->message = (!grid_msg || grid_age > params_.grid_stale_timeout) ?
        "occupancy grid stale (" + std::to_string(grid_age).substr(0, 4) +
        "s), semantic_map 未运行?" :
        "occupancy grid not ready (semantic_map rebuilding)";
      gh->abort(res);
      return;
    }
    gp::Request request;
    request.grid = {grid_msg->info.width, grid_msg->info.height,
      grid_msg->info.resolution, grid_msg->info.origin.position.x,
      grid_msg->info.origin.position.y, grid_msg->data.data(), grid_msg->data.size()};
    request.map.revision = revision;
    request.map.source_stamp_ns = static_cast<std::int64_t>(grid_msg->header.stamp.sec) *
      1000000000 + grid_msg->header.stamp.nanosec;
    const auto state_snapshot = snapState();
    if (!stateValid(state_snapshot)) {
      auto res = std::make_shared<NavigateTo::Result>();
      res->success = false;
      res->message = "uav state invalid or stale";
      gh->abort(res);
      return;
    }
    const auto & uav = state_snapshot.state;
    // ---- 目标高度: NaN = 当前高度, 双端钳位 ----
    const double goal_z = std::isfinite(goal->target_z) ?
      std::clamp(goal->target_z, params_.z_min, params_.z_max) :
      std::clamp(uav.position_z, params_.z_min, params_.z_max);
    // ---- 规划 ----
    request.start = {uav.position_x, uav.position_y};
    request.goal = {goal->target_x, goal->target_y};
    gp::SearchControl control;
    control.cancel_requested = &search_stop_;
    control.expansion_limit = planning_max_expansions_;
    control.deadline = gp::SearchControl::Clock::now() +
      std::chrono::duration_cast<gp::SearchControl::Clock::duration>(
      std::chrono::duration<double>(planning_timeout_));
    const auto plan = planner_->plan(request, control);
    auto mapStillValid = [&]() {
        double age = 0;
        std::uint64_t invalidations = 0;
        const auto current = snapGrid(age, nullptr, &invalidations);
        return current && invalidations == map_generation &&
               age <= params_.grid_stale_timeout && occupancyGridReady(*current);
      };
    if (cancel_req_ || stopping_ || !mapStillValid() || plan.status != gp::Status::ok) {
      const char * msg =
        cancel_req_ ? "cancelled" :
        stopping_ ? "planner stopping" :
        !mapStillValid() ? "occupancy grid invalidated during planning" :
        plan.status == gp::Status::timed_out ? "planning time budget exhausted" :
        plan.status == gp::Status::budget_exhausted ? "planning expansion budget exhausted" :
        plan.status == gp::Status::resource_limit ? "planning resource limit" :
        plan.status == gp::Status::canceled ? "planning interrupted" :
        plan.status == gp::Status::goal_in_obstacle ? "goal in inflated obstacle" :
        plan.status == gp::Status::out_of_bounds ? "goal out of grid bounds" :
        plan.status == gp::Status::start_in_obstacle ? "start deeply in obstacle" :
        "no path found";
      RCLCPP_WARN(get_logger(), "规划失败: %s", msg);
      auto res = std::make_shared<NavigateTo::Result>();
      res->success = false;
      res->message = msg;
      if (cancel_req_) {gh->canceled(res);} else {gh->abort(res);}
      return;
    }
    const size_t total_seg = plan.waypoints.size() - 1;
    RCLCPP_INFO(
      get_logger(), "导航规划: (%.1f,%.1f)→(%.1f,%.1f) %zu 段 (剪枝后)",
      uav.position_x, uav.position_y, goal->target_x, goal->target_y,
      total_seg);
    // ---- 逐段执行 (waypoints[0] ≈ 当前位置, 跳过) ----
    const auto t_total0 = std::chrono::steady_clock::now();
    auto requireState = [&](const char * phase, StateSnapshot & snapshot) {
        snapshot = snapState();
        if (stateValid(snapshot) && mapStillValid()) {return true;}
        RCLCPP_ERROR(get_logger(), "导航安全终止: UAV state invalid/stale (%s)", phase);
        if (!cancelWaypointUntilTerminal(
            wp_gh, wp_goal_fut, wp_res_fut, child_lifecycle, cancel_sent))
        {
          return false;
        }
        auto state_res = std::make_shared<NavigateTo::Result>();
        state_res->success = false;
        state_res->message = !mapStillValid() ?
          "occupancy grid invalidated or stale" : "uav state invalid or stale";
        gh->abort(state_res);
        return false;
      };
    for (size_t seg = 1; seg < plan.waypoints.size(); ++seg) {
      if (cancel_req_) {
        if (!cancelWaypointUntilTerminal(
            wp_gh, wp_goal_fut, wp_res_fut, child_lifecycle, cancel_sent))
        {
          return;
        }
        auto res = std::make_shared<NavigateTo::Result>();
        res->success = false;
        res->message = "cancelled";
        gh->canceled(res);
        return;
      }
      StateSnapshot segment_state;
      if (!requireState("before segment", segment_state)) {return;}
      const double wx = plan.waypoints[seg].x;
      const double wy = plan.waypoints[seg].y;
      // 末段用目标 z; 中间段维持巡航高度
      const double wz = (seg + 1 == plan.waypoints.size()) ? goal_z :
        std::clamp(segment_state.state.position_z, params_.z_min, params_.z_max);
      GoToWaypoint::Goal seg_goal;
      seg_goal.target_x = wx;
      seg_goal.target_y = wy;
      seg_goal.target_z = wz;
      seg_goal.max_speed = max_speed;
      seg_goal.acceptance_radius = params_.segment_acceptance;
      if (child_lifecycle.state() != WaypointLifecycleState::kIdle &&
        !cancelWaypointUntilTerminal(
          wp_gh, wp_goal_fut, wp_res_fut, child_lifecycle, cancel_sent))
      {
        return;
      }
      wp_gh.reset();
      wp_goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
      wp_res_fut = std::shared_future<
        rclcpp_action::Client<GoToWaypoint>::WrappedResult>();
      cancel_sent = false;
      child_lifecycle = WaypointLifecycle();
      if (!issueWaypoint(
          seg_goal, wp_gh, wp_goal_fut, child_lifecycle,
          [this, &mapStillValid]() {return stateValid(snapState()) && mapStillValid();}))
      {
        if (!cancelWaypointUntilTerminal(
            wp_gh, wp_goal_fut, wp_res_fut, child_lifecycle, cancel_sent))
        {
          return;
        }
        auto res = std::make_shared<NavigateTo::Result>();
        res->success = false;
        res->message = cancel_req_ ? "cancelled" : "GoToWaypoint send failed";
        if (cancel_req_) {
          gh->canceled(res);
        } else {
          gh->abort(res);
        }
        return;
      }
      try {
        wp_res_fut = cli_wp_->async_get_result(wp_gh);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "获取子航点 result future 异常: %s", e.what());
        if (!cancelWaypointUntilTerminal(
            wp_gh, wp_goal_fut, wp_res_fut, child_lifecycle, cancel_sent))
        {
          return;
        }
        auto res = std::make_shared<NavigateTo::Result>();
        res->success = false;
        res->message = "waypoint result unavailable";
        gh->abort(res);
        return;
      }
      if (!wp_res_fut.valid()) {
        RCLCPP_ERROR(get_logger(), "子航点 result future 无效");
        if (!cancelWaypointUntilTerminal(
            wp_gh, wp_goal_fut, wp_res_fut, child_lifecycle, cancel_sent))
        {
          return;
        }
        auto res = std::make_shared<NavigateTo::Result>();
        res->success = false;
        res->message = "waypoint result future invalid";
        gh->abort(res);
        return;
      }
      // 段结果轮询
      bool seg_done = false;
      int seg_fail_code = 0;        // 0=ok, 1=takeover, 2=timeout
      while (!stopping_ && !seg_done) {
        if (cancel_req_) {
          if (!cancelWaypointUntilTerminal(
              wp_gh, wp_goal_fut, wp_res_fut,
              child_lifecycle, cancel_sent))
          {
            return;
          }
          auto res = std::make_shared<NavigateTo::Result>();
          res->success = false;
          res->message = "cancelled";
          gh->canceled(res);
          return;
        }
        StateSnapshot wait_state;
        if (!requireState("while waiting for segment", wait_state)) {return;}
        if (wp_res_fut.wait_for(100ms) == std::future_status::ready) {
          rclcpp_action::Client<GoToWaypoint>::WrappedResult wrapped;
          try {
            wrapped = wp_res_fut.get();
          } catch (const std::exception & e) {
            // A ready exceptional shared_future is one-shot.
            // Clear it before recovery so the next round can
            // request a fresh result and resend cancellation,
            // while retaining wp_gh and the parent goal.
            wp_res_fut = std::shared_future<
              rclcpp_action::Client<GoToWaypoint>::WrappedResult>();
            cancel_sent = false;
            child_lifecycle.markFault();
            RCLCPP_ERROR(get_logger(), "读取子航点终态异常: %s", e.what());
            if (!cancelWaypointUntilTerminal(
                wp_gh, wp_goal_fut, wp_res_fut,
                child_lifecycle, cancel_sent))
            {
              return;
            }
            auto res = std::make_shared<NavigateTo::Result>();
            res->success = false;
            res->message = "waypoint result read failed";
            gh->abort(res);
            return;
          }
          if (!child_lifecycle.confirmTerminal()) {
            RCLCPP_ERROR(get_logger(), "子航点终态与生命周期状态不一致");
            return;
          }
          wp_gh.reset();
          wp_goal_fut = std::shared_future<GoalHandleWp::SharedPtr>();
          wp_res_fut = std::shared_future<
            rclcpp_action::Client<GoToWaypoint>::WrappedResult>();
          cancel_sent = false;
          const bool succeeded = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
            wrapped.result && wrapped.result->success;
          if (wrapped.code == rclcpp_action::ResultCode::ABORTED) {
            if (wrapped.result &&
              wrapped.result->message.find("takeover") != std::string::npos)
            {
              seg_fail_code = 1;
            } else {
              seg_fail_code = 2;                // 段超时就地悬停
            }
          } else if (!succeeded) {
            seg_fail_code = 2;
          }
          seg_done = true;
          break;
        }
        // 反馈
        try {
          auto fb = std::make_shared<NavigateTo::Feedback>();
          fb->current_segment = static_cast<int32_t>(seg);
          fb->total_segments = static_cast<int32_t>(total_seg);
          // 剩余 = 当前位置到本段终点 + 其后各段长度
          const auto & cur = wait_state.state;
          double remaining = std::hypot(
            plan.waypoints[seg].x - cur.position_x,
            plan.waypoints[seg].y - cur.position_y);
          for (size_t k = seg + 1; k < plan.waypoints.size(); ++k) {
            remaining += std::hypot(
              plan.waypoints[k].x - plan.waypoints[k - 1].x,
              plan.waypoints[k].y - plan.waypoints[k - 1].y);
          }
          fb->distance_remaining = remaining;
          gh->publish_feedback(fb);
        } catch (...) {}
      }
      if (seg_fail_code == 1) {
        RCLCPP_WARN(get_logger(), "段 %zu 被外部接管中止, 导航让位", seg);
        auto res = std::make_shared<NavigateTo::Result>();
        res->success = false;
        res->message = "external takeover (RTL/AUTO.LAND)";
        gh->abort(res);
        return;
      }
      if (seg_fail_code == 2) {
        RCLCPP_WARN(get_logger(), "段 %zu 超时就地悬停, 导航中止", seg);
        auto res = std::make_shared<NavigateTo::Result>();
        res->success = false;
        res->message = "segment timeout (hovering in place)";
        gh->abort(res);
        return;
      }
      RCLCPP_INFO(
        get_logger(), "段 %zu/%zu 到达 (%.1f, %.1f)",
        seg, total_seg, wx, wy);
    }
    const double total_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t_total0).count();
    RCLCPP_INFO(get_logger(), "导航完成, 用时 %.0fs", total_s);
    auto res = std::make_shared<NavigateTo::Result>();
    res->success = true;
    res->message = "navigated";
    gh->succeed(res);
  }
  // ------------------------------------------------ 成员
  NodeParams params_;
  std::mutex snap_mtx_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_grid_;
  std::chrono::steady_clock::time_point grid_arrival_{std::chrono::steady_clock::now()};
  bool grid_received_{false};
  std::mutex send_mtx_;
  std::unique_ptr<gp::GridPlanner> planner_;
  std::atomic_bool search_stop_{false};
  std::uint64_t grid_revision_{0}, map_invalidations_{0};    // Guarded by snap_mtx_.
  std::size_t planning_max_expansions_{4000000};
  double planning_timeout_{5.0};
  easyfly_msgs::msg::UAVState latest_state_;
  bool state_received_{false};
  std::chrono::steady_clock::time_point state_arrival_{};
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  std::atomic<bool> active_{false};
  std::atomic<bool> cancel_req_{false};
  std::atomic<bool> child_fault_{false};
  std::mutex goal_mtx_;
  std::weak_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>> active_goal_;
  rclcpp_action::GoalUUID pending_goal_uuid_{};
  bool goal_pending_{false};
  bool pending_cancel_{false};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp_action::Client<GoToWaypoint>::SharedPtr cli_wp_;
  rclcpp_action::Server<NavigateTo>::SharedPtr as_nav_;
};
}  // namespace easyfly_global_planner
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_global_planner::GlobalPlannerNode)
