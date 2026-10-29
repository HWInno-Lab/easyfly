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
// EasyFly E3: TrackTarget 跟随飞行节点 (M4.1)
//
// 数据流:
//   /target_positions (Target3DArray, odom 系, M5 输出)
//     → 目标选择 (goal.target_id, -1=最近; 同 id 观测受速度/单帧位移上限保护)
//     → 首次捕获时固定 odom 水平偏移，跟随点随目标平移
//     → GoToWaypoint 重发式跟随 (偏差 >1.0m 才重发, 静止零重发)
//
// 安全设计:
//   - 控制通道复用 flight_control 全部安全链 (渐近 setpoint/超时保护/接管检测);
//   - 丢失处理: first_seen_timeout 内无目标 abort; 跟随中丢失超过 lost_timeout
//     → cancel 航点 + 原地悬停 + abort (不盲飞);
//   - 接管传播: GoToWaypoint 返回 "external takeover" abort (低电量 RTL 等)
//     → TrackTarget 立即 abort, 让位给安全链;
//   - 仅悬停状态可发起 (自身不含起飞, 从地面发起会在首个航点发送时明确报错);
//   - 与任务 FSM 互斥 (flight_control 单动作槽), 不要同时使用。
#include <yaml-cpp/yaml.h>
#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <easyfly_msgs/action/track_target.hpp>
#include <easyfly_msgs/action/go_to_waypoint.hpp>
#include <easyfly_msgs/msg/target3_d_array.hpp>
#include <easyfly_msgs/msg/uav_state.hpp>
#include "easyfly_target_tracking/waypoint_lifecycle.hpp"
#include "easyfly_target_tracking/target_frame.hpp"
#include "easyfly_target_tracking/follow_offset.hpp"
namespace easyfly_target_tracking
{
using TrackTarget = easyfly_msgs::action::TrackTarget;
using GoToWaypoint = easyfly_msgs::action::GoToWaypoint;
using Target3D = easyfly_msgs::msg::Target3D;
using GoalHandleWp = rclcpp_action::ClientGoalHandle<GoToWaypoint>;
using namespace std::chrono_literals;
using namespace std::placeholders;
class TargetTrackingNode : public rclcpp::Node
{
public:
  explicit TargetTrackingNode(const rclcpp::NodeOptions & options)
  : Node("easyfly_target_tracking", options)
  {
    loadParams();
    auto qos = rclcpp::QoS(10);
    targets_sub_ = create_subscription<easyfly_msgs::msg::Target3DArray>(
      params_.input_topic, qos,
      [this](easyfly_msgs::msg::Target3DArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(snap_mtx_);
        latest_targets_ = *msg;
        ++targets_generation_;
        last_arrival_ = std::chrono::steady_clock::now();          // 数据断流检测
      });
    state_sub_ = create_subscription<easyfly_msgs::msg::UAVState>(
      params_.uav_state_topic, qos,
      [this](easyfly_msgs::msg::UAVState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(snap_mtx_);
        latest_state_ = *msg;
        state_received_ = true;
        last_state_arrival_ = std::chrono::steady_clock::now();
      });
    cli_wp_ = rclcpp_action::create_client<GoToWaypoint>(this, "/goto_waypoint");
    as_track_ = rclcpp_action::create_server<TrackTarget>(
      this, "/track_target",
      std::bind(&TargetTrackingNode::handle_goal, this, _1, _2),
      std::bind(&TargetTrackingNode::handle_cancel, this, _1),
      std::bind(&TargetTrackingNode::handle_accepted, this, _1));
    RCLCPP_INFO(
      get_logger(),
      "TargetTracking 启动: %s → /goto_waypoint (重发阈值 %.1fm, 丢失 %.1fs, "
      "高度上限 %.1fm, 任务半径 %.1fm, 目标速度上限 %.1fm/s)",
      params_.input_topic.c_str(), params_.reissue_threshold,
      params_.lost_timeout, params_.max_altitude,
      params_.max_follow_radius, params_.max_target_speed);
  }

private:
  struct NodeParams
  {
    std::string input_topic{"/target_positions"};
    std::string uav_state_topic{"/uav_state"};
    double reissue_threshold{1.0};
    double default_follow_distance{4.0};
    double default_follow_height{1.5};
    double max_altitude{2.5};
    // 观测安全边界: 允许正常的 1m/s SITL 行人运动和少量深度噪声,
    // 但任何单帧都不能跨越一个不可实现的距离。旧的连续异常放行
    // 机制已移除, 不再通过异常帧放行假目标。
    double max_target_speed{3.0};
    double max_target_jump{2.0};
    double jump_guard_margin{0.5};
    // 用相邻观测周期计算速度预算，并封顶，避免断帧后给单次跳变
    // 无限大的 dt。该值只影响速度预算，max_target_jump 仍是硬上限。
    double max_observation_gap{1.0};
    double max_follow_radius{35.0};
    double state_timeout{1.0};
    double child_cancel_timeout{5.0};
    double child_cancel_max_wait{30.0};
    double first_seen_timeout{15.0};
    double lost_timeout{5.0};
    double reacquire_radius{2.5};
  };
  void loadParams()
  {
    declare_parameter<std::string>("config_file", "");
    std::string cfg = get_parameter("config_file").as_string();
    if (cfg.empty()) {
      cfg = ament_index_cpp::get_package_share_directory("easyfly_target_tracking") +
        "/config/follow.yaml";
    }
    YAML::Node root = YAML::LoadFile(cfg);
    const auto & p = root["target_tracking"];
    params_.input_topic = p["input_topic"].as<std::string>(params_.input_topic);
    params_.uav_state_topic = p["uav_state_topic"].as<std::string>(params_.uav_state_topic);
    params_.reissue_threshold = p["reissue_threshold"].as<double>(params_.reissue_threshold);
    params_.default_follow_distance =
      p["default_follow_distance"].as<double>(params_.default_follow_distance);
    params_.default_follow_height =
      p["default_follow_height"].as<double>(params_.default_follow_height);
    params_.max_altitude = p["max_altitude"].as<double>(params_.max_altitude);
    params_.max_target_speed =
      p["max_target_speed"].as<double>(params_.max_target_speed);
    params_.max_target_jump =
      p["max_target_jump"].as<double>(params_.max_target_jump);
    params_.jump_guard_margin =
      p["jump_guard_margin"].as<double>(params_.jump_guard_margin);
    params_.max_observation_gap =
      p["max_observation_gap"].as<double>(params_.max_observation_gap);
    params_.max_follow_radius =
      p["max_follow_radius"].as<double>(params_.max_follow_radius);
    params_.state_timeout = p["state_timeout"].as<double>(params_.state_timeout);
    params_.child_cancel_timeout =
      p["child_cancel_timeout"].as<double>(params_.child_cancel_timeout);
    params_.child_cancel_max_wait =
      p["child_cancel_max_wait"].as<double>(params_.child_cancel_max_wait);
    params_.first_seen_timeout = p["first_seen_timeout"].as<double>(params_.first_seen_timeout);
    params_.lost_timeout = p["lost_timeout"].as<double>(params_.lost_timeout);
    params_.reacquire_radius = p["reacquire_radius"].as<double>(params_.reacquire_radius);
  }
  // ------------------------------------------------ 快照
  struct TargetSnapshot
  {
    easyfly_msgs::msg::Target3DArray message;
    uint64_t generation{0};
  };
  TargetSnapshot snapTargets()
  {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    return {latest_targets_, targets_generation_};
  }
  /// /target_positions 数据断流时长 [s] (感知节点死亡 → 到达时刻不再更新)
  double arrivalAgeSec()
  {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_arrival_).count();
  }
  /// /uav_state 新鲜度。尚未收到首帧时返回 +Inf，不能把默认零值当成
  /// 有效位姿用于发航点。
  double stateAgeSec()
  {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    if (!state_received_) {return std::numeric_limits<double>::infinity();}
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_state_arrival_).count();
  }
  easyfly_msgs::msg::UAVState snapState()
  {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    return latest_state_;
  }
  bool stateSafeForAnchor(const double anchor_x, const double anchor_y)
  {
    std::lock_guard<std::mutex> lk(snap_mtx_);
    if (!state_received_) {return false;}
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_state_arrival_).count();
    const auto & state = latest_state_;
    return age <= params_.state_timeout && state.connected && state.pose_valid &&
           std::isfinite(state.position_x) && std::isfinite(state.position_y) &&
           std::isfinite(state.position_z) && std::isfinite(state.yaw) &&
           std::hypot(state.position_x - anchor_x, state.position_y - anchor_y) <=
           params_.max_follow_radius;
  }
  // ------------------------------------------------ Action 服务端
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TrackTarget::Goal> goal)
  {
    if (goal->target_id == -2) {      // 停止跟踪: 总是接受 (作为对活跃跟随的停止信号)
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    std::lock_guard<std::mutex> lk(goal_mtx_);
    // handle_goal 与 handle_accepted 在 action server 的不同执行路径上
    // 到达；先占用 admission 槽，避免两个普通 goal 同时看到 active=false。
    if (active_ || goal_pending_) {
      RCLCPP_WARN(get_logger(), "已有跟随在执行, 拒绝新 goal (先发 target_id=-2 停止)");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target_id < -2) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_pending_ = true;
    pending_goal_uuid_ = uuid;
    pending_cancel_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> gh)
  {
    std::lock_guard<std::mutex> lk(goal_mtx_);
    // 只允许当前 active goal 取消；不能用全局标志误取消未来/其他 goal。
    if (auto current = active_goal_.lock(); current && current == gh) {
      cancel_req_ = true;        // 控制线程在轮询点响应: cancel 航点 + 悬停
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    if (goal_pending_ && gh && gh->get_goal_id() == pending_goal_uuid_) {
      pending_cancel_ = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    return rclcpp_action::CancelResponse::REJECT;
  }
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> gh)
  {
    if (gh->get_goal()->target_id == -2) {      // 停止信号
      {
        std::lock_guard<std::mutex> lk(goal_mtx_);
        if (active_goal_.lock()) {
          stop_req_ = true;
          cancel_req_ = true;
        } else if (goal_pending_) {
          // 普通 goal 已通过 handle_goal 但尚未进入线程；让它在
          // handle_accepted 时直接终止，避免 stop 被新 goal 覆盖。
          pending_cancel_ = true;
        }
      }
      // Do not report the stop command as complete while the active
      // parent/child action is still unwinding.  The waiter has a
      // finite bound; on failure it reports an explicit fault instead
      // of claiming that a child waypoint was already stopped.
      std::thread(
        [this, gh]() {
          const auto deadline = std::chrono::steady_clock::now() +
          std::chrono::duration<double>(
            std::max(1.0, params_.child_cancel_max_wait));
          while (rclcpp::ok()) {
            bool active = false;
            bool pending = false;
            {
              std::lock_guard<std::mutex> lk(goal_mtx_);
              active = active_.load();
              pending = goal_pending_;
            }
            if (!active && !pending) {break;}
            if (std::chrono::steady_clock::now() >= deadline) {
              auto res = std::make_shared<TrackTarget::Result>();
              res->success = false;
              res->message = "stop requested but tracking child did not terminate";
              try {gh->abort(res);} catch (...) {}
              return;
            }
            std::this_thread::sleep_for(100ms);
          }
          auto res = std::make_shared<TrackTarget::Result>();
          res->success = true;
          res->message = "tracking stopped";
          try {gh->succeed(res);} catch (...) {}
        }).detach();
      RCLCPP_INFO(get_logger(), "收到停止跟踪信号, 等待活动子航点终止");
      return;
    }
    {
      std::lock_guard<std::mutex> lk(goal_mtx_);
      if (!goal_pending_) {
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = "goal admission expired";
        try {gh->abort(res);} catch (...) {}
        return;
      }
      goal_pending_ = false;
      if (pending_cancel_) {
        pending_cancel_ = false;
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = "cancelled before start";
        try {gh->canceled(res);} catch (...) {}
        return;
      }
      active_ = true;
      active_goal_ = gh;
      cancel_req_ = false;
      stop_req_ = false;
      child_fault_ = false;
      waypoint_lifecycle_ = WaypointLifecycle();
      ++goal_generation_;
    }
    takeover_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());      // 跨跟随残留清零
    // mission_fsm 同款: detached 线程 + 顶层 try/catch (未捕获异常会杀死进程)
    std::thread(
      [this, gh]() {
        try {
          followLoop(gh);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "跟随线程异常: %s", e.what());
          if (!child_fault_) {
            try {
              auto res = std::make_shared<TrackTarget::Result>();
              res->success = false;
              res->message = std::string("internal error: ") + e.what();
              gh->abort(res);
            } catch (...) {}
          } else {
            RCLCPP_ERROR(
              get_logger(),
              "子航点故障未确认终态，保持 TrackTarget parent active");
          }
        }
      }).detach();
  }
  // ------------------------------------------------ 跟随控制线程
  /// 目标查找: id 精确匹配 (target_id<0 时取距无人机最近的 valid 目标)
  /// 返回 false = 本帧无有效观测
  bool findTarget(
    const easyfly_msgs::msg::Target3DArray & arr, int target_id,
    const easyfly_msgs::msg::UAVState & uav, Target3D & out)
  {
    const bool nearest = target_id < 0;
    double best = 1e18;
    bool found = false;
    for (const auto & t : arr.targets) {
      if (!t.position_valid) {continue;}
      if (!nearest && t.track_id != target_id) {continue;}
      if (nearest) {
        const double dx = t.position_x - uav.position_x;
        const double dy = t.position_y - uav.position_y;
        const double d = dx * dx + dy * dy;
        if (d >= best) {continue;}
        best = d;
      }
      out = t;
      found = true;
      if (!nearest) {break;}
    }
    return found;
  }
  static bool finiteTarget(const Target3D & t)
  {
    return std::isfinite(static_cast<double>(t.position_x)) &&
           std::isfinite(static_cast<double>(t.position_y)) &&
           std::isfinite(static_cast<double>(t.position_z));
  }
  /// 半径内最近目标 (重捕获用)。重捕获必须保持原目标类别，不能把
  /// 另一个类别的目标当成 SORT 的新 ID。
  bool findNearestTo(
    double x, double y, double radius,
    const easyfly_msgs::msg::Target3DArray & arr,
    const std::string & expected_class,
    Target3D & out)
  {
    double best = radius * radius;
    bool found = false;
    for (const auto & t : arr.targets) {
      if (!t.position_valid || !finiteTarget(t)) {continue;}
      if (t.class_name != expected_class) {continue;}
      const double dx = t.position_x - x;
      const double dy = t.position_y - y;
      const double d = dx * dx + dy * dy;
      if (d < best) {
        best = d;
        out = t;
        found = true;
      }
    }
    return found;
  }
  void clearActiveGoal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> & gh)
  {
    std::lock_guard<std::mutex> lk(goal_mtx_);
    if (auto current = active_goal_.lock(); current && current == gh) {
      active_goal_.reset();
      active_ = false;
      cancel_req_ = false;
      stop_req_ = false;
    }
  }
  /// 跳变拒绝: 同 id 的观测必须满足速度和单帧位移上限。
  ///
  /// 旧实现会在连续三次异常后放行任意跳变，这会把持续发布的假目标
  /// 直接变成跟随航点。这里按唯一的输入快照只处理一次，并且永远不
  /// 通过异常次数放宽阈值；短暂毛刺被丢弃，持续异常最终由 lost_timeout
  /// 走 cancel 航点 + 悬停 + ABORTED 路径。
  bool jumpGuard(int id, const Target3D & t, uint64_t generation)
  {
    const auto guard_it = guard_generation_.find(id);
    if (guard_it != guard_generation_.end() && guard_it->second == generation) {
      return guard_result_[id];
    }
    guard_generation_[id] = generation;
    // 记录每个 id 的相邻观测到达时间，必须在判定前更新，拒绝帧也
    // 算作一次观测。dt 不再从“最后一次接受”累积，否则持续发布的
    // 519m 假目标会靠等待逐渐获得足够大的速度预算。
    const auto now_steady = std::chrono::steady_clock::now();
    double dt = 0.1;
    const auto observation_it = last_observation_time_.find(id);
    if (observation_it != last_observation_time_.end()) {
      dt = std::chrono::duration<double>(now_steady - observation_it->second).count();
      if (!std::isfinite(dt) || dt < 0.01) {dt = 0.01;}
    }
    last_observation_time_[id] = now_steady;
    dt = std::min(dt, std::max(0.01, params_.max_observation_gap));
    const auto reject = [this, id]() {
        guard_result_[id] = false;
        return false;
      };
    if (!finiteTarget(t)) {return reject();}
    auto it = last_accepted_.find(id);
    if (it == last_accepted_.end()) {
      last_accepted_[id] = {t.position_x, t.position_y, t.position_z};
      guard_result_[id] = true;
      return true;
    }
    const double dx = t.position_x - it->second[0];
    const double dy = t.position_y - it->second[1];
    const double dz = t.position_z - it->second[2];
    const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
    // 单帧上限同时防止长时间断帧后通过 speed*dt 放行巨跳。
    const double allowed = std::min(
      params_.max_target_jump,
      params_.max_target_speed * dt + params_.jump_guard_margin);
    if (step > allowed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "拒绝目标 #%d 跳变 %.2fm (允许 %.2fm, dt=%.2fs)",
        id, step, allowed, dt);
      return reject();
    }
    last_accepted_[id] = {t.position_x, t.position_y, t.position_z};
    guard_result_[id] = true;
    return true;
  }
  /// 轮询等待 future, 期间响应取消/停止和状态安全检查.
  /// 返回 0=ready, 1=timeout, 2=中断, 3=安全条件失效。
  template<typename F>
  int pollFuture(
    F & fut, double timeout_s,
    const std::function<bool()> & safety_check = {})
  {
    if (!fut.valid()) {return 1;}
    const auto t0 = std::chrono::steady_clock::now();
    while (true) {
      if (cancel_req_ || stop_req_) {return 2;}
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
  /// 发送一个航点 (阻塞至被接受). 成功后由调用方挂 result future.
  ///
  /// 返回 false 时，如果 wp_pending=true，表示 send future 尚未解析；这
  /// 时绝不能重发或丢弃 future，调用方必须先走 cancelWaypointUntilTerminal。
  bool issueWaypoint(
    double x, double y, double z, double max_speed,
    std::shared_ptr<GoalHandleWp> & gh,
    std::shared_future<typename GoalHandleWp::SharedPtr> & goal_fut,
    bool & wp_pending,
    const std::function<bool()> & safety_check)
  {
    if (!waypoint_lifecycle_.beginSend()) {return false;}
    GoToWaypoint::Goal g;
    g.target_x = x;
    g.target_y = y;
    g.target_z = z;
    // GoToWaypoint defines NaN as hold current heading; its default zero
    // would rotate an oblique camera away from the acquired target.
    g.target_yaw = std::numeric_limits<double>::quiet_NaN();
    g.max_speed = max_speed;
    g.acceptance_radius = 0.5;
    const auto server_deadline = std::chrono::steady_clock::now() + 3s;
    while (!cli_wp_->wait_for_action_server(100ms)) {
      if (cancel_req_ || stop_req_ || (safety_check && !safety_check())) {
        waypoint_lifecycle_.resolveSend(false);
        return false;
      }
      if (std::chrono::steady_clock::now() >= server_deadline) {
        RCLCPP_ERROR(get_logger(), "GoToWaypoint action server 不可用");
        waypoint_lifecycle_.resolveSend(false);
        return false;
      }
    }
    try {
      goal_fut = cli_wp_->async_send_goal(g);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "发送 GoToWaypoint 异常: %s", e.what());
      waypoint_lifecycle_.resolveSend(false);
      return false;
    }
    if (!goal_fut.valid()) {
      RCLCPP_ERROR(get_logger(), "发送 GoToWaypoint 返回无效 future");
      waypoint_lifecycle_.resolveSend(false);
      return false;
    }
    wp_pending = true;
    const int pst = pollFuture(goal_fut, 10.0, safety_check);
    if (pst != 0) {
      // 中断/超时不再短暂等待后丢弃 future。它可能稍后返回一个已
      // 接受的 child，留在 wp_pending 中由确定性的取消路径接管。
      if (pst == 1) {
        RCLCPP_WARN(
          get_logger(),
          "GoToWaypoint send future 超时, 保留 pending child 等待解析");
      }
      return false;
    }
    try {
      gh = goal_fut.get();
    } catch (const std::exception & e) {
      // An exceptional send future can never yield a handle on a later
      // read.  Drop only this future and move to unknown-handle
      // recovery; the parent and any late server-side child ownership
      // remain held by the follow thread.
      goal_fut = std::shared_future<typename GoalHandleWp::SharedPtr>();
      wp_pending = false;
      waypoint_lifecycle_.markFault();
      RCLCPP_ERROR(get_logger(), "解析 GoToWaypoint future 异常: %s", e.what());
      return false;
    } catch (...) {
      goal_fut = std::shared_future<typename GoalHandleWp::SharedPtr>();
      wp_pending = false;
      waypoint_lifecycle_.markFault();
      RCLCPP_ERROR(get_logger(), "解析 GoToWaypoint future 未知异常");
      return false;
    }
    wp_pending = false;
    if (!gh) {
      waypoint_lifecycle_.resolveSend(false);
      RCLCPP_ERROR(
        get_logger(),
        "GoToWaypoint 被拒 (flight_control 不在 HOVER? 需先起飞悬停)");
      return false;
    }
    waypoint_lifecycle_.resolveSend(true);
    return true;
  }
  /// 取消当前航点并等待其终态 (CANCELED/SUCCEEDED/ABORTED 都代表
  /// flight_control 已回 HOVER, 可接受新 goal)。正常窗口超时后进入
  /// 持续 recovery，保留 handle/future 直到终态；只有 shutdown 才返回 false。
  bool cancelWaypoint(
    std::shared_ptr<GoalHandleWp> & gh,
    std::shared_future<typename GoalHandleWp::SharedPtr> & goal_fut,
    std::shared_future<rclcpp_action::Client<GoToWaypoint>::WrappedResult> & res_fut,
    bool & wp_has, bool & wp_pending, bool & cancel_sent,
    double timeout_s = 5.0)
  {
    try {
      if (waypoint_lifecycle_.state() == WaypointLifecycleState::kIdle) {return true;}
      waypoint_lifecycle_.requestCancel();
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(std::max(0.1, timeout_s));
      // send future 尚未解析时，先等它解析。若后来被接受，立即挂上
      // result future 并进入同一 cancel 流程；若被拒绝，则安全回到 IDLE。
      if (wp_pending) {
        if (!goal_fut.valid()) {return false;}
        while (goal_fut.wait_for(100ms) != std::future_status::ready) {
          if (!rclcpp::ok() || std::chrono::steady_clock::now() >= deadline) {
            return false;
          }
        }
        try {
          gh = goal_fut.get();
        } catch (const std::exception & e) {
          // This pending future is permanently exceptional.  Clear
          // it, leave pending state and retain fault ownership;
          // recovery must not cancel another client's goal.
          goal_fut = std::shared_future<typename GoalHandleWp::SharedPtr>();
          wp_pending = false;
          wp_has = false;
          waypoint_lifecycle_.markFault();
          RCLCPP_WARN(get_logger(), "解析 pending 航点 future 异常: %s", e.what());
          return false;
        } catch (...) {
          goal_fut = std::shared_future<typename GoalHandleWp::SharedPtr>();
          wp_pending = false;
          wp_has = false;
          waypoint_lifecycle_.markFault();
          RCLCPP_WARN(get_logger(), "解析 pending 航点 future 未知异常");
          return false;
        }
        wp_pending = false;
        if (!gh) {
          goal_fut = std::shared_future<typename GoalHandleWp::SharedPtr>();
          wp_has = false;
          waypoint_lifecycle_.resolveSend(false);
          return true;
        }
        wp_has = true;
        waypoint_lifecycle_.resolveSend(true);
      }
      if (!wp_has) {
        return !wp_pending;
      }
      if (!gh) {return false;}
      // A child may have reached a terminal state just before the
      // parent cancellation request.  Consume that result first;
      // canceling an already-terminal handle can throw and strand
      // recovery forever.
      if (res_fut.valid() &&
        res_fut.wait_for(0s) == std::future_status::ready)
      {
        if (!waypoint_lifecycle_.consumeReadyResult(res_fut, cancel_sent)) {
          waypoint_lifecycle_.markFault();
          RCLCPP_WARN(
            get_logger(),
            "读取已完成航点终态异常，清空 future 后重试 cancel/result");
          return false;
        }
        if (!waypoint_lifecycle_.confirmTerminal()) {return false;}
        gh.reset();
        wp_has = false;
        cancel_sent = false;
        res_fut = std::shared_future<
          rclcpp_action::Client<GoToWaypoint>::WrappedResult>();
        goal_fut = std::shared_future<typename GoalHandleWp::SharedPtr>();
        return true;
      }
      // With no result future, ask for the result before cancellation.
      // A cancel transport exception must not skip a result future that
      // was already obtained (or becomes available while cancel fails).
      if (!waypoint_lifecycle_.prepareCancel(
          res_fut, cancel_sent,
          [this, &gh]() {return cli_wp_->async_get_result(gh);},
          [this, &gh]() {cli_wp_->async_cancel_goal(gh);}))
      {
        return false;
      }
      while (res_fut.wait_for(100ms) != std::future_status::ready) {
        if (!rclcpp::ok() || std::chrono::steady_clock::now() >= deadline) {
          return false;
        }
      }
      if (!waypoint_lifecycle_.consumeReadyResult(res_fut, cancel_sent)) {
        waypoint_lifecycle_.markFault();
        RCLCPP_WARN(
          get_logger(),
          "读取航点终态异常，清空 future 后重试 cancel/result");
        return false;
      }
      if (!waypoint_lifecycle_.confirmTerminal()) {return false;}
      gh.reset();
      wp_has = false;
      cancel_sent = false;
      res_fut = std::shared_future<rclcpp_action::Client<GoToWaypoint>::WrappedResult>();
      goal_fut = std::shared_future<typename GoalHandleWp::SharedPtr>();
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "航点取消状态机异常: %s", e.what());
      return false;
    } catch (...) {
      RCLCPP_WARN(get_logger(), "航点取消状态机未知异常");
      return false;
    }
  }
  bool cancelWaypointUntilTerminal(
    std::shared_ptr<GoalHandleWp> & gh,
    std::shared_future<typename GoalHandleWp::SharedPtr> & goal_fut,
    std::shared_future<rclcpp_action::Client<GoToWaypoint>::WrappedResult> & res_fut,
    bool & wp_has, bool & wp_pending, bool & cancel_sent)
  {
    if (waypoint_lifecycle_.state() == WaypointLifecycleState::kIdle) {return true;}
    bool unknown_handle = !wp_has && !wp_pending && !goal_fut.valid() &&
      waypoint_lifecycle_.state() == WaypointLifecycleState::kFault;
    if (unknown_handle) {
      // A send future that threw cannot be resolved into a handle.  Do
      // not spend the bounded per-goal window trying to read it again;
      // retain the parent and enter action-server-wide recovery.
      waypoint_lifecycle_.enterRecovery();
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(std::max(1.0, params_.child_cancel_max_wait));
    while (!unknown_handle && rclcpp::ok() && waypoint_lifecycle_.state() !=
      WaypointLifecycleState::kRecovering && waypoint_lifecycle_.state() !=
      WaypointLifecycleState::kIdle)
    {
      const auto remaining = std::chrono::duration<double>(
        deadline - std::chrono::steady_clock::now()).count();
      if (remaining <= 0.0) {break;}
      if (cancelWaypoint(
          gh, goal_fut, res_fut, wp_has, wp_pending, cancel_sent,
          std::min(params_.child_cancel_timeout, remaining)) &&
        waypoint_lifecycle_.state() == WaypointLifecycleState::kIdle)
      {
        return true;
      }
      if (!unknown_handle && !wp_has && !wp_pending && !goal_fut.valid() &&
        waypoint_lifecycle_.state() == WaypointLifecycleState::kFault)
      {
        unknown_handle = true;
        waypoint_lifecycle_.enterRecovery();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "等待子航点取消终态，暂不发布新航点");
      std::this_thread::sleep_for(100ms);
    }
    if (!rclcpp::ok()) {return false;}
    if (waypoint_lifecycle_.state() != WaypointLifecycleState::kIdle) {
      // The normal cancellation window is bounded, but ownership is
      // not.  Keep this follow thread and its strong goal/child context
      // alive in a low-rate recovery loop until terminal confirmation.
      waypoint_lifecycle_.enterRecovery();
      RCLCPP_ERROR(
        get_logger(),
        "子航点取消超过 %.1fs，进入持续 recovery；保持 child 所有权",
        params_.child_cancel_max_wait);
      int recovery_attempts = 0;
      while (rclcpp::ok() && waypoint_lifecycle_.state() !=
        WaypointLifecycleState::kIdle)
      {
        if (cancelWaypoint(
            gh, goal_fut, res_fut, wp_has, wp_pending, cancel_sent, 1.0) &&
          waypoint_lifecycle_.state() == WaypointLifecycleState::kIdle)
        {
          return true;
        }
        // Re-issue a per-goal cancel request at a low rate after a
        // transport/action-server failure; never infer terminal here.
        cancel_sent = false;
        ++recovery_attempts;
        if ((!wp_has && !wp_pending && !goal_fut.valid() &&
          (recovery_attempts == 1 || recovery_attempts % 10 == 0)) ||
          recovery_attempts % 10 == 0)
        {
          // A lost send response does not grant ownership of other
          // clients' goals. Keep our recovery slot and wait for the
          // exact child; cancel-all could stop unified navigation.
          RCLCPP_WARN(
            get_logger(),
            "子航点终态仍未确认，仅重试所属目标取消；保持 recovery");
        }
        std::this_thread::sleep_for(1s);
      }
    }
    return waypoint_lifecycle_.state() == WaypointLifecycleState::kIdle;
  }
  void followLoop(std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> gh)
  {
    // 这些引用贯穿整个 followLoop，包括异常展开阶段。这样 send future
    // 若在 issueWaypoint 超时后才解析，RAII 清理仍能拿到 child 并取消。
    std::shared_ptr<GoalHandleWp> wp_gh_;
    std::shared_future<typename GoalHandleWp::SharedPtr> wp_goal_fut_;
    std::shared_future<rclcpp_action::Client<GoToWaypoint>::WrappedResult> wp_res_fut_;
    bool wp_has_ = false;
    bool wp_pending_ = false;
    bool wp_cancel_sent_ = false;
    struct ActiveGoalCleanup
    {
      TargetTrackingNode * node;
      std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> goal;
      ~ActiveGoalCleanup() noexcept
      {
        // If a child could not be confirmed terminal, keep the
        // parent admission occupied.  Reporting a free slot here
        // would allow a second goal to race an orphan waypoint.
        if (node->waypoint_lifecycle_.mayIssue()) {
          node->child_fault_ = false;
          try {node->clearActiveGoal(goal);} catch (...) {}
        } else {
          node->child_fault_ = true;
          RCLCPP_ERROR(
            node->get_logger(),
            "父跟随保持 active：子航点终态未确认，禁止新 goal");
        }
      }
    } active_cleanup{this, gh};
    struct WaypointCleanup
    {
      TargetTrackingNode * node;
      std::shared_ptr<GoalHandleWp> * gh;
      std::shared_future<typename GoalHandleWp::SharedPtr> * goal_fut;
      std::shared_future<rclcpp_action::Client<GoToWaypoint>::WrappedResult> * res_fut;
      bool * has;
      bool * pending;
      bool * cancel_sent;
      ~WaypointCleanup() noexcept
      {
        try {
          node->cancelWaypointUntilTerminal(
            *gh, *goal_fut, *res_fut, *has, *pending, *cancel_sent);
        } catch (...) {
          RCLCPP_ERROR(
            node->get_logger(), "子航点 RAII 清理异常，保持父 goal active");
        }
      }
    } waypoint_cleanup{this, &wp_gh_, &wp_goal_fut_, &wp_res_fut_,
      &wp_has_, &wp_pending_, &wp_cancel_sent_};
    const auto goal = gh->get_goal();
    const int target_id = goal->target_id;
    const double follow_distance =
      std::isfinite(goal->follow_distance) && goal->follow_distance > 0 ?
      goal->follow_distance : params_.default_follow_distance;
    const double follow_height_req =
      std::isfinite(goal->follow_height) && goal->follow_height > 0 ?
      goal->follow_height : params_.default_follow_height;
    RCLCPP_INFO(
      get_logger(), "跟随发起: id=%d 距离=%.1fm 高度=%.1fm",
      target_id, follow_distance, follow_height_req);
    int cur_tid_ = target_id;
    // 每个跟随 goal 都建立自己的安全状态，不能继承上一 goal 的
    // 位置、异常计数或类别。任务半径以发起时无人机位置为锚，所有
    // 后续目标观测和下发航点都必须留在该边界内。
    last_accepted_.clear();
    last_observation_time_.clear();
    guard_generation_.clear();
    guard_result_.clear();
    const auto anchor_uav = snapState();
    const bool anchor_valid = anchor_uav.connected && anchor_uav.pose_valid &&
      std::isfinite(anchor_uav.position_x) &&
      std::isfinite(anchor_uav.position_y) &&
      std::isfinite(anchor_uav.position_z) &&
      std::isfinite(anchor_uav.yaw) &&
      stateAgeSec() <= params_.state_timeout;
    if (!anchor_valid) {
      auto res = std::make_shared<TrackTarget::Result>();
      res->success = false;
      res->message = "uav pose is not valid";
      gh->abort(res);
      return;
    }
    const double anchor_x = anchor_uav.position_x;
    const double anchor_y = anchor_uav.position_y;
    const auto within_task_radius = [this, anchor_x, anchor_y](double x, double y) {
        return std::isfinite(x) && std::isfinite(y) &&
               std::hypot(x - anchor_x, y - anchor_y) <= params_.max_follow_radius;
      };
    const auto waypoint_safety = [this, anchor_x, anchor_y]() {
        return stateSafeForAnchor(anchor_x, anchor_y);
      };
    // ---- 等待首次捕获 ----
    Target3D tgt;
    std::string locked_class;
    std::string acquisition_error;
    {
      const auto t0 = std::chrono::steady_clock::now();
      bool found = false;
      while (std::chrono::steady_clock::now() - t0 <
        std::chrono::duration<double>(params_.first_seen_timeout))
      {
        if (cancel_req_ || stop_req_) {break;}
        const auto arr = snapTargets();
        // generation==0 is the default-constructed startup snapshot,
        // not a received message.  Once a sample arrives, an empty
        // or non-odom frame is a hard coordinate-contract failure.
        if (arr.generation > 0 && !targetArrayFrameValid(arr.message)) {
          acquisition_error = "target frame_id must be odom";
          break;
        }
        const auto uav = snapState();
        if (!findTarget(arr.message, target_id, uav, tgt)) {
          std::this_thread::sleep_for(100ms);
          continue;
        }
        if (!finiteTarget(tgt)) {
          acquisition_error = "target position is not finite";
          break;
        }
        if (!within_task_radius(tgt.position_x, tgt.position_y)) {
          acquisition_error = "target outside max follow radius";
          break;
        }
        if (jumpGuard(tgt.track_id, tgt, arr.generation)) {
          locked_class = tgt.class_name;
          if (cur_tid_ < 0) {cur_tid_ = tgt.track_id;}
          found = true;
          break;
        }
        std::this_thread::sleep_for(100ms);
      }
      if (cancel_req_ || stop_req_) {
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = "cancelled before target acquired";
        gh->canceled(res);
        return;
      }
      if (!acquisition_error.empty()) {
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = acquisition_error;
        gh->abort(res);
        return;
      }
      if (!found) {
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = "target not found (" +
          std::to_string(params_.first_seen_timeout) + "s)";
        gh->abort(res);
        return;
      }
    }
    const auto offset = followOffset(
      tgt.position_x, tgt.position_y, anchor_x, anchor_y,
      anchor_uav.yaw, follow_distance);
    auto last_seen = std::chrono::steady_clock::now();
    // ---- 跟随主循环 ----
    bool takeover_ = false;
    double cur_wp_x_ = 0, cur_wp_y_ = 0;
    bool first_issue_ = true;
    while (true) {
      if (cancel_req_ || stop_req_ || takeover_) {
        if (!cancelWaypointUntilTerminal(
            wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
            wp_cancel_sent_))
        {
          return;
        }
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = stop_req_ ? "stopped" :
          (takeover_ ? "external takeover (RTL/AUTO.LAND)" : "cancelled");
        // 终态语义: 用户主动取消=CANCELED, 接管/异常=ABORTED, 停止指令=SUCCEEDED
        if (stop_req_) {gh->succeed(res);} else if (cancel_req_) {gh->canceled(res);} else {
          gh->abort(res);
        }
        break;
      }
      const auto arr = snapTargets();
      if (arr.generation > 0 && !targetArrayFrameValid(arr.message)) {
        RCLCPP_ERROR(
          get_logger(),
          "跟随安全终止: Target3DArray frame_id 不是 odom (收到 '%s')",
          arr.message.header.frame_id.c_str());
        if (!cancelWaypointUntilTerminal(
            wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
            wp_cancel_sent_))
        {
          return;
        }
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = "target frame_id must be odom";
        gh->abort(res);
        break;
      }
      const auto uav = snapState();
      const bool uav_state_valid = uav.connected && uav.pose_valid &&
        std::isfinite(uav.position_x) && std::isfinite(uav.position_y) &&
        std::isfinite(uav.position_z) && std::isfinite(uav.yaw) &&
        stateAgeSec() <= params_.state_timeout;
      const bool uav_inside_task_radius = uav_state_valid &&
        within_task_radius(uav.position_x, uav.position_y);
      if (!uav_inside_task_radius) {
        RCLCPP_ERROR(
          get_logger(),
          "跟随安全终止: UAV state invalid/stale/outside anchor radius "
          "(age=%.2fs)",
          stateAgeSec());
        if (!cancelWaypointUntilTerminal(
            wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
            wp_cancel_sent_))
        {
          return;
        }
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = "uav state invalid or stale";
        gh->abort(res);
        break;
      }
      Target3D t;
      bool seen = false;
      bool abnormal_observation = false;
      std::string abnormal_reason;
      if (findTarget(arr.message, cur_tid_, uav, t)) {
        if (!finiteTarget(t)) {
          abnormal_observation = true;
          abnormal_reason = "target position is not finite";
        } else if (t.class_name != locked_class) {
          // 同一 ID 被错误复用到另一类别时不能继续跟随；只有
          // 同类别且在重捕获半径内的新 ID 才允许接管。
          abnormal_observation = true;
          abnormal_reason = "target class changed";
        } else if (!within_task_radius(t.position_x, t.position_y)) {
          abnormal_observation = true;
          abnormal_reason = "target outside max follow radius";
        } else {
          seen = jumpGuard(cur_tid_, t, arr.generation);
        }
      }
      // 越界、NaN/Inf、类别漂移是安全异常，不等待 lost_timeout，也不
      // 让当前航点继续把无人机带向异常观测。
      if (abnormal_observation) {
        RCLCPP_ERROR(get_logger(), "跟随安全终止: %s", abnormal_reason.c_str());
        if (!cancelWaypointUntilTerminal(
            wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
            wp_cancel_sent_))
        {
          return;
        }
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = abnormal_reason;
        gh->abort(res);
        break;
      }
      if (seen) {last_seen = std::chrono::steady_clock::now();}
      // ---- id 轮换重捕获 ----
      // SORT 在视角/检出变化时会重新分配 id (Q19: 飞行中 ~26 个/次),
      // 锁定单一 id 在实战中脆弱。丢失判定前先在"最后已知位置"附近
      // (reacquire_radius) 认领新 id —— 等价于操作员视觉重锁。
      if (!seen && last_accepted_.count(cur_tid_)) {
        const auto arr2 = snapTargets();
        if (arr2.generation > 0 && !targetArrayFrameValid(arr2.message)) {
          RCLCPP_ERROR(
            get_logger(),
            "跟随安全终止: 重捕获 Target3DArray frame_id 不是 odom");
          if (!cancelWaypointUntilTerminal(
              wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
              wp_cancel_sent_))
          {
            return;
          }
          auto res = std::make_shared<TrackTarget::Result>();
          res->success = false;
          res->message = "target frame_id must be odom";
          gh->abort(res);
          break;
        }
        Target3D cand;
        const bool candidate_found = findNearestTo(
          last_accepted_[cur_tid_][0], last_accepted_[cur_tid_][1],
          params_.reacquire_radius, arr2.message, locked_class, cand);
        if (candidate_found && !within_task_radius(cand.position_x, cand.position_y)) {
          RCLCPP_ERROR(
            get_logger(),
            "跟随安全终止: reacquire candidate outside max follow radius");
          if (!cancelWaypointUntilTerminal(
              wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
              wp_cancel_sent_))
          {
            return;
          }
          auto res = std::make_shared<TrackTarget::Result>();
          res->success = false;
          res->message = "reacquire candidate outside max follow radius";
          gh->abort(res);
          break;
        }
        if (candidate_found && jumpGuard(cand.track_id, cand, arr2.generation)) {
          RCLCPP_WARN(
            get_logger(), "id 轮换重捕获: %d → %d (@ %.2f,%.2f)",
            cur_tid_, cand.track_id, cand.position_x, cand.position_y);
          last_accepted_.erase(cur_tid_);             // 旧 id 状态清理 (防 map 无界增长)
          last_observation_time_.erase(cur_tid_);
          guard_generation_.erase(cur_tid_);
          guard_result_.erase(cur_tid_);
          cur_tid_ = cand.track_id;
          t = cand;
          last_seen = std::chrono::steady_clock::now();
          seen = true;
        }
      }
      // ---- 丢失处理: cancel 航点 + 悬停 + abort ----
      // 两路信号: (a) /target_positions 断流 (快照是缓存的最后一帧,
      // 不按到达时刻判定的话, 感知节点死亡后 seen 永远为 true — 实测);
      // (b) 跟随 id 长时间无有效观测 (数据在流但目标消失)
      const double stale_s = arrivalAgeSec();
      const double lost_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_seen).count();
      if (stale_s > params_.lost_timeout || lost_s > params_.lost_timeout) {
        RCLCPP_WARN(
          get_logger(), "目标丢失超过 %.0fs, 悬停并终止跟随",
          params_.lost_timeout);
        if (!cancelWaypointUntilTerminal(
            wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
            wp_cancel_sent_))
        {
          return;
        }
        auto res = std::make_shared<TrackTarget::Result>();
        res->success = false;
        res->message = "target lost";
        gh->abort(res);
        break;
      }
      // ---- 外部接管检测: PX4 被切 RTL/AUTO.LAND (低电量/地面站) ----
      // 直接读 flight_mode (比等航点 abort 消息更快更可靠: 丢失判定
      // 5s 可能先触发)。持续 2s 去抖 (模式反馈链有 ~1s 延迟)。
      {
        const int mode = uav.flight_mode;
        if (mode == 6 || mode == 7) {
          if (takeover_since_.nanoseconds() == 0) {
            takeover_since_ = now();
          } else if ((now() - takeover_since_).seconds() > 2.0) {
            RCLCPP_WARN(
              get_logger(),
              "检测到外部接管 (mode=%d), 跟随让位", mode);
            takeover_ = true;
            continue;
          }
        } else {
          takeover_since_ = rclcpp::Time(0, 0, now().get_clock_type());
        }
      }
      // ---- 航点结果检查: 接管传播 / 终态清位 ----
      if (wp_has_ && wp_res_fut_.valid() &&
        wp_res_fut_.wait_for(0s) == std::future_status::ready)
      {
        rclcpp_action::Client<GoToWaypoint>::WrappedResult wrapped;
        try {
          wrapped = wp_res_fut_.get();
        } catch (const std::exception & e) {
          // A ready exceptional shared_future is permanently
          // exceptional.  Clear it so the recovery pass requests a
          // fresh result future and re-sends cancellation while
          // retaining wp_gh_ and the parent goal.
          wp_res_fut_ = std::shared_future<
            rclcpp_action::Client<GoToWaypoint>::WrappedResult>();
          wp_cancel_sent_ = false;
          waypoint_lifecycle_.markFault();
          RCLCPP_ERROR(get_logger(), "读取航点终态异常: %s", e.what());
          if (!cancelWaypointUntilTerminal(
              wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
              wp_cancel_sent_))
          {
            return;
          }
          auto res = std::make_shared<TrackTarget::Result>();
          res->success = false;
          res->message = "waypoint result read failed";
          gh->abort(res);
          break;
        }
        if (!waypoint_lifecycle_.confirmTerminal()) {
          RCLCPP_ERROR(get_logger(), "航点终态与生命周期状态不一致");
          if (!cancelWaypointUntilTerminal(
              wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
              wp_cancel_sent_))
          {
            return;
          }
        }
        wp_has_ = false;
        wp_gh_.reset();
        wp_cancel_sent_ = false;
        wp_res_fut_ = std::shared_future<
          rclcpp_action::Client<GoToWaypoint>::WrappedResult>();
        wp_goal_fut_ = std::shared_future<typename GoalHandleWp::SharedPtr>();
        if (wrapped.code == rclcpp_action::ResultCode::ABORTED &&
          wrapped.result && wrapped.result->message.find("takeover") !=
          std::string::npos)
        {
          RCLCPP_WARN(
            get_logger(), "航点被外部接管中止 (%s), 跟随让位",
            wrapped.result->message.c_str());
          takeover_ = true;
          continue;
        }
        // SUCCEEDED (到达跟随点) / CANCELED / ABORTED(超时就地悬停):
        // 均回到 HOVER, 等下一帧目标位置触发重发
      }
      // ---- 跟随点计算与航点重发 ----
      if (seen) {
        // 固定首次捕获的方位，否则沿当前视线退距离会抵消横向运动，
        // 目标往返数米而航点仍停在原地。ID 重捕获不改变同一任务偏移。
        const double fx = t.position_x + offset[0];
        const double fy = t.position_y + offset[1];
        if (!within_task_radius(fx, fy)) {
          RCLCPP_ERROR(
            get_logger(),
            "跟随安全终止: follow waypoint outside max follow radius");
          if (!cancelWaypointUntilTerminal(
              wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
              wp_cancel_sent_))
          {
            return;
          }
          auto res = std::make_shared<TrackTarget::Result>();
          res->success = false;
          res->message = "follow waypoint outside max follow radius";
          gh->abort(res);
          break;
        }
        // 高度双端 clamp: 深度毛刺可给出异常 target_z (实测 -1.4 → 跟随点
        // 落地), 下限防砸地, 上限为 SLAM 盲区约束
        const double fz = std::clamp(
          t.position_z + follow_height_req, 0.8, params_.max_altitude);
        // 重发条件 = 跟随点相对"上次下发点"移动超阈值 (水平)。
        // 注意不含 !wp_has_: 航点完成后 flight_control 已在跟随点悬停,
        // 若无条件重发会形成 300ms 一次的 0.0Xm 微航点风暴 (实测)。
        const bool need =
          first_issue_ ||
          std::sqrt(std::pow(fx - cur_wp_x_, 2) + std::pow(fy - cur_wp_y_, 2)) >
          params_.reissue_threshold;
        if (need) {
          if (wp_has_ || wp_pending_) {
            if (!cancelWaypointUntilTerminal(
                wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
                wp_cancel_sent_))
            {
              return;
            }
          }
          if (cancel_req_ || stop_req_ || takeover_) {continue;}
          // 首个航点礼貌重试: flight_control 从 "上个动作结束" 到 HOVER
          // 存在时序窗口 (如跟随紧接起飞发起), REJECT 不是错误而是"未就绪"
          bool issued = false;
          for (int attempt = 0; attempt < 10 && !issued; ++attempt) {
            if (issueWaypoint(
                fx, fy, fz, 2.5, wp_gh_, wp_goal_fut_, wp_pending_,
                waypoint_safety))
            {
              issued = true;
              break;
            }
            if (wp_pending_) {
              // send future 未完成时禁止重试；先解析并取消，防止
              // 稍后被接受的 orphan child 继续驱动 flight_control。
              if (!cancelWaypointUntilTerminal(
                  wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
                  wp_cancel_sent_))
              {
                return;
              }
              if (cancel_req_ || stop_req_) {break;}
              auto res = std::make_shared<TrackTarget::Result>();
              res->success = false;
              res->message = "waypoint send unresolved; child cancellation required";
              gh->abort(res);
              break;
            }
            if (!waypoint_safety()) {
              // Safety became invalid while waiting/retrying;
              // never issue another child and do not report a
              // terminal parent until any existing child is
              // confirmed terminal.
              if (!cancelWaypointUntilTerminal(
                  wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
                  wp_cancel_sent_))
              {
                return;
              }
              auto res = std::make_shared<TrackTarget::Result>();
              res->success = false;
              res->message = "uav state invalid during waypoint send";
              gh->abort(res);
              break;
            }
            if (cancel_req_ || stop_req_) {break;}
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 3000,
              "航点未就绪 (等 HOVER), 重试 %d/10", attempt + 1);
            std::this_thread::sleep_for(1s);
          }
          if (issued) {
            // Mark the handle before acquiring the result future;
            // an exception here must still enter the cancel path.
            wp_has_ = true;
            try {
              wp_res_fut_ = cli_wp_->async_get_result(wp_gh_);
            } catch (const std::exception & e) {
              RCLCPP_ERROR(
                get_logger(),
                "获取新航点 result future 异常: %s", e.what());
              if (!cancelWaypointUntilTerminal(
                  wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
                  wp_cancel_sent_))
              {
                return;
              }
              auto res = std::make_shared<TrackTarget::Result>();
              res->success = false;
              res->message = "waypoint result unavailable";
              gh->abort(res);
              break;
            }
            if (!wp_res_fut_.valid()) {
              RCLCPP_ERROR(get_logger(), "航点 result future 无效");
              if (!cancelWaypointUntilTerminal(
                  wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
                  wp_cancel_sent_))
              {
                return;
              }
              auto res = std::make_shared<TrackTarget::Result>();
              res->success = false;
              res->message = "waypoint result future invalid";
              gh->abort(res);
              break;
            }
            cur_wp_x_ = fx;
            cur_wp_y_ = fy;
            wp_cancel_sent_ = false;
            first_issue_ = false;
            RCLCPP_INFO(get_logger(), "跟随点 → (%.2f, %.2f, %.2f)", fx, fy, fz);
          } else {
            if (wp_pending_ || wp_has_ ||
              waypoint_lifecycle_.state() != WaypointLifecycleState::kIdle)
            {
              // 这里只能在 shutdown 或 action future 异常时到达；
              // 未确认终态时不得终止父 goal；RAII 会继续持有并清理 child。
              if (!cancelWaypointUntilTerminal(
                  wp_gh_, wp_goal_fut_, wp_res_fut_, wp_has_, wp_pending_,
                  wp_cancel_sent_))
              {
                return;
              }
              auto res = std::make_shared<TrackTarget::Result>();
              res->success = false;
              res->message = "waypoint child unresolved";
              gh->abort(res);
              break;
            }
            if (cancel_req_ || stop_req_) {continue;}
            auto res = std::make_shared<TrackTarget::Result>();
            res->success = false;
            res->message = "waypoint rejected (need takeoff+hover first?)";
            gh->abort(res);
            break;
          }
        }
      }
      // ---- 反馈 ----
      try {
        auto fb = std::make_shared<TrackTarget::Feedback>();
        fb->current_target_id = cur_tid_;
        fb->target_visible = seen;
        fb->distance_to_target = seen ?
          std::sqrt(
          std::pow(t.position_x - uav.position_x, 2) +
          std::pow(t.position_y - uav.position_y, 2)) :
          0.0;
        gh->publish_feedback(fb);
      } catch (...) {}
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(), "跟随结束");
  }
  // ------------------------------------------------ 成员
  NodeParams params_;
  // Single-goal admission guarantees only one follow thread owns this
  // lifecycle at a time.  Keeping it on the node lets recovery outlive the
  // bounded normal-cancel phase without destroying child ownership.
  WaypointLifecycle waypoint_lifecycle_;
  std::mutex snap_mtx_;
  std::chrono::steady_clock::time_point last_arrival_{std::chrono::steady_clock::now()};
  easyfly_msgs::msg::Target3DArray latest_targets_;
  uint64_t targets_generation_{0};
  easyfly_msgs::msg::UAVState latest_state_;
  bool state_received_{false};
  std::chrono::steady_clock::time_point last_state_arrival_{};
  rclcpp::Time takeover_since_{0, 0, RCL_ROS_TIME};    // 接管检测去抖 (跟随线程私有)
  std::map<int, std::array<double, 3>> last_accepted_;    // 跳变拒绝 (跟随线程私有)
  std::map<int, std::chrono::steady_clock::time_point> last_observation_time_;
  std::map<int, uint64_t> guard_generation_;
  std::map<int, bool> guard_result_;
  std::mutex goal_mtx_;
  std::weak_ptr<rclcpp_action::ServerGoalHandle<TrackTarget>> active_goal_;
  rclcpp_action::GoalUUID pending_goal_uuid_{};
  bool goal_pending_{false};
  bool pending_cancel_{false};
  uint64_t goal_generation_{0};
  std::atomic<bool> active_{false};
  std::atomic<bool> cancel_req_{false};
  std::atomic<bool> stop_req_{false};
  std::atomic<bool> child_fault_{false};
  rclcpp::Subscription<easyfly_msgs::msg::Target3DArray>::SharedPtr targets_sub_;
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp_action::Client<GoToWaypoint>::SharedPtr cli_wp_;
  rclcpp_action::Server<TrackTarget>::SharedPtr as_track_;
};
}  // namespace easyfly_target_tracking
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_target_tracking::TargetTrackingNode)
