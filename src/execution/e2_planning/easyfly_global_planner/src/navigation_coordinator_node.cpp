#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <easyfly_msgs/action/navigate_to.hpp>
#include <easyfly_msgs/action/follow_trajectory.hpp>
#include <easyfly_msgs/msg/uav_state.hpp>
#include <easyfly_msgs/msg/navigation_session.hpp>
#include <easyfly_msgs/srv/set_navigation_session.hpp>
#include <easyfly_msgs/srv/stop_navigation.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <easyfly_core/ownership_registry.hpp>
#include <easyfly_msgs/srv/get_action_ownership.hpp>
#include "easyfly_astar/astar.hpp"
#include "easyfly_global_planner/adapters/planner_factory.hpp"
#include "easyfly_global_planner/map_validity.hpp"

namespace easyfly_global_planner {
namespace gp = easyfly::grid_planning;
using easyfly::astar::OccupancyGrid2d;
// All callbacks use the default mutually exclusive group, also when composed.
// Public termination does not release unresolved child or planner ownership.
class NavigationCoordinatorNode : public rclcpp::Node {
  using Nav = easyfly_msgs::action::NavigateTo;
  using Follow = easyfly_msgs::action::FollowTrajectory;
  using Session = easyfly_msgs::srv::SetNavigationSession;
  using Parent = rclcpp_action::ServerGoalHandle<Nav>;
  using AChild = rclcpp_action::ClientGoalHandle<Nav>;
  using EChild = rclcpp_action::ClientGoalHandle<Follow>;
  using Clock = std::chrono::steady_clock;
  struct Context {
    std::shared_ptr<Parent> parent;
    AChild::SharedPtr astar;
    EChild::SharedPtr ego;
    bool sent{false}, terminal{true}, cancel_sent{false};
    bool start_sent{false}, start_done{false}, stop_sent{false}, stopped{false};
    bool draining{false}, finished{false}, user_cancel{false}, supervisor{false}, replan{false};
    bool child_success{false};
    bool normal_completion{false};
    std::string reason, session;
    std::vector<std::pair<double, double>> route;
    size_t segment{0};
    int replans{0}, sequence{0};
    double z{0}, speed{0};
    Clock::time_point created{Clock::now()}, phase{Clock::now()}, drain{Clock::now()};
  };
public:
  explicit NavigationCoordinatorNode(const rclcpp::NodeOptions & options)
  : Node("navigation_coordinator", options) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto localization = declare_parameter<std::string>(
      "localization_backend", "fast_lio", descriptor);
    if (localization != "fast_lio" && localization != "openvins" && localization != "disabled")
      throw std::invalid_argument("unsupported localization backend");
    navigation_enabled_ = localization == "fast_lio";
    if (!navigation_enabled_) RCLCPP_WARN(get_logger(),
      "Navigation unavailable for selected localization backend: %s; frame/map integration required",
      localization.c_str());
    combined_planner_ = make_planner("astar", {1000000, 0});
    strategy_ = declare_parameter<std::string>("strategy", "astar");
    if (strategy_ != "astar" && strategy_ != "ego" && strategy_ != "combined")
      throw std::invalid_argument("strategy must be astar, ego or combined");
    auto backend = declare_parameter<std::string>("astar_action", "/navigate_astar");
    if (backend.empty() || backend.front() != '/' || backend == "/navigate")
      throw std::invalid_argument("private backend must be absolute and distinct from /navigate");
    response_timeout_ = positive("response_timeout", 5.0);
    cancel_timeout_ = positive("cancel_timeout", 5.0);
    goal_timeout_ = positive("goal_timeout", 120.0);
    input_timeout_ = positive("input_timeout", 5.0);
    freshness_ = positive("input_freshness", 1.0);
    step_ = positive("local_goal_distance", 3.0);
    default_speed_ = positive("default_speed", 2.0);
    z_min_ = declare_parameter<double>("z_min", 0.5);
    z_max_ = declare_parameter<double>("z_max", 2.5);
    max_replans_ = declare_parameter<int>("max_replans", 3);
    if (!std::isfinite(z_min_) || !std::isfinite(z_max_) || z_min_ >= z_max_ ||
        max_replans_ < 0 || max_replans_ > 20 || step_ < 0.05 || step_ > 20)
      throw std::invalid_argument("invalid navigation limits");
    astar_client_ = rclcpp_action::create_client<Nav>(this, backend);
    ego_client_ = rclcpp_action::create_client<Follow>(this,
      declare_parameter<std::string>("follow_action", "/follow_trajectory"));
    session_client_ = create_client<Session>(
      declare_parameter<std::string>("session_service", "/planning/navigation_session"));
    state_sub_ = create_subscription<easyfly_msgs::msg::UAVState>("/uav_state", 10,
      [this](easyfly_msgs::msg::UAVState::ConstSharedPtr msg) {state_ = msg; state_received_ = Clock::now();});
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      declare_parameter<std::string>("grid_topic", "/occupancy_grid"), 1,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {map_ = msg; map_received_ = Clock::now();});
    fault_sub_ = create_subscription<easyfly_msgs::msg::NavigationSession>(
      "/planning/navigation_fault", rclcpp::QoS(1).reliable().transient_local(),
      [this](easyfly_msgs::msg::NavigationSession::ConstSharedPtr msg) {
        if (ctx_ && !ctx_->finished && !msg->active && !ctx_->session.empty() &&
            msg->session_id == ctx_->session) drain(ctx_, "planner_emergency_stop");
      });
    server_ = rclcpp_action::create_server<Nav>(this, "/navigate",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Nav::Goal> goal) {
        bool available = strategy_ == "astar" ? astar_client_->action_server_is_ready() :
          ego_client_->action_server_is_ready() && session_client_->service_is_ready();
        bool expected = false;
        if (!navigation_enabled_ || !validGoal(*goal) || !available || !reserved_.compare_exchange_strong(expected, true))
          return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<Parent> parent) {
        if (!ctx_ || ctx_->parent != parent || ctx_->finished) return rclcpp_action::CancelResponse::REJECT;
        ctx_->user_cancel = true; drain(ctx_, "navigation canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
      }, [this](std::shared_ptr<Parent> parent) {
        ctx_ = std::make_shared<Context>(); ctx_->parent = parent;
        ownership_.own(goalId(ctx_));
        if (strategy_ == "astar") sendAstar(ctx_);
      });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      declare_parameter<std::string>("stop_service", "/navigate/_stop"),
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr reply) {
        reply->success = ctx_ && !ctx_->finished;
        reply->message = reply->success ? "stop requested, awaiting terminal" : "idle or fault locked";
        if (reply->success) {ctx_->supervisor = true; drain(ctx_, "supervisor_stop");}
      });
    owned_stop_service_ = create_service<easyfly_msgs::srv::StopNavigation>("/navigate/_stop_owned",
      [this](easyfly_msgs::srv::StopNavigation::Request::SharedPtr request,
             easyfly_msgs::srv::StopNavigation::Response::SharedPtr reply) {
        reply->success = ctx_ && !ctx_->finished && !request->navigation_goal_id.empty() &&
          request->navigation_goal_id == goalId(ctx_);
        reply->message = reply->success ? "stop requested, awaiting terminal" : "navigation goal no longer active";
        if (reply->success) {ctx_->supervisor = true; drain(ctx_, "supervisor_stop");}
      });
    ownership_service_ = create_service<easyfly_msgs::srv::GetActionOwnership>("/navigation/ownership",
      [this](easyfly_msgs::srv::GetActionOwnership::Request::SharedPtr request,
             easyfly_msgs::srv::GetActionOwnership::Response::SharedPtr response) {
        response->goal_id = request->goal_id; response->state = ownership_.state(request->goal_id);
      });
    goal_id_pub_ = create_publisher<std_msgs::msg::String>("/navigation/goal_id", 1);
    rviz_client_ = rclcpp_action::create_client<Nav>(this, "/navigate");
    rviz_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id != "odom" || !stampFresh(msg->header.stamp) || reserved_) {
          RCLCPP_WARN(get_logger(), "RViz goal rejected: frame, stamp or navigation busy"); return;
        }
        Nav::Goal goal;
        goal.target_x = msg->pose.position.x; goal.target_y = msg->pose.position.y;
        goal.target_z = msg->pose.position.z == 0.0 ? std::numeric_limits<double>::quiet_NaN() : msg->pose.position.z;
        goal.max_speed = default_speed_;
        if (!validGoal(goal) || !rviz_client_->action_server_is_ready()) return;
        rclcpp_action::Client<Nav>::SendGoalOptions callbacks;
        callbacks.goal_response_callback = [this](AChild::SharedPtr child) {
          if (!child) RCLCPP_WARN(get_logger(), "RViz navigation was not admitted");
        };
        callbacks.result_callback = [this](const AChild::WrappedResult & result) {
          RCLCPP_INFO(get_logger(), "RViz navigation result %d: %s", static_cast<int>(result.code),
            result.result ? result.result->message.c_str() : "no result");
        };
        try {rviz_client_->async_send_goal(goal, callbacks);}
        catch (const std::exception & e) {RCLCPP_WARN(get_logger(), "RViz send: %s", e.what());}
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }
private:
  bool navigation_enabled_{false};
  static std::string goalId(const std::shared_ptr<Context> & c) {
    std::ostringstream id;
    for (auto byte : c->parent->get_goal_id())
      id << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
    return id.str();
  }
  double positive(const std::string & key, double fallback) {
    double value = declare_parameter<double>(key, fallback);
    if (!std::isfinite(value) || value <= 0 || value > 3600) throw std::invalid_argument(key);
    return value;
  }
  static double elapsed(Clock::time_point time) {return std::chrono::duration<double>(Clock::now() - time).count();}
  bool validGoal(const Nav::Goal & goal) const {
    return std::isfinite(goal.target_x) && std::isfinite(goal.target_y) &&
      std::abs(goal.target_x) <= 1000 && std::abs(goal.target_y) <= 1000 &&
      !std::isinf(goal.target_z) && (std::isnan(goal.target_z) || (goal.target_z >= z_min_ && goal.target_z <= z_max_)) &&
      std::isfinite(goal.max_speed);
  }
  bool stampFresh(const builtin_interfaces::msg::Time & stamp) const {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
        (stamp.sec == 0 && stamp.nanosec == 0)) return false;
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age >= -0.2 && age <= freshness_;
  }
  bool stateReady() const {
    return state_ && state_->header.frame_id == "odom" && state_->connected && state_->pose_valid &&
      state_->armed && state_->flight_mode == 3 && stampFresh(state_->header.stamp) &&
      elapsed(state_received_) <= freshness_ && std::isfinite(state_->position_x) &&
      std::isfinite(state_->position_y) && std::isfinite(state_->position_z);
  }
  bool hoverReady() const {
    return stateReady() && state_->velocity_valid && std::isfinite(state_->velocity_x) &&
      std::isfinite(state_->velocity_y) && std::isfinite(state_->velocity_z) &&
      std::hypot(std::hypot(state_->velocity_x, state_->velocity_y), state_->velocity_z) <= 0.3;
  }
  bool mapReady() const {
    if (!map_ || !occupancyGridReady(*map_) || !stampFresh(map_->header.stamp) ||
        elapsed(map_received_) > freshness_ || map_->data.size() > 1000000) return false;
    if (map_->info.resolution < 0.01 || map_->info.resolution > 10 ||
        std::abs(map_->info.origin.position.x) > 10000 ||
        std::abs(map_->info.origin.position.y) > 10000) return false;
    const auto & q = map_->info.origin.orientation;
    // Existing A* supports only an axis-aligned odom grid.
    return std::isfinite(q.w) && std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 &&
      std::abs(q.z) < 1e-6 && std::abs(std::abs(q.w) - 1) < 1e-6;
  }
  OccupancyGrid2d grid() const {
    OccupancyGrid2d g;
    g.width = static_cast<int>(map_->info.width); g.height = static_cast<int>(map_->info.height);
    g.resolution = map_->info.resolution; g.origin_x = map_->info.origin.position.x;
    g.origin_y = map_->info.origin.position.y; g.data = map_->data;
    for (auto & value : g.data) if (value < 0 || value > 100) value = 100;
    return g; // Unknown is not a free corridor in combined mode.
  }
  bool plan(const std::shared_ptr<Context> & c) {
    const auto goal = c->parent->get_goal();
    c->z = std::isnan(goal->target_z) ? state_->position_z : goal->target_z;
    c->speed = goal->max_speed > 0 ? goal->max_speed : default_speed_;
    if (c->z < z_min_ || c->z > z_max_) {drain(c, "target altitude outside navigation limits"); return false;}
    c->route.clear(); c->segment = 0;
    if (strategy_ == "ego") {c->route.emplace_back(goal->target_x, goal->target_y); return true;}
    gp::Request request;
    request.grid = {map_->info.width, map_->info.height, map_->info.resolution,
      map_->info.origin.position.x, map_->info.origin.position.y,
      map_->data.data(), map_->data.size(), 50, gp::UnknownPolicy::occupied};
    request.start = {state_->position_x, state_->position_y};
    request.goal = {goal->target_x, goal->target_y};
    request.map.source_stamp_ns = static_cast<std::int64_t>(map_->header.stamp.sec) *
      1000000000 + map_->header.stamp.nanosec;
    gp::SearchControl control;
    control.expansion_limit = 1000000;
    // This coordinator's callbacks are mutually exclusive. Bound synchronous
    // planning by the remaining parent budget and the input freshness window.
    control.deadline = std::min(c->created + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(goal_timeout_)), Clock::now() +
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(freshness_)));
    const auto path = combined_planner_->plan(request, control);
    if (path.status != gp::Status::ok) {drain(c, "global route unavailable"); return false;}
    // planPath preserves the requested endpoint before collision/LOS pruning;
    // never replace a checked final segment with an unchecked endpoint here.
    for (size_t i = 1; i < path.waypoints.size(); ++i) {
      const auto from = path.waypoints[i - 1], to = path.waypoints[i];
      const double distance = std::hypot(to.x - from.x, to.y - from.y);
      const int count = std::max(1, static_cast<int>(std::ceil(distance / step_)));
      for (int j = 1; j <= count; ++j) {
        const double t = static_cast<double>(j) / count;
        c->route.emplace_back(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t);
        if (c->route.size() > 256) {drain(c, "global route exceeds 256 local goals"); return false;}
      }
    }
    return true;
  }
  void finish(const std::shared_ptr<Context> & c, bool success, const std::string & reason, bool force_abort = false) {
    if (c->finished) return;
    auto result = std::make_shared<Nav::Result>(); result->success = success; result->message = reason;
    if (success) c->parent->succeed(result);
    else if (!force_abort && !c->supervisor && c->user_cancel && c->parent->is_canceling()) c->parent->canceled(result);
    else c->parent->abort(result);
    c->finished = true;
  }
  void drain(const std::shared_ptr<Context> & c, const std::string & reason, bool replan = false) {
    if (!c->draining) {c->draining = true; c->drain = Clock::now();}
    c->reason = reason; c->replan = replan && !c->user_cancel && !c->supervisor;
  }
  void release(const std::shared_ptr<Context> & c) {
    if (ctx_ == c) {ownership_.release(goalId(c)); ctx_.reset(); reserved_ = false;}
  }
  void sendAstar(const std::shared_ptr<Context> & c) {
    c->sent = true; c->terminal = false; c->phase = Clock::now();
    rclcpp_action::Client<Nav>::SendGoalOptions cb;
    cb.goal_response_callback = [this, c](AChild::SharedPtr child) {
      c->astar = child;
      if (!child) {c->terminal = true; drain(c, "astar rejected goal");}
    };
    cb.feedback_callback = [c](AChild::SharedPtr, std::shared_ptr<const Nav::Feedback> feedback) {
      if (!c->finished) c->parent->publish_feedback(std::make_shared<Nav::Feedback>(*feedback));
    };
    cb.result_callback = [this, c](const AChild::WrappedResult & result) {
      c->terminal = true;
      c->child_success = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->success;
      if (!c->draining) {
        c->normal_completion = elapsed(c->created) < goal_timeout_;
        drain(c, c->normal_completion ? (result.result ? result.result->message : "astar missing result") :
          "navigation time budget exhausted");
      }
    };
    try {astar_client_->async_send_goal(*c->parent->get_goal(), cb);}
    catch (const std::exception & e) {drain(c, std::string("astar send exception: ") + e.what()); finish(c, false, c->reason, true);}
  }
  void sendFollow(const std::shared_ptr<Context> & c) {
    c->sent = true; c->terminal = false; c->phase = Clock::now();
    c->session = goalId(c) + ":" + std::to_string(++c->sequence);
    Follow::Goal goal;
    goal.target_x = c->route[c->segment].first; goal.target_y = c->route[c->segment].second;
    goal.target_z = c->z; goal.acceptance_radius = 0.1; goal.timeout = goal_timeout_ - elapsed(c->created);
    goal.navigation_session_id = c->session;
    rclcpp_action::Client<Follow>::SendGoalOptions cb;
    const int sequence = c->sequence;
    cb.goal_response_callback = [this, c, sequence](EChild::SharedPtr child) {
      if (c->sequence != sequence) return;
      c->ego = child;
      if (!child) {c->terminal = true; drain(c, "follow trajectory rejected goal");}
      else if (!c->draining) startSession(c);
    };
    cb.feedback_callback = [c, sequence](EChild::SharedPtr, std::shared_ptr<const Follow::Feedback> feedback) {
      if (c->finished || c->sequence != sequence) return;
      auto out = std::make_shared<Nav::Feedback>();
      out->current_segment = c->segment + 1; out->total_segments = c->route.size();
      out->distance_remaining = feedback->distance_remaining; c->parent->publish_feedback(out);
    };
    cb.result_callback = [this, c, sequence](const EChild::WrappedResult & result) {
      if (c->sequence != sequence) return;
      c->terminal = true;
      c->child_success = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->success;
      if (!c->draining) {
        c->normal_completion = elapsed(c->created) < goal_timeout_;
        drain(c, !c->normal_completion ? "navigation time budget exhausted" : c->child_success ? "segment reached" :
          (result.result ? result.result->message : "follow trajectory missing result"));
      }
    };
    try {ego_client_->async_send_goal(goal, cb);}
    catch (const std::exception & e) {drain(c, std::string("follow send exception: ") + e.what()); finish(c, false, c->reason, true);}
  }
  void startSession(const std::shared_ptr<Context> & c) {
    c->start_sent = true; c->phase = Clock::now();
    auto request = std::make_shared<Session::Request>();
    request->session_id = c->session; request->active = true;
    request->target_x = c->route[c->segment].first; request->target_y = c->route[c->segment].second;
    request->target_z = c->z; request->max_speed = c->speed;
    try {session_client_->async_send_request(request, [this, c](rclcpp::Client<Session>::SharedFuture future) {
      c->start_done = true;
      try {if (!future.get()->success) drain(c, "planner rejected session: " + future.get()->message);}
      catch (const std::exception & e) {drain(c, std::string("planner response exception: ") + e.what());}
    });} catch (const std::exception & e) {
      drain(c, std::string("planner start exception: ") + e.what()); finish(c, false, c->reason, true);
    }
  }
  void stopSession(const std::shared_ptr<Context> & c) {
    if (!c->start_sent) {c->stopped = true; return;}
    if (!c->start_done || c->stop_sent || !session_client_->service_is_ready()) return;
    c->stop_sent = true;
    auto request = std::make_shared<Session::Request>(); request->session_id = c->session; request->active = false;
    try {session_client_->async_send_request(request, [this, c](rclcpp::Client<Session>::SharedFuture future) {
      try {c->stopped = future.get()->success;
        if (!c->stopped) {c->replan = false; finish(c, false, "planner session stop rejected: " + future.get()->message, true);}
      } catch (const std::exception & e) {c->replan = false; finish(c, false, std::string("planner stop response: ") + e.what(), true);}
    });} catch (const std::exception & e) {c->replan = false; finish(c, false, std::string("planner stop exception: ") + e.what(), true);}
  }
  void cancelChild(const std::shared_ptr<Context> & c) {
    if (c->terminal || c->cancel_sent || (!c->astar && !c->ego)) return;
    c->cancel_sent = true;
    const int sequence = c->sequence;
    auto cb = [this, c, sequence](auto response) {
      if (c->sequence != sequence) return;
      if (!c->terminal && response->return_code != 0) {c->replan = false; finish(c, false, "child cancellation rejected", true);}
    };
    try {if (c->astar) astar_client_->async_cancel_goal(c->astar, cb);
      else ego_client_->async_cancel_goal(c->ego, cb);
    } catch (const std::exception & e) {c->replan = false; finish(c, false, std::string("child cancel exception: ") + e.what(), true);}
  }
  void nextSegment(const std::shared_ptr<Context> & c) {
    c->ego.reset(); c->sent = false; c->terminal = true; c->cancel_sent = false;
    c->start_sent = c->start_done = c->stop_sent = c->stopped = false;
    c->draining = false; c->child_success = false; c->normal_completion = false; c->phase = Clock::now();
  }
  void tick() {
    auto c = ctx_;
    std_msgs::msg::String identity;
    if (c && !c->finished) identity.data = goalId(c);
    goal_id_pub_->publish(identity);
    if (!c) return;
    if (!c->finished && !c->draining && elapsed(c->created) >= goal_timeout_) drain(c, "navigation time budget exhausted");
    if (strategy_ != "astar" && c->sent && !c->draining &&
        (!stateReady() || (strategy_ == "combined" && !mapReady()))) drain(c, "navigation input invalid or stale");
    if (!c->draining && !c->sent) {
      if (!hoverReady() || (strategy_ == "combined" && !mapReady())) {
        if (elapsed(c->phase) >= input_timeout_) drain(c, "navigation input timeout");
      } else if (!c->route.empty() || plan(c)) sendFollow(c);
    }
    if (!c->draining && c->sent && elapsed(c->phase) >= response_timeout_ &&
        ((!c->astar && !c->ego && !c->terminal) || (c->start_sent && !c->start_done))) drain(c, "child or planner response timeout");
    if (!c->draining && strategy_ == "combined" && c->start_done) {
      auto g = grid(); auto target = c->route[c->segment];
      if (g.worldOccupied(target.first, target.second) ||
          !easyfly::astar::detail::lineOfSight(g, state_->position_x, state_->position_y, target.first, target.second)) {
        if (c->replans++ < max_replans_) drain(c, "global corridor blocked, replanning", true);
        else drain(c, "global replan budget exhausted");
      }
    }
    if (!c->draining) return;
    cancelChild(c); stopSession(c);
    if ((!c->terminal || !c->stopped) && elapsed(c->drain) >= cancel_timeout_) {
      c->replan = false;
      finish(c, false, c->supervisor ? "supervisor_stop" : "child terminal or session stop timeout", true);
    }
    if (!c->terminal || !c->stopped) return; // Retain fault latch until ownership is actually resolved.
    if (c->finished) {release(c); return;}
    if (c->replan) {nextSegment(c); c->route.clear(); c->segment = 0; c->replan = false; return;}
    const bool success = c->normal_completion && c->child_success && !c->user_cancel && !c->supervisor &&
      (strategy_ == "astar" || c->reason == "segment reached");
    if (success && strategy_ != "astar" && ++c->segment < c->route.size()) {nextSegment(c); return;}
    finish(c, success, c->supervisor ? "supervisor_stop" : c->reason); release(c);
  }
  std::unique_ptr<gp::GridPlanner> combined_planner_;
  std::string strategy_;
  std::atomic<bool> reserved_{false};
  int max_replans_;
  double response_timeout_, cancel_timeout_, goal_timeout_, input_timeout_, freshness_, step_, default_speed_, z_min_, z_max_;
  std::shared_ptr<Context> ctx_;
  easyfly_msgs::msg::UAVState::ConstSharedPtr state_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  Clock::time_point state_received_, map_received_;
  rclcpp_action::Client<Nav>::SharedPtr astar_client_, rviz_client_;
  rclcpp_action::Client<Follow>::SharedPtr ego_client_;
  rclcpp::Client<Session>::SharedPtr session_client_;
  rclcpp_action::Server<Nav>::SharedPtr server_;
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;
  rclcpp::Subscription<easyfly_msgs::msg::NavigationSession>::SharedPtr fault_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<easyfly_msgs::srv::StopNavigation>::SharedPtr owned_stop_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_id_pub_;
  easyfly_core::OwnershipRegistry ownership_;
  rclcpp::Service<easyfly_msgs::srv::GetActionOwnership>::SharedPtr ownership_service_;
};
} // namespace easyfly_global_planner
RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_global_planner::NavigationCoordinatorNode)
