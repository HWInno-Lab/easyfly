#include <easyfly_core/client.hpp>
#include <easyfly_core/ownership_registry.hpp>
#include <easyfly_msgs/srv/get_action_ownership.hpp>
#include <easyfly_msgs/action/search_area.hpp>
#include <easyfly_msgs/msg/target3_d_array.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "easyfly_coverage_planning/search_geometry.hpp"
#include <chrono>
#include <set>

namespace easyfly_mission_fsm {
using easyfly::coverage::SearchPoint;
using easyfly::coverage::SearchBounds;
using easyfly::coverage::searchRoute;
using Clock = std::chrono::steady_clock;
using Search = easyfly_msgs::action::SearchArea;
using Handle = rclcpp_action::ServerGoalHandle<Search>;
class SearchNode : public rclcpp::Node {
  struct Run {
    std::shared_ptr<Handle> parent;
    std::vector<SearchPoint> route;
    size_t index{0};
    double area{0}, total_area{0};
    std::set<int32_t> targets;
    std::shared_ptr<easyfly_core::Operation> child;
    Clock::time_point started{Clock::now()}, segment{Clock::now()}, stopping;
    std::string stop_reason;
    bool user_cancel{false}, published_terminal{false};
  };
 public:
  SearchNode() : Node("search_area") {
    rcl_interfaces::msg::ParameterDescriptor frozen; frozen.read_only = true;
    auto parameter = [&](const char * name, double value, double low, double high) {
      auto v = declare_parameter<double>(name, value, frozen);
      if (!std::isfinite(v) || v < low || v > high) throw std::invalid_argument(name);
      return v;
    };
    spacing_ = parameter("strip_spacing", 2, .1, 20);
    segment_budget_ = parameter("segment_timeout", 60, .05, 3600);
    total_budget_ = parameter("total_timeout", 300, .05, 7200);
    cancel_budget_ = parameter("cancel_timeout", 5, .05, 30);
    bounds_.min_x = parameter("min_x", -50, -1000, 1000);
    bounds_.max_x = parameter("max_x", 50, -1000, 1000);
    bounds_.min_y = parameter("min_y", -50, -1000, 1000);
    bounds_.max_y = parameter("max_y", 50, -1000, 1000);
    bounds_.min_z = parameter("min_z", .5, 0, 50);
    bounds_.max_z = parameter("max_z", 2.5, 0, 50);
    if (bounds_.min_x >= bounds_.max_x || bounds_.min_y >= bounds_.max_y || bounds_.min_z > bounds_.max_z)
      throw std::invalid_argument("invalid search flight bounds");
    easyfly_core::ClientOptions options;
    options.use_sim_time = get_parameter("use_sim_time").as_bool();
    options.max_pending_operations = 1;
    options.require_managed_ownership = true;
    sdk_ = std::make_unique<easyfly_core::Client>(options);
    server_ = rclcpp_action::create_server<Search>(this, "/search_area",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Search::Goal> goal) {
        if (active_ || !validState() || !sdk_->server_ready("navigate")) return rclcpp_action::GoalResponse::REJECT;
        try {route(*goal);} catch (const std::exception & exc) {
          RCLCPP_WARN(get_logger(), "search rejected: %s", exc.what());
          return rclcpp_action::GoalResponse::REJECT;
        }
        active_ = std::make_shared<Run>();
        active_->route = route(*goal); active_->total_area = goal->area_width * goal->area_height;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<Handle> handle) {
        if (!active_ || active_->parent != handle || active_->published_terminal) return rclcpp_action::CancelResponse::REJECT;
        stop("user_cancel", true); return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<Handle> handle) {
        active_->parent = handle; ownership_.own(easyfly_core::goalIdentity(handle->get_goal_id()));
      });
    ownership_service_ = create_service<easyfly_msgs::srv::GetActionOwnership>("/search/ownership",
      [this](easyfly_msgs::srv::GetActionOwnership::Request::SharedPtr request,
             easyfly_msgs::srv::GetActionOwnership::Response::SharedPtr response) {
        response->goal_id = request->goal_id; response->state = ownership_.state(request->goal_id);
      });
    targets_ = create_subscription<easyfly_msgs::msg::Target3DArray>("/target_positions", 10,
      [this](easyfly_msgs::msg::Target3DArray::ConstSharedPtr message) {observe(*message);});
    capabilities_ = create_service<easyfly_msgs::srv::GetCapabilities>("/platform/search/capabilities",
      [this](easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
        easyfly_msgs::srv::GetCapabilities::Response::SharedPtr response) {
        response->header.stamp = now(); response->header.frame_id = "odom";
        response->names = {"boustrophedon", "spiral", "grid"}; response->implemented = {true, false, false};
        response->ready = {!active_ && validState() && sdk_->server_ready("navigate"), false, false};
        response->details = {"completed-strip geometric estimate, not sensor coverage", "unsupported", "unsupported"};
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {tick();});
  }
  ~SearchNode() override {if (active_ && active_->child) active_->child->cancel(); sdk_->close(0);}
 private:
  bool validState() const {
    const auto snapshot = sdk_->state(); const auto & s = snapshot.value;
    return snapshot.fresh && s.connected && s.pose_valid && s.velocity_valid && s.armed &&
      s.flight_mode != 6 && s.flight_mode != 7 && std::isfinite(s.position_x) &&
      std::isfinite(s.position_y) && std::isfinite(s.position_z) &&
      std::isfinite(s.velocity_x) && std::isfinite(s.velocity_y) && std::isfinite(s.velocity_z);
  }
  std::vector<SearchPoint> route(const Search::Goal & goal) const {
    if (goal.search_pattern != 1) throw std::invalid_argument("unsupported search pattern; use 1=boustrophedon");
    if (!std::isfinite(goal.search_speed) || goal.search_speed <= 0 || goal.search_speed > 20)
      throw std::invalid_argument("search_speed outside (0,20]");
    return searchRoute(goal.center_x, goal.center_y, goal.area_width, goal.area_height,
      goal.search_altitude, spacing_, bounds_);
  }
  bool fresh(const std_msgs::msg::Header & header) const {
    if (header.frame_id != "odom" || header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000u ||
      !(header.stamp.sec || header.stamp.nanosec)) return false;
    const auto age = (now() - rclcpp::Time(header.stamp, get_clock()->get_clock_type())).seconds();
    return age >= -.2 && age <= 1;
  }
  void observe(const easyfly_msgs::msg::Target3DArray & message) {
    if (!active_ || active_->published_terminal || !active_->stop_reason.empty() ||
      !fresh(message.header) || message.targets.size() > 10000) return;
    const auto goal = active_->parent ? active_->parent->get_goal() : nullptr;
    if (!goal) return;
    for (const auto & t : message.targets) {
      if (t.track_id < 0 || !t.position_valid || !fresh(t.header) || !std::isfinite(t.position_x) ||
        !std::isfinite(t.position_y) || !std::isfinite(t.position_z) || !std::isfinite(t.confidence) ||
        t.confidence < 0 || t.confidence > 1 ||
        std::abs(t.position_x - goal->center_x) > goal->area_width / 2 ||
        std::abs(t.position_y - goal->center_y) > goal->area_height / 2) continue;
      if (!active_->targets.count(t.track_id) && active_->targets.size() == 4096) {
        stop("target_identity_capacity_exceeded"); return;
      }
      active_->targets.insert(t.track_id);
    }
  }
  void stop(const std::string & reason, bool user=false) {
    if (!active_ || !active_->stop_reason.empty()) return;
    active_->stop_reason = reason; active_->user_cancel = user; active_->stopping = Clock::now();
    if (active_->child) active_->child->cancel();
  }
  void terminal(bool success) {
    if (active_->published_terminal) return;
    auto feedback = std::make_shared<Search::Feedback>();
    feedback->area_covered = active_->area; feedback->progress = active_->area / active_->total_area;
    feedback->targets_found = static_cast<int32_t>(active_->targets.size());
    active_->parent->publish_feedback(feedback);
    auto result = std::make_shared<Search::Result>(); result->success = success;
    result->total_targets_found = static_cast<int32_t>(active_->targets.size());
    result->message = success ? "geometric coverage complete; sensor coverage not certified" : active_->stop_reason;
    if (success) active_->parent->succeed(result);
    else if (active_->user_cancel && active_->parent->is_canceling()) active_->parent->canceled(result);
    else active_->parent->abort(result);
    active_->published_terminal = true;
  }
  void release() {
    ownership_.release(easyfly_core::goalIdentity(active_->parent->get_goal_id())); active_.reset();
  }
  void tick() {
    if (!active_ || !active_->parent) return;
    try {
      auto & run = *active_;
      auto pending = [&] {return run.child && run.child->snapshot().remote_execution_pending;};
      if (run.published_terminal) {if (!pending()) release(); return;}
      const auto time = Clock::now();
      if (run.stop_reason.empty() && !validState()) stop("state_invalid_or_takeover");
      if (run.stop_reason.empty() && time - run.started >= std::chrono::duration<double>(total_budget_)) stop("search_timeout");
      if (!run.stop_reason.empty()) {
        if (!pending() || time - run.stopping >= std::chrono::duration<double>(cancel_budget_)) {
          if (pending()) run.stop_reason += ":child_terminal_unconfirmed";
          terminal(false); if (!pending()) release();
        }
        return;
      }
      if (run.child) {
        auto snapshot = run.child->snapshot();
        if (time - run.segment >= std::chrono::duration<double>(segment_budget_)) stop("navigation_segment_timeout");
        else if (snapshot.action_terminal_received && snapshot.status != easyfly_core::Status::Succeeded)
          stop("navigation_failed:" + snapshot.message);
        else if (snapshot.local_status != easyfly_core::LocalStatus::Active) stop("navigation_transport_or_admission_fault");
        else if (snapshot.remote_execution_pending) {
          if (time - run.segment >= std::chrono::duration<double>(segment_budget_)) stop("navigation_segment_timeout");
        } else if (snapshot.status != easyfly_core::Status::Succeeded) stop("navigation_failed:" + snapshot.message);
        else {
          const auto state = sdk_->state().value; const auto point = run.route.at(run.index);
          const auto error = std::hypot(std::hypot(state.position_x-point.x, state.position_y-point.y), state.position_z-point.z);
          const auto speed = std::hypot(std::hypot(state.velocity_x, state.velocity_y), state.velocity_z);
          if (error > .15 || speed > .3) {
            if (time - run.segment >= std::chrono::duration<double>(segment_budget_)) stop("navigation_arrival_not_confirmed");
          } else {run.area = point.completed_area; ++run.index; run.child.reset();}
        }
      }
      if (run.stop_reason.empty() && !run.child) {
        if (run.index == run.route.size()) {terminal(true); release(); return;}
        const auto point = run.route[run.index];
        easyfly_msgs::action::NavigateTo::Goal goal;
        goal.target_x = point.x; goal.target_y = point.y; goal.target_z = point.z;
        goal.max_speed = run.parent->get_goal()->search_speed;
        run.segment = Clock::now(); run.child = sdk_->navigate(goal);
      }
      auto feedback = std::make_shared<Search::Feedback>();
      feedback->area_covered = run.area; feedback->progress = run.area / run.total_area;
      feedback->targets_found = static_cast<int32_t>(run.targets.size());
      run.parent->publish_feedback(feedback);
    } catch (const std::exception & exc) {stop(std::string("search_internal_error:") + exc.what());}
  }
  SearchBounds bounds_;
  double spacing_, segment_budget_, total_budget_, cancel_budget_;
  std::unique_ptr<easyfly_core::Client> sdk_;
  std::shared_ptr<Run> active_;
  rclcpp_action::Server<Search>::SharedPtr server_;
  rclcpp::Subscription<easyfly_msgs::msg::Target3DArray>::SharedPtr targets_;
  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr capabilities_;
  rclcpp::TimerBase::SharedPtr timer_;
  easyfly_core::OwnershipRegistry ownership_;
  rclcpp::Service<easyfly_msgs::srv::GetActionOwnership>::SharedPtr ownership_service_;
};
}  // namespace easyfly_mission_fsm
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<easyfly_mission_fsm::SearchNode>());
  rclcpp::shutdown(); return 0;
}
