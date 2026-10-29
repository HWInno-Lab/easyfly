#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <easyfly_core/client.hpp>
#include <easyfly_core/ownership_registry.hpp>
#include <easyfly_msgs/srv/get_action_ownership.hpp>
#include <easyfly_msgs/action/execute_mission.hpp>
#include <easyfly_msgs/msg/mission_status.hpp>
#include <easyfly_msgs/msg/mission_step_feedback.hpp>
#include <easyfly_msgs/srv/set_flight_mode.hpp>
#include "easyfly_mission_fsm/legacy_mission.hpp"
#include <atomic>
#include <mutex>
#include <thread>
namespace easyfly_mission_fsm {
class MissionFSM : public rclcpp::Node {
 public:
  using ExecuteMission = easyfly_msgs::action::ExecuteMission;
  using Handle = rclcpp_action::ServerGoalHandle<ExecuteMission>;
  explicit MissionFSM(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MissionFSM() override;
 private:
  rclcpp_action::GoalResponse handle_mission_goal(const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteMission::Goal>);
  rclcpp_action::CancelResponse handle_mission_cancel(std::shared_ptr<Handle>);
  void handle_mission_accepted(std::shared_ptr<Handle>);
  void execute_mission(std::vector<MissionStep>, std::shared_ptr<Handle>);
  bool execute_step(const MissionStep &, int, int, std::shared_ptr<Handle>);
  void cancel_current_step();
  void safety_check();
  void trigger_rtl();
  void publish_status(uint8_t, const std::string &, int, int, float,
    bool success = false, const std::string &message = "");
  void feedback(uint8_t, const std::string &, int, int, float, std::shared_ptr<Handle>);
  bool state_ready(const easyfly_core::StateSnapshot &) const;
  void stop(const std::string &reason);
  std::string reason() const;
  std::unique_ptr<easyfly_core::Client> sdk_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr as_mission_;
  rclcpp::Client<easyfly_msgs::srv::SetFlightMode>::SharedPtr cli_set_mode_;
  rclcpp::Publisher<easyfly_msgs::msg::MissionStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<easyfly_msgs::msg::MissionStepFeedback>::SharedPtr pub_step_feedback_;
  rclcpp::TimerBase::SharedPtr timer_safety_;
  std::atomic<bool> mission_active_{false}, cancel_requested_{false}, shutdown_{false};
  std::atomic<bool> rtl_triggered_{false};
  std::atomic<int> steps_completed_{0};
  mutable std::mutex mission_mtx_;
  std::string stop_reason_;
  std::vector<MissionStep> accepted_steps_;
  double cancel_budget_s_{3}, battery_invalid_grace_s_{5}, backend_discovery_s_{3};
  std::chrono::steady_clock::time_point battery_invalid_since_;
  std::mutex step_mtx_;
  std::shared_ptr<easyfly_core::Operation> current_;
  std::string current_type_;
  std::mutex worker_mtx_;
  std::thread worker_;
  easyfly_core::OwnershipRegistry ownership_registry_;
  rclcpp::Service<easyfly_msgs::srv::GetActionOwnership>::SharedPtr ownership_service_;
};
}
