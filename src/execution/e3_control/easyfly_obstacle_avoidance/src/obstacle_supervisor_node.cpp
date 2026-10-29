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
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <easyfly_msgs/msg/obstacle_info_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <easyfly_msgs/msg/uav_state.hpp>
#include <easyfly_msgs/srv/get_obstacles.hpp>
#include <easyfly_msgs/srv/stop_navigation.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
// A supervisor, not a flight controller: it publishes no setpoints.
class ObstacleSupervisor : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;
  using Stop = easyfly_msgs::srv::StopNavigation;
  using Trigger = std_srvs::srv::Trigger;
  struct Pending
  {
    std::shared_ptr<rclcpp::Service<Trigger>> service;
    std::shared_ptr<rmw_request_id_t> header;
    Clock::time_point created{Clock::now()};
    int64_t request_id{0};
    bool replied{false};
  };

public:
  ObstacleSupervisor()
  : Node("obstacle_supervisor")
  {
    freshness_ = positive("freshness", 1.0);
    stop_distance_ = positive("stop_distance", 0.5);
    radius_ = positive("uav_radius", 0.3);
    auto_stop_ = declare_parameter<bool>("auto_stop", false);
    obstacles_sub_ = create_subscription<easyfly_msgs::msg::ObstacleInfoArray>(
      "/obstacles", 10,
      [this](easyfly_msgs::msg::ObstacleInfoArray::ConstSharedPtr msg) {
        obstacles_ = msg; obstacle_received_ = Clock::now();
      });
    state_sub_ = create_subscription<easyfly_msgs::msg::UAVState>(
      "/uav_state", 10,
      [this](easyfly_msgs::msg::UAVState::ConstSharedPtr msg) {
        state_ = msg; state_received_ = Clock::now();
      });
    goal_sub_ = create_subscription<std_msgs::msg::String>(
      "/navigation/goal_id", 1,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        goal_id_ = msg->data; goal_received_ = Clock::now();
      });
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/avoidance/constraints_valid", 1);
    near_pub_ = create_publisher<std_msgs::msg::Bool>("/avoidance/near_obstacle", 1);
    query_ = create_service<easyfly_msgs::srv::GetObstacles>(
      "/avoidance/get_near_obstacles",
      [this](easyfly_msgs::srv::GetObstacles::Request::SharedPtr req,
      easyfly_msgs::srv::GetObstacles::Response::SharedPtr reply) {
        reply->obstacles.header.frame_id = "odom";  // Zero stamp is explicitly invalid.
        if (!std::isfinite(req->range_limit) || !valid()) {return;}
        reply->obstacles.header = obstacles_->header;
        for (const auto & obstacle : obstacles_->obstacles) {
          if (req->range_limit <= 0 || clearance(obstacle) <= req->range_limit) {
            reply->obstacles.obstacles.push_back(obstacle);
          }
        }
      });
    stop_client_ = create_client<Stop>("/navigate/_stop_owned");
    stop_ = create_service<Trigger>(
      "/avoidance/stop_navigation",
      [this](std::shared_ptr<rclcpp::Service<Trigger>> service,
      std::shared_ptr<rmw_request_id_t> header, Trigger::Request::SharedPtr) {
        auto pending = std::make_shared<Pending>();
        pending->service = service;
        pending->header = header;
        if (pending_) {reply(pending, false, "stop request already pending"); return;}
        requestStop(pending);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {
        const bool ready = valid();
        bool near = false;
        if (ready) {
          for (const auto & obstacle : obstacles_->obstacles) {
            if (clearance(obstacle) <= stop_distance_) {near = true; break;}}
        }
        std_msgs::msg::Bool message; message.data = ready; valid_pub_->publish(message);
        message.data = near; near_pub_->publish(message);
        if (pending_ && elapsed(pending_->created) > 2.0) {
          reply(pending_, false, "navigation stop response timeout");
          stop_client_->remove_pending_request(pending_->request_id);
          next_retry_ = Clock::now() + std::chrono::seconds(1);
          // The keyed late request cannot stop a different goal. Release reply storage.
          pending_.reset();
        }
        if (auto_stop_ && !pending_ && Clock::now() >= next_retry_ &&
        (!ready || near) && currentGoal() && last_stopped_ != goal_id_)
        {
          requestStop(std::make_shared<Pending>());
        }
      });
  }

private:
  static double elapsed(Clock::time_point time)
  {
    return std::chrono::duration<double>(Clock::now() - time).count();
  }
  double positive(const std::string & key, double value)
  {
    value = declare_parameter<double>(key, value);
    if (!std::isfinite(value) || value <= 0 || value > 60) {throw std::invalid_argument(key);}
    return value;
  }
  bool stampFresh(const builtin_interfaces::msg::Time & time) const
  {
    if (time.sec < 0 || time.nanosec >= 1000000000u ||
      (time.sec == 0 && time.nanosec == 0)) {return false;}
    const auto age = (now() - rclcpp::Time(time, get_clock()->get_clock_type())).seconds();
    return age >= -0.2 && age <= freshness_;
  }
  bool currentGoal() const {return goal_id_.size() == 32 && elapsed(goal_received_) <= freshness_;}
  bool valid() const
  {
    if (!state_ || !state_->connected || !state_->pose_valid || state_->header.frame_id != "odom" ||
      !stampFresh(state_->header.stamp) || elapsed(state_received_) > freshness_ ||
      !std::isfinite(state_->position_x) || !std::isfinite(state_->position_y) ||
      !std::isfinite(state_->position_z) ||
      !obstacles_ || obstacles_->header.frame_id != "odom" ||
      !stampFresh(obstacles_->header.stamp) ||
      elapsed(obstacle_received_) > freshness_ || obstacles_->obstacles.size() > 10000)
    {
      return false;
    }
    for (const auto & o : obstacles_->obstacles) {
      if (o.header.frame_id != "odom" || !stampFresh(o.header.stamp) ||
        !std::isfinite(o.center_x) || !std::isfinite(o.center_y) || !std::isfinite(o.center_z) ||
        !std::isfinite(o.size_x) || !std::isfinite(o.size_y) || !std::isfinite(o.size_z) ||
        o.size_x < 0 || o.size_y < 0 || o.size_z < 0 || !std::isfinite(o.velocity_x) ||
        !std::isfinite(o.velocity_y) || !std::isfinite(o.velocity_z)) {return false;}
    }
    return true;
  }
  double clearance(const easyfly_msgs::msg::ObstacleInfo & o) const
  {
    const double x = std::max(0.0, std::abs(state_->position_x - o.center_x) - o.size_x / 2);
    const double y = std::max(0.0, std::abs(state_->position_y - o.center_y) - o.size_y / 2);
    const double z = std::max(0.0, std::abs(state_->position_z - o.center_z) - o.size_z / 2);
    return std::max(0.0, std::sqrt(x * x + y * y + z * z) - radius_);
  }
  void reply(const std::shared_ptr<Pending> & p, bool success, const std::string & message)
  {
    if (p->replied) {return;}
    p->replied = true;
    if (!p->service) {return;}
    Trigger::Response response; response.success = success; response.message = message;
    try {p->service->send_response(*p->header, response);} catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "stop reply: %s", e.what());
    }
  }
  void requestStop(const std::shared_ptr<Pending> & p)
  {
    if (!currentGoal() || !stop_client_->service_is_ready()) {
      reply(p, false, "no active navigation or stop service"); return;
    }
    pending_ = p;
    auto request = std::make_shared<Stop::Request>(); request->navigation_goal_id = goal_id_;
    try {
      auto future_and_id = stop_client_->async_send_request(
        request, [this, p, request](rclcpp::Client<Stop>::SharedFuture future) {
          try {
            if (future.get()->success) {last_stopped_ = request->navigation_goal_id;}
            reply(p, future.get()->success, future.get()->message);
          } catch (const std::exception & e) {reply(p, false, e.what());}
          next_retry_ = Clock::now() + std::chrono::seconds(1);
          if (pending_ == p) {pending_.reset();}
        }); p->request_id = future_and_id.request_id;
    } catch (const std::exception & e) {
      reply(p, false, e.what()); pending_.reset();
      next_retry_ = Clock::now() + std::chrono::seconds(1);
    }
  }
  double freshness_, stop_distance_, radius_; bool auto_stop_;
  std::string goal_id_, last_stopped_;
  Clock::time_point goal_received_, state_received_, obstacle_received_, next_retry_;
  std::shared_ptr<Pending> pending_;
  easyfly_msgs::msg::UAVState::ConstSharedPtr state_;
  easyfly_msgs::msg::ObstacleInfoArray::ConstSharedPtr obstacles_;
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp::Subscription<easyfly_msgs::msg::ObstacleInfoArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_, near_pub_;
  rclcpp::Service<easyfly_msgs::srv::GetObstacles>::SharedPtr query_;
  rclcpp::Service<Trigger>::SharedPtr stop_;
  rclcpp::Client<Stop>::SharedPtr stop_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleSupervisor>());
  rclcpp::shutdown();
}
