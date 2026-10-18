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

#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include "easyfly_motion_prediction/predictor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <easyfly_msgs/msg/target3_d_array.hpp>
#include <easyfly_msgs/msg/target_prediction_array.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>

namespace easyfly_target_prediction
{
using easyfly::motion_prediction::Options;
using easyfly::motion_prediction::Observation;
using easyfly::motion_prediction::Frame;
using easyfly::motion_prediction::Prediction;
using easyfly::motion_prediction::Predictor;

class TargetPredictionNode : public rclcpp::Node
{
public:
  TargetPredictionNode()
  : Node("target_prediction")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only = true;
    Options o;
    o.horizon = declare_parameter("horizon_seconds", .5, descriptor);
    o.max_age = declare_parameter("max_age_seconds", .5, descriptor);
    o.max_speed = declare_parameter("max_speed", 10., descriptor);
    o.max_jump = declare_parameter("max_jump", 5., descriptor);
    o.min_confidence = declare_parameter("min_confidence", .2, descriptor);
    o.base_variance = declare_parameter("base_variance", .04, descriptor);
    o.variance_growth = declare_parameter("variance_growth", 1., descriptor);
    const auto tracks = declare_parameter("max_tracks", 256, descriptor);
    if (tracks < 1 || tracks > 1024) {throw std::invalid_argument("max_tracks must be 1..1024");}
    o.max_tracks = static_cast<size_t>(tracks);
    const double period = declare_parameter("publish_period_seconds", .1, descriptor);
    if (!std::isfinite(period) || period < .05 || period > 1) {
      throw std::invalid_argument("invalid publish period");
    }
    predictor_ = std::make_unique<Predictor>(o);
    publisher_ = create_publisher<easyfly_msgs::msg::TargetPredictionArray>(
      "/target_predictions",
      10);
    subscriber_ = create_subscription<easyfly_msgs::msg::Target3DArray>(
      "/target_positions", rclcpp::SensorDataQoS(),
      [this](easyfly_msgs::msg::Target3DArray::ConstSharedPtr message) {
        Frame frame;frame.frame = message->header.frame_id;
        frame.stamp_ns = stamp(message->header.stamp);
        frame.well_formed = frame.stamp_ns > 0;
        // Bound conversion before allocating a per-target model input vector.
        if (message->targets.size() > max_tracks_) {frame.well_formed = false;} else {
          for (const auto & target : message->targets) {
            Observation o;o.id = target.track_id;o.stamp_ns = stamp(
              target.header.stamp);
            o.frame = target.header.frame_id;o.class_name = target.class_name;
            o.confidence = target.confidence;
            o.position_valid = target.position_valid;
            o.position = {target.position_x, target.position_y, target.position_z};
            frame.observations.push_back(o);
          }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        predictor_->observe(frame, now().nanoseconds(), steady());
        publish();  // Empty/disappeared identities invalidate immediately, not at next timer.
      });
    max_tracks_ = o.max_tracks;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period), [this] {
        std::lock_guard<std::mutex> lock(mutex_);publish();
      });
    capabilities_ = create_service<easyfly_msgs::srv::GetCapabilities>(
      "/platform/prediction/capabilities",
      [this](const easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr response) {
        response->header.stamp = now();response->header.frame_id = "odom";
        response->names = {"constant_velocity_prediction"};response->implemented = {true};
        std::lock_guard<std::mutex> lock(mutex_);
        const auto predictions = predictor_->predict(now().nanoseconds(), steady());
        response->ready = {std::any_of(
            predictions.begin(), predictions.end(), [](
              const auto & p) {
              return p.valid;
            })};
        response->details = {"Independent odom model only; readiness is not flight permission. "
          "Variance is model-based, not calibrated sensor covariance."};
      });
  }

private:
  static int64_t stamp(const builtin_interfaces::msg::Time & t)
  {
    if (t.sec < 0 || t.nanosec >= 1000000000u) {return 0;}
    return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
  }
  static int64_t steady()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void publish()
  {
    easyfly_msgs::msg::TargetPredictionArray output;output.header.stamp = now();
    output.header.frame_id = "odom";
    for (const auto & p :
      predictor_->predict(rclcpp::Time(output.header.stamp).nanoseconds(), steady()))
    {
      easyfly_msgs::msg::TargetPrediction message;message.header = output.header;
      message.observation_header.frame_id = p.observation.frame;
      message.observation_header.stamp = rclcpp::Time(p.observation.stamp_ns);
      message.track_id = p.observation.id;message.class_name = p.observation.class_name;
      message.confidence = p.observation.confidence;
      message.horizon_seconds = p.horizon;message.observation_age_seconds = p.age;
      message.position_x = p.position[0];message.position_y = p.position[1];
      message.position_z = p.position[2];
      message.velocity_x = p.velocity[0];message.velocity_y = p.velocity[1];
      message.velocity_z = p.velocity[2];
      message.variance_x = message.variance_y = message.variance_z = p.variance;
      message.valid = p.valid;message.reason = p.reason;output.predictions.push_back(message);
    }
    publisher_->publish(output);
  }
  size_t max_tracks_{256};
  std::mutex mutex_;
  std::unique_ptr<Predictor> predictor_;
  rclcpp::Publisher<easyfly_msgs::msg::TargetPredictionArray>::SharedPtr publisher_;
  rclcpp::Subscription<easyfly_msgs::msg::Target3DArray>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr capabilities_;
};
}  // namespace easyfly_target_prediction
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<easyfly_target_prediction::TargetPredictionNode>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "target prediction: %s\n", e.what());rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
