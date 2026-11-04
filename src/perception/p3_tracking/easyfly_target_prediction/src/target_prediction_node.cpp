// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_target_prediction/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#include <chrono>
#include <easyfly_perception_contracts/contracts.hpp>
namespace pc = easyfly::perception;
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
    const auto version = declare_parameter("perception_version", 1, descriptor);
    if (version != 1 && version != 2) {throw std::invalid_argument("perception_version must be 1 or 2");}
    v2_ = version == 2;
    legacy_ = declare_parameter("publish_legacy_compatibility", !v2_, descriptor);
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
    options_ = o;
    predictor_ = std::make_unique<Predictor>(o);
    native_pub_ = create_publisher<easyfly_msgs::msg::TargetPredictionArrayV2>("/perception/v2/predictions", 10);
    publisher_ = create_publisher<easyfly_msgs::msg::TargetPredictionArray>(
      "/target_predictions",
      10);
    if (!v2_) {subscriber_ = create_subscription<easyfly_msgs::msg::Target3DArray>(
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
    } else {
      native_input_ = std::make_unique<pc::WorldInput>(*this,
        [this](easyfly_msgs::msg::WorldTargetArrayV2::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          native_ = m; keys_.clear();
          Frame frame; frame.frame = m->context.header.frame_id;
          frame.stamp_ns = pc::stamp(m->context.header.stamp); frame.well_formed = true;
          for (const auto & t : m->targets) {
            Observation o; o.id = t.key.legacy_raw_id; o.stamp_ns = frame.stamp_ns;
            o.frame = frame.frame; o.class_name = t.class_name;
            o.confidence = t.confidence_valid ? t.confidence : 0;
            o.position_valid = t.position_valid; o.position = {t.position_x, t.position_y, t.position_z};
            frame.observations.push_back(o); keys_[o.id] = t;
          }
          predictor_->observe(frame, now().nanoseconds(), steady()); publish();
        }, [this](const std::string & reason) {
          std::lock_guard<std::mutex> lock(mutex_);
          predictor_ = std::make_unique<Predictor>(options_); keys_.clear(); native_.reset();
          producer_.reset(); publish_error_ = reason; publish();
        });
    }
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
    if (legacy_) {publisher_->publish(output);}
    if (v2_) {
      easyfly_msgs::msg::TargetPredictionArrayV2 native;
      native.context = producer_.next(output.header, native_ ? native_->context.camera_id : "front",
        native_ ? native_->context.image_width : 0, native_ ? native_->context.image_height : 0,
        native_ ? pc::Context::VALID : pc::Context::NOT_READY, native_ ? "" : publish_error_);
      if (native_) {native.upstream = pc::reference(native_->context);}
      for (const auto & p : output.predictions) {
        const auto found = keys_.find(p.track_id); if (found == keys_.end()) {continue;}
        const auto & t = found->second;
        easyfly_msgs::msg::TargetPredictionV2 value;
        value.header = p.header; value.observation_header = p.observation_header;
        value.key = t.key; value.observation = t.observation; value.class_id = t.class_id;
        value.class_name = p.class_name; value.confidence_valid = t.confidence_valid; value.confidence = p.confidence;
        value.horizon_seconds = p.horizon_seconds; value.observation_age_seconds = p.observation_age_seconds;
        value.position_x = p.position_x; value.position_y = p.position_y; value.position_z = p.position_z;
        value.velocity_x = p.velocity_x; value.velocity_y = p.velocity_y; value.velocity_z = p.velocity_z;
        value.variance_x = p.variance_x; value.variance_y = p.variance_y; value.variance_z = p.variance_z;
        value.valid = p.valid; value.reason = p.reason; native.predictions.push_back(value);
      }
      native_pub_->publish(native);
    }
  }
  bool v2_{false}, legacy_{true}; Options options_; pc::Producer producer_;
  std::string publish_error_{"waiting_for_native_world"};
  std::unique_ptr<pc::WorldInput> native_input_;
  easyfly_msgs::msg::WorldTargetArrayV2::ConstSharedPtr native_;
  std::map<int64_t, easyfly_msgs::msg::WorldTargetV2> keys_;
  rclcpp::Publisher<easyfly_msgs::msg::TargetPredictionArrayV2>::SharedPtr native_pub_;
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
