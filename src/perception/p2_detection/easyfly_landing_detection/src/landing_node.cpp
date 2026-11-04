// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_landing_detection/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#include "safety.hpp"
#include "timing_trace.hpp"
#include <easyfly_msgs/msg/landing_target.hpp>
#include <easyfly_msgs/msg/tag_observation_array.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>
class LandingNode : public rclcpp::Node
{
public:
  LandingNode()
  : Node("landing_detection")
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;

    id_ = declare_parameter("selected_tag_id", -1, d);
    frame_ = declare_parameter(
      "optical_frame",
      "camera_color_optical_frame",
      d);

    age_ = declare_parameter("max_age_seconds", .5, d);
    trace_ = std::make_unique<ef_timing::Trace>(declare_parameter("timing_trace_path", "", d));

    if (id_ < -1 || frame_.empty() || !std::isfinite(age_) || age_ < .05 || age_ > 5) {
      throw std::invalid_argument("invalid landing configuration");
    }
    pub_ = create_publisher<easyfly_msgs::msg::LandingTarget>("/landing_target", 10);

    sub_ = create_subscription<easyfly_msgs::msg::TagObservationArray>(
      "/tag_observations", rclcpp::QoS(1).best_effort(), [this](
        easyfly_msgs::msg::TagObservationArray::ConstSharedPtr message,
        const rclcpp::MessageInfo & info) {
        if (trace_->enabled()) {
          event_ = {};
          event_.input = true;
          event_.entry_wall_ns = ef_timing::wall_ns();
          event_.entry_ros_ns = now().nanoseconds();
          const auto & native = info.get_rmw_message_info();
          event_.writer_wall_ns = native.source_timestamp;
          event_.receive_wall_ns = native.received_timestamp;
        }
        last_ = message->header;
        receipt_ = ef_safety::steady();

        const auto stamp = ef_safety::stamp(last_);

        if (!good_clock() ||
        !ef_safety::fresh(
          last_,
          now().nanoseconds(), receipt_, age_, frame_) || stamp <= watermark_)
        {
          invalid("source_invalid");
          return;
        }
        watermark_ = stamp;

        if (id_ < 0) {
          invalid("tag_not_selected");
          return;
        }
        if (!message->valid || message->observations.size() > 64) {
          invalid("upstream_invalid");
          return;
        }
        const easyfly_msgs::msg::TagObservation * selected = nullptr;

        for (const auto & observation : message->observations) {
          if (observation.tag_id == id_) {
            if (selected) {
              invalid("duplicate_selected_tag");
              return;
            }
            selected = &observation;
          }
        }
        if (!selected) {
          invalid(
            "selected_tag_missing");
          return;
        }
        const auto & o = *selected;
        const auto & p = o.pose.position;
        const auto & q = o.pose.orientation;

        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

        if (!o.valid || ef_safety::stamp(o.header) != stamp || o.header.frame_id != frame_ ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0 ||
        !std::isfinite(norm) || std::abs(norm - 1) > 1e-4 || !std::isfinite(o.tag_size_m) ||
        o.tag_size_m <= 0 ||
        !std::isfinite(o.reprojection_error_px) || o.reprojection_error_px < 0)
        {
          invalid("tag_pose_invalid");
          return;
        }
        double x = 1 - 2 * (q.y * q.y + q.z * q.z), y = 2 * (q.x * q.y + q.w * q.z);

        if (x * x + y * y < 1e-10) {
          invalid(
            "yaw_unobservable");
          return;
        }
        easyfly_msgs::msg::LandingTarget output;
        output.header = last_;
        output.tag_id = id_;
        output.tag_size_m = o.tag_size_m;
        output.pose = o.pose;

        output.optical_depth_m = p.z;
        output.yaw_rad = std::atan2(
          y,
          x);
        output.reprojection_error_px = o.reprojection_error_px;

        output.valid = true;
        output.reason = "selected_tag_optical_pose";
        valid_ = true;

        publish(output);
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this] {
        if (trace_->enabled()) {
          event_ = {};
          event_.entry_wall_ns = ef_timing::wall_ns();
          event_.entry_ros_ns = now().nanoseconds();
        }
        if (!good_clock() || !ef_safety::fresh(
          last_,
          now().nanoseconds(), receipt_, age_, frame_))
        {
          invalid("expired_or_clock_invalid");
        }
      });

    service_ = create_service<easyfly_msgs::srv::GetCapabilities>(
      "/platform/landing/capabilities",
      [this](easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr r) {
        r->header.stamp = now();
        r->names = {"selected_tag_landing_perception"};
        r->implemented = {true};

        r->ready =
        {good_clock() && valid_ &&
          ef_safety::fresh(
            last_,
            now().nanoseconds(), receipt_, age_, frame_)};

        r->details = {"Optical depth is not vertical ground height; "
          "perception only, no descent control."};
      });
  }

private:
  void publish(const easyfly_msgs::msg::LandingTarget & output)
  {
    if (!trace_->enabled()) {
      pub_->publish(output);
      return;
    }
    event_.source_ns = ef_safety::stamp(output.header);
    event_.valid = output.valid;
    event_.publish_begin_wall_ns = ef_timing::wall_ns();
    pub_->publish(output);
    event_.publish_end_wall_ns = ef_timing::wall_ns();
    trace_->record(event_, output.reason);
  }
  bool good_clock()
  {
    const auto t = now().nanoseconds();
    if (t < clock_.last) {
      watermark_ = 0;
      valid_ = false;
    }
    return clock_.check(t, age_);
  }
  void invalid(const std::string & reason)
  {
    easyfly_msgs::msg::LandingTarget output;
    output.header = last_;
    output.tag_id = id_;

    output.valid = false;
    output.reason = reason;
    valid_ = false;
    publish(output);
  }
  std::unique_ptr<ef_timing::Trace> trace_;
  ef_timing::Event event_;
  int id_;
  std::string frame_;
  double age_, receipt_{0};
  int64_t watermark_{0};
  bool valid_{false};

  ef_safety::ClockGate clock_;
  std_msgs::msg::Header last_;

  rclcpp::Publisher<easyfly_msgs::msg::LandingTarget>::SharedPtr pub_;

  rclcpp::Subscription<easyfly_msgs::msg::TagObservationArray>::SharedPtr sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LandingNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
