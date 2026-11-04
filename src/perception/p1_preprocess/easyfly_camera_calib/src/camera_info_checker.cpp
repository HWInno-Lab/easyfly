// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_camera_calib/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

// Continuous validation; a once-valid CameraInfo never grants perpetual readiness.
#include "easyfly_camera_calib/camera_contract.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace easyfly_camera_calib
{
class CameraInfoChecker : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;

public:
  explicit CameraInfoChecker(const rclcpp::NodeOptions & options)
  : Node("camera_info_checker", options)
  {
    rcl_interfaces::msg::ParameterDescriptor frozen; frozen.read_only = true;
    auto file = declare_parameter<std::string>("calib_file", "", frozen);
    if (file.empty()) {
      file = ament_index_cpp::get_package_share_directory("easyfly_camera_calib") +
        "/config/sensors.yaml";
    }
    reference_ = loadCalibration(file);
    require(reference_.has_camera, "missing camera calibration");
    tolerance_ = declare_parameter("tolerance", 1.0, frozen); max_age_ = declare_parameter(
      "max_age", 1.0, frozen);
    require(
      std::isfinite(tolerance_) && tolerance_ >= 0 && tolerance_ <= 10 &&
      std::isfinite(max_age_) && max_age_ >= .05 && max_age_ <= 10, "invalid checker thresholds");
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/camera/calibration/status", 10);
    subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      declare_parameter<std::string>(
        "camera_info_topic", "/camera/color/camera_info",
        frozen), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        latest_ = info; received_ = Clock::now(); publish();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {publish();});
  }

private:
  void publish()
  {
    std::string reason = "not_received"; double age = -1;
    const auto ros_time = now().nanoseconds();
    const auto steady_time = Clock::now();
    bool reset = false;
    if (!clock_seen_) {clock_seen_ = true; clock_progress_ = steady_time;}
    if (ros_time < last_clock_) {
      latest_.reset(); clock_progress_ = steady_time; reset = true;
    } else if (ros_time > last_clock_) {clock_progress_ = steady_time;}
    last_clock_ = ros_time;
    if (latest_) {
      reason = cameraMismatch(*latest_, reference_, tolerance_); auto stamp = latest_->header.stamp;
      if (stamp.sec < 0 || stamp.nanosec >= 1000000000u || (stamp.sec == 0 && stamp.nanosec == 0)) {
        reason = "invalid_stamp";
      } else {
        age = (now().nanoseconds() - (int64_t(stamp.sec) * 1000000000 + stamp.nanosec)) / 1e9;
        const bool stale = age > max_age_ ||
          std::chrono::duration<double>(Clock::now() - received_).count() > max_age_;
        if (age < -.2) {
          reason = "future_stamp";
        } else if (stale) {
          reason = "stale_camera_info";
        }
      }
    }
    const bool paused =
      std::chrono::duration<double>(steady_time - clock_progress_).count() > max_age_;
    if (ros_time <= 0) {reason = "clock_not_ready";} else if (reset) {
      reason = "clock_reset";
    } else if (paused) {
      reason = "clock_paused";
    }
    diagnostic_msgs::msg::DiagnosticArray message; message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status; status.name = "easyfly/camera_calibration";
    status.hardware_id = reference_.camera_frame;
    status.level = reason == "valid" ? status.OK : status.ERROR; status.message = reason;
    diagnostic_msgs::msg::KeyValue value; value.key = "source_age_seconds";
    value.value = std::to_string(age);
    status.values.push_back(value); message.status.push_back(status); publisher_->publish(message);
  }
  Calibration reference_;
  double tolerance_, max_age_;
  Clock::time_point received_;
  Clock::time_point clock_progress_;
  int64_t last_clock_{0};
  bool clock_seen_{false};
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace easyfly_camera_calib
RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_camera_calib::CameraInfoChecker)
