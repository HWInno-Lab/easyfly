// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_time_sync/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#include <chrono>
#include "easyfly_time_sync/pairing.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <easyfly_msgs/msg/time_sync_status.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>

namespace easyfly_time_sync
{
class TimeSyncNode : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using Clock = std::chrono::steady_clock;

public:
  TimeSyncNode()
  : Node("time_sync")
  {
    rcl_interfaces::msg::ParameterDescriptor frozen; frozen.read_only = true;
    Options options;
    options.tolerance = declare_parameter("tolerance", .03, frozen);
    options.max_age = declare_parameter("max_age", .5, frozen);
    const auto count = declare_parameter("queue_size", 8, frozen);
    const auto budget = declare_parameter("max_queued_bytes", 67108864, frozen);
    if (count < 1 || budget < 1) {throw std::invalid_argument("invalid queue limits");}
    options.queue_size = count; options.max_bytes = budget;
    pairer_ = std::make_unique<Pairer<Image>>(options);
    image_budget_ = declare_parameter("max_image_bytes", 4194304, frozen);
    if (image_budget_ < 1 || image_budget_ > 16777216) {
      throw std::invalid_argument("invalid image budget");
    }
    const auto first = declare_parameter<std::string>(
      "first_topic", "/camera/color/image_raw",
      frozen);
    const auto second = declare_parameter<std::string>(
      "second_topic",
      "/camera/aligned_depth_to_color/depth/image_raw",
      frozen);
    const auto output_first = declare_parameter<std::string>(
      "first_output",
      "/time_sync/color/image_raw", frozen);
    const auto output_second = declare_parameter<std::string>(
      "second_output",
      "/time_sync/depth/image_raw", frozen);
    if (first.empty() || second.empty() || first == second || output_first.empty() ||
      output_second.empty() ||
      output_first == output_second || output_first == first || output_first == second ||
      output_second == first || output_second == second)
    {
      throw std::invalid_argument("pairing topics must be distinct and nonempty");
    }
    outputs_[0] = create_publisher<Image>(output_first, rclcpp::SensorDataQoS());
    outputs_[1] = create_publisher<Image>(output_second, rclcpp::SensorDataQoS());
    status_ = create_publisher<easyfly_msgs::msg::TimeSyncStatus>("/time_sync/status", 10);
    subscriptions_[0] = create_subscription<Image>(
      first, rclcpp::SensorDataQoS(),
      [this](Image::ConstSharedPtr image) {receive(0, image);});
    subscriptions_[1] = create_subscription<Image>(
      second, rclcpp::SensorDataQoS(),
      [this](Image::ConstSharedPtr image) {receive(1, image);});
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this] {
        pairer_->expire(now().nanoseconds(), wall()); publishStatus();
      });
    capabilities_ = create_service<easyfly_msgs::srv::GetCapabilities>(
      "/platform/time_sync/capabilities",
      [this](easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr response) {
        pairer_->expire(
          now().nanoseconds(),
          wall()); response->header.stamp = now();
        response->names = {"software_image_pairing", "hardware_clock_sync"};
        response->implemented = {true, false};
        response->ready = {pairer_->valid(now().nanoseconds(), wall()), false};
        response->details = {pairer_->reason(), "unsupported: source stamps are never rewritten"};
      });
  }

private:
  static double wall()
  {
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
  }
  void receive(size_t side, const Image::ConstSharedPtr & image)
  {
    pairer_->expire(now().nanoseconds(), wall());
    const auto stamp = image->header.stamp;
    // Pairing transports image bytes; no pixel interpretation/conversion is implied.
    if (image->header.frame_id.empty() || image->header.frame_id.size() > 128 || stamp.sec < 0 ||
      stamp.nanosec >= 1000000000u ||
      !image->width || !image->height || image->width > 8192 || image->height > 8192 ||
      !image->step || image->encoding.empty() ||
      image->data.size() > static_cast<size_t>(image_budget_) ||
      uint64_t(image->step) * image->height != image->data.size())
    {
      pairer_->reject(side, "invalid_image_header_or_layout"); publishStatus(); return;
    }
    // Function-argument evaluation order must not make receipt appear AFTER check time.
    const auto received = wall();
    const auto reference = now().nanoseconds();
    const auto pairs = pairer_->add(
      side, {int64_t(stamp.sec) * 1000000000 + stamp.nanosec,
        received, image->data.size(), image}, reference, received);
    for (const auto & pair : pairs) {
      outputs_[0]->publish(*pair.first.payload); outputs_[1]->publish(*pair.second.payload);
      last_headers_[0] = pair.first.payload->header; last_headers_[1] = pair.second.payload->header;
    }
    publishStatus();
  }
  void publishStatus()
  {
    easyfly_msgs::msg::TimeSyncStatus message; message.header.stamp = now();
    message.first_header = last_headers_[0]; message.second_header = last_headers_[1];
    message.delta_seconds =
      ((int64_t(message.second_header.stamp.sec) - message.first_header.stamp.sec) * 1000000000 +
      int64_t(message.second_header.stamp.nanosec) - message.first_header.stamp.nanosec) / 1e9;
    message.pair_valid = pairer_->valid(now().nanoseconds(), wall());
    message.reason = pairer_->reason();
    message.paired_count = pairer_->paired(); message.dropped_first = pairer_->dropped(0);
    message.dropped_second = pairer_->dropped(1);
    message.queued_first = pairer_->queued(0); message.queued_second = pairer_->queued(1);
    status_->publish(message);
  }
  int image_budget_;
  std::unique_ptr<Pairer<Image>> pairer_;
  std::array<std_msgs::msg::Header, 2> last_headers_;
  std::array<rclcpp::Publisher<Image>::SharedPtr, 2> outputs_;
  std::array<rclcpp::Subscription<Image>::SharedPtr, 2> subscriptions_;
  rclcpp::Publisher<easyfly_msgs::msg::TimeSyncStatus>::SharedPtr status_;
  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr capabilities_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace easyfly_time_sync
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<easyfly_time_sync::TimeSyncNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "time_sync: %s\n", error.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
