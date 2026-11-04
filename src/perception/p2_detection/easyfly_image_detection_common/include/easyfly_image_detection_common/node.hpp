// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_image_detection_common/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#ifndef EASYFLY_IMAGE_DETECTION_COMMON__NODE_HPP_
#define EASYFLY_IMAGE_DETECTION_COMMON__NODE_HPP_

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <easyfly_msgs/srv/get_capabilities.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace easyfly::image_detection
{
// Standalone, single-threaded executor only. DDS depth one bounds the input queue;
// detection runs synchronously, so no stale worker can republish after invalidation.
template<typename Result>
class Node : public rclcpp::Node
{
public:
  Node(
    const std::string & name, const std::string & capability, bool allow_mono,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(name, options), capability_(capability), allow_mono_(allow_mono)
  {
    frame_ = parameter<std::string>("optical_frame", "camera_color_optical_frame");
    age_ = parameter<double>("max_age_seconds", 0.5);
    input_ = parameter<std::string>("input_topic", "/camera/image_raw");
    input_reliability_ = parameter<std::string>("input_reliability", "auto");
    const auto output = parameter<std::string>("output_topic", "/" + capability + "_observations");
    const auto service = parameter<std::string>(
      "capabilities_service", "/platform/" + capability + "/capabilities");
    const auto threads = parameter<int>("opencv_threads", 2);
    if (frame_.empty() || input_.empty() || output.empty() || service.empty() ||
      (input_reliability_ != "auto" && input_reliability_ != "reliable" &&
      input_reliability_ != "best_effort") ||
      !std::isfinite(age_) || age_ < 0.05 || age_ > 5.0 || threads < 1 || threads > 8)
    {
      throw std::invalid_argument("invalid image detector transport parameters");
    }
    cv::setNumThreads(threads);
    publisher_ = this->template create_publisher<Result>(output, rclcpp::QoS(1));
    refresh_input();
    graph_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {refresh_input();});
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(25), [this]() {
        if (!clock_valid() || (last_result_.valid && !fresh(last_result_.header, receipt_))) {
          invalidate("expired_or_clock_invalid");
        }
      });
    service_ = this->template create_service<easyfly_msgs::srv::GetCapabilities>(
      service,
      [this](const easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr response) {
        response->header.stamp = this->now();
        response->names = {capability_};
        response->implemented = {true};
        response->ready = {clock_valid() && last_result_.valid &&
          fresh(last_result_.header, receipt_)};
        response->details = {"Image geometry only; ready includes valid images with no targets."};
      });
  }

protected:
  template<typename T>
  T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return this->template declare_parameter<T>(name, value, descriptor);
  }
  virtual void detect(const cv::Mat & image, Result & output) = 0;

private:
  void refresh_input()
  {
    std::vector<std::string> sources;
    bool all_reliable = true;
    for (const auto & endpoint : this->get_publishers_info_by_topic(input_)) {
      if (endpoint.topic_type() != "sensor_msgs/msg/Image") {continue;}
      const auto & gid = endpoint.endpoint_gid();
      sources.emplace_back(reinterpret_cast<const char *>(gid.data()), gid.size());
      all_reliable = all_reliable && endpoint.qos_profile().reliability() ==
        rclcpp::ReliabilityPolicy::Reliable;
    }
    std::sort(sources.begin(), sources.end());
    const bool reliable = input_reliability_ == "reliable" ||
      (input_reliability_ == "auto" && !sources.empty() && all_reliable);
    if (subscription_ && sources == input_sources_ && reliable == reliable_input_) {return;}
    // Retire the old result before changing transport. In particular, a QoS
    // rebuild must not give the same source frame another receipt lifetime.
    invalidate("input_source_changed");
    const auto generation = ++input_generation_;
    subscription_.reset(); input_sources_ = std::move(sources); reliable_input_ = reliable;
    auto qos = rclcpp::QoS(1).durability_volatile();
    if (reliable) {qos.reliable();} else {qos.best_effort();}
    subscription_ = this->template create_subscription<sensor_msgs::msg::Image>(
      input_, qos,
      [this, generation](sensor_msgs::msg::Image::ConstSharedPtr image,
      const rclcpp::MessageInfo & info) {
        if (generation != input_generation_) {return;}
        const auto & gid = info.get_rmw_message_info().publisher_gid;
        const std::string source(reinterpret_cast<const char *>(gid.data), sizeof(gid.data));
        if (source != receiving_source_) {
          invalidate("input_source_changed"); receiving_source_ = source; watermark_ = 0;
        }
        receive(*image);
      });
    RCLCPP_INFO(
      this->get_logger(), "Image input reliability: %s (%s, %zu publisher(s))",
      reliable ? "reliable" : "best_effort", input_reliability_.c_str(), input_sources_.size());
  }
  static double steady()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static int64_t stamp(const std_msgs::msg::Header & header)
  {
    if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000u) {return 0;}
    return static_cast<int64_t>(header.stamp.sec) * 1000000000LL + header.stamp.nanosec;
  }
  bool clock_valid()
  {
    const auto current = this->now().nanoseconds();
    const double wall = steady();
    if (current < clock_) {
      watermark_ = 0;
      clock_ = current;
      progress_ = wall;
      invalidate("clock_rewind");
      return false;
    }
    if (current != clock_) {progress_ = wall;}
    clock_ = current;
    return current > 0 && wall - progress_ <= age_;
  }
  bool fresh(const std_msgs::msg::Header & header, double receipt) const
  {
    const auto source = stamp(header);
    const double source_age = (this->now().nanoseconds() - source) * 1e-9;
    const double wall_age = steady() - receipt;
    return source > 0 && header.frame_id == frame_ && source_age >= -0.05 &&
           source_age <= age_ && wall_age >= 0 && wall_age <= age_;
  }
  void invalidate(const std::string & reason)
  {
    // Emit once per transition. Preserve original source header, never renew validity.
    if (last_result_.valid || last_result_.reason != reason) {
      last_result_.observations.clear();
      last_result_.valid = false;
      last_result_.reason = reason;
      publisher_->publish(last_result_);
    }
  }
  void receive(const sensor_msgs::msg::Image & image)
  {
    const double receipt = steady();
    // Check the clock before replacing the result header, to invalidate the old epoch.
    const bool good_clock = clock_valid();
    last_result_ = Result();
    last_result_.header = image.header;
    last_result_.image_width = image.width;
    last_result_.image_height = image.height;
    const auto source = stamp(image.header);
    if (!good_clock || !fresh(image.header, receipt) || source <= watermark_) {
      invalidate("invalid_time_or_frame");
      return;
    }
    // Keep the watermark even for a malformed frame; replay cannot extend its lifetime.
    watermark_ = source;
    const int channels = image.encoding == "bgr8" || image.encoding == "rgb8" ? 3 :
      (allow_mono_ && image.encoding == "mono8" ? 1 : 0);
    if (channels == 0 || image.width < 8 || image.height < 8 ||
      uint64_t(image.width) * image.height > 2073600 ||
      image.step < uint64_t(image.width) * channels ||
      uint64_t(image.step) * image.height > 16777216 ||
      image.data.size() != uint64_t(image.step) * image.height)
    {
      invalidate("invalid_image_layout_or_encoding");
      return;
    }
    try {
      // The view remains within the callback-owned message. Algorithms never mutate it.
      cv::Mat view(
        image.height, image.width, channels == 1 ? CV_8UC1 : CV_8UC3,
        const_cast<unsigned char *>(image.data.data()), image.step);
      cv::Mat converted;
      if (image.encoding == "rgb8") {
        cv::cvtColor(view, converted, cv::COLOR_RGB2BGR);
        view = converted;
      }
      detect(view, last_result_);
      if (!clock_valid() || !fresh(image.header, receipt)) {
        invalidate("processing_expired");
        return;
      }
      receipt_ = receipt;
      last_result_.valid = true;
      last_result_.reason = last_result_.observations.empty() ? "no_detection" : "ok";
      publisher_->publish(last_result_);
    } catch (const std::exception & error) {
      RCLCPP_WARN(this->get_logger(), "Detection rejected: %s", error.what());
      invalidate("algorithm_error");
    }
  }

  std::string capability_, frame_, input_, input_reliability_, receiving_source_;
  std::vector<std::string> input_sources_;
  bool reliable_input_{false};
  uint64_t input_generation_{0};
  bool allow_mono_;
  double age_{0.5}, progress_{0.0}, receipt_{0.0};
  int64_t clock_{0}, watermark_{0};
  Result last_result_;
  typename rclcpp::Publisher<Result>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr service_;
};
}  // namespace easyfly::image_detection

#endif  // EASYFLY_IMAGE_DETECTION_COMMON__NODE_HPP_
