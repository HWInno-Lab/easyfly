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

#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
volatile std::sig_atomic_t stopping = 0;
void stop(int) {stopping = 1;}
using Steady = std::chrono::steady_clock;
int64_t stamp(const builtin_interfaces::msg::Time & value)
{
  return static_cast<int64_t>(value.sec) * 1000000000LL + value.nanosec;
}
struct Counter
{
  uint64_t received{0}, forwarded{0}, rejected{0};
  int64_t last_stamp{0};
};

class Boundary
{
public:
  Boundary(
    const std::string & name, int common_domain, int local_domain,
    const std::string & output)
  : name_(name), frame_(name + "/"), output_(output),
    common_context_(std::make_shared<rclcpp::Context>()),
    local_context_(std::make_shared<rclcpp::Context>())
  {
    rclcpp::InitOptions common_options, local_options;
    common_options.set_domain_id(static_cast<size_t>(common_domain));
    local_options.set_domain_id(static_cast<size_t>(local_domain));
    common_context_->init(0, nullptr, common_options);
    local_context_->init(0, nullptr, local_options);
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).context(common_context_);
    common_ = std::make_shared<rclcpp::Node>("sensor_boundary", "/fleet/" + name, options);
    options.context(local_context_);
    local_ = std::make_shared<rclcpp::Node>("fleet_sensor_boundary", options);
    clock_pub_ = local_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::SensorDataQoS().keep_last(100));
    clock_sub_ = common_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::SensorDataQoS().keep_last(100),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
        const auto value = stamp(message->clock);
        if (value < clock_) {clock_fault_ = true;}
        if (value != clock_) {
          clock_ = value;
          clock_at_ = Steady::now();
          clock_pub_->publish(*message);
          ++clocks_;
        }
      });
    relay<sensor_msgs::msg::Image>("/camera/color/image_raw", 5);
    relay<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", 5);
    relay<sensor_msgs::msg::Image>("/camera/aligned_depth_to_color/depth/image_raw", 5);
    relay<sensor_msgs::msg::CameraInfo>("/camera/aligned_depth_to_color/depth/camera_info", 5);
    relay<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", 5);
    relay<sensor_msgs::msg::Imu>("/livox/imu", 256);
    status_ = common_->create_publisher<std_msgs::msg::String>(
      "/fleet/" + name + "/sensor_boundary_status", 5);
    timer_ = common_->create_wall_timer(
      std::chrono::milliseconds(200), [this]() {
        std_msgs::msg::String message;
        message.data = statistics();
        status_->publish(message);
      });
  }

  void run()
  {
    rclcpp::ExecutorOptions options;
    options.context = common_context_;
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(common_);
    while (!stopping && common_context_->is_valid() && local_context_->is_valid()) {
      executor.spin_some(std::chrono::milliseconds(3));
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    executor.remove_node(common_);
    std::ofstream stream(output_);
    stream << statistics() << "\n";
    stream.close();
    if (!stream) {throw std::runtime_error("native sensor statistics write failed");}
    timer_.reset();
    subscriptions_.clear();
    clock_sub_.reset();
    clock_pub_.reset();
    status_.reset();
    common_.reset();
    local_.reset();
    common_context_->shutdown("owned fleet sensor boundary retired");
    local_context_->shutdown("owned fleet sensor boundary retired");
  }

private:
  template<class Message>
  static bool bounded(const Message &) {return true;}

  static bool bounded(const sensor_msgs::msg::Image & message)
  {
    return message.data.size() <= 8 * 1024 * 1024;
  }
  static bool bounded(const livox_ros_driver2::msg::CustomMsg & message)
  {
    return message.points.size() <= 100000 && message.point_num == message.points.size();
  }

  template<class Message>
  void relay(const std::string & topic, size_t depth)
  {
    auto qos = rclcpp::SensorDataQoS().keep_last(depth);
    auto publisher = local_->create_publisher<Message>(topic, qos);
    counters_[topic] = Counter{};
    auto subscription = common_->create_subscription<Message>(
      "/fleet/" + name_ + topic, qos,
      [this, publisher, topic](typename Message::SharedPtr message) {
        auto & counter = counters_.at(topic);
        ++counter.received;
        const auto source = stamp(message->header.stamp);
        const double age = static_cast<double>(clock_ - source) * 1e-9;
        const double clock_age = std::chrono::duration<double>(Steady::now() - clock_at_).count();
        if (clock_fault_ || clock_age > 1. || source <= counter.last_stamp || source <= 0 ||
        age < -.2 || age > 1. || message->header.stamp.nanosec >= 1000000000U ||
        message->header.frame_id.compare(0, frame_.size(), frame_) != 0 ||
        message->header.frame_id.size() <= frame_.size() ||
        !std::regex_match(message->header.frame_id, frame_pattern_) || !bounded(*message))
        {
          ++counter.rejected;
          return;
        }
        counter.last_stamp = source;
        // The received C++ object is private to this subscription. Preserve all
        // source fields and payload; only remove its exact owning frame prefix.
        message->header.frame_id.erase(0, frame_.size());
        publisher->publish(*message);
        ++counter.forwarded;
      });
    subscriptions_.push_back(subscription);
  }

  std::string statistics() const
  {
    std::ostringstream stream;
    stream << "{\"vehicle\":\"" << name_ << "\",\"clock_forwarded\":" << clocks_
           << ",\"clock_fault\":" << (clock_fault_ ? "true" : "false")
           << ",\"source_ns\":" << clock_ << ",\"sensors\":{";
    bool first = true;
    for (const auto & item : counters_) {
      if (!first) {stream << ',';}
      first = false;
      const auto & value = item.second;
      stream << "\"" << item.first << "\":{\"received\":" << value.received
             << ",\"forwarded\":" << value.forwarded << ",\"rejected\":" << value.rejected << '}';
    }
    stream << "}}";
    return stream.str();
  }

  std::string name_, frame_, output_;
  std::regex frame_pattern_{"[A-Za-z_][A-Za-z0-9_/]{0,127}"};
  rclcpp::Context::SharedPtr common_context_, local_context_;
  rclcpp::Node::SharedPtr common_, local_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, Counter> counters_;
  int64_t clock_{0};
  uint64_t clocks_{0};
  bool clock_fault_{false};
  Steady::time_point clock_at_{};
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr << "usage: fleet_sensor_boundary VEHICLE COMMON_DOMAIN LOCAL_DOMAIN OUTPUT_JSON\n";
    return 2;
  }
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  try {
    const std::string name(argv[1]);
    const int common = std::stoi(argv[2]), local = std::stoi(argv[3]);
    if (!std::regex_match(name, std::regex("[a-z][a-z0-9_]{0,15}")) ||
      common < 0 || common > 232 || local < 0 || local > 232 || common == local)
    {
      throw std::invalid_argument("invalid native sensor boundary identity or domain");
    }
    Boundary boundary(name, common, local, argv[4]);
    boundary.run();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
