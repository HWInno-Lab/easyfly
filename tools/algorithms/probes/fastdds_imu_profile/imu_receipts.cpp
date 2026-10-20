// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Read-only native observer. Middleware events and executor callbacks are separate.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <time.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
int64_t Now(clockid_t clock)
{
  timespec value{};
  clock_gettime(clock, &value);
  return static_cast<int64_t>(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

struct Receipt
{
  int stream;
  int64_t stamp, mono, real, source, received;
  uint64_t sequence;
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid;
  std::array<double, 6> values;
};

struct Event
{
  int stream;
  int64_t mono;
  size_t count;
};

struct Trace
{
  std::mutex mutex;
  std::vector<Receipt> rows;
  std::vector<Event> events;
  size_t maximum, dropped_rows = 0, dropped_events = 0;
  int64_t pause_start = 0, pause_end = 0;

  explicit Trace(size_t capacity)
  : maximum(capacity)
  {
    rows.reserve(capacity);
    events.reserve(capacity);
  }

  void Notify(int stream, size_t count)
  {
    const auto stamp = Now(CLOCK_MONOTONIC);
    std::lock_guard<std::mutex> lock(mutex);
    if (events.size() < maximum) {
      events.push_back({stream, stamp, count});
    } else {
      ++dropped_events;
    }
  }

  void Write(const std::string & output)
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::ofstream file(output);
    file << "# CLOCK_MONOTONIC capacity=" << maximum << " discarded=" << dropped_rows <<
      " pause_start_ns=" << pause_start << " pause_end_ns=" << pause_end << '\n';
    file << "stream,stamp_ns,mono_ns,real_ns,rmw_source_ns,rmw_received_ns,sequence,gid," <<
      "ax,ay,az,wx,wy,wz\n";
    file << std::setprecision(17);
    for (const auto & row : rows) {
      file << row.stream << ',' << row.stamp << ',' << row.mono << ',' << row.real << ',' <<
        row.source << ',' << row.received << ',' << row.sequence << ',';
      for (auto byte : row.gid) {
        file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
      }
      file << std::dec;
      for (auto value : row.values) {
        file << ',' << value;
      }
      file << '\n';
    }
    file.close();
    if (!file) {
      throw std::runtime_error("cannot write native receipts");
    }
    std::ofstream notifications(output + ".events.csv");
    notifications << "# CLOCK_MONOTONIC capacity=" << maximum << " discarded=" <<
      dropped_events << '\n';
    notifications << "stream,mono_ns,count\n";
    for (const auto & event : events) {
      notifications << event.stream << ',' << event.mono << ',' << event.count << '\n';
    }
    notifications.close();
    if (!notifications) {
      throw std::runtime_error("cannot write middleware notifications");
    }
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("openvins_native_imu_observer");
  const auto output = node->declare_parameter<std::string>("output", "");
  const auto duration = node->declare_parameter<double>("duration", 120.0);
  const auto capacity = node->declare_parameter<int>("max_samples", 200000);
  const auto pause_after = node->declare_parameter<int>("pause_after", 0);
  const auto pause_ms = node->declare_parameter<int>("pause_ms", 0);
  const auto notifications = node->declare_parameter<bool>("notifications", true);
  const auto camera_load = node->declare_parameter<bool>("camera_load", false);
  if (!std::filesystem::path(output).is_absolute() ||
    std::filesystem::exists(output) || std::filesystem::exists(output + ".events.csv") ||
    std::filesystem::exists(output + ".ready") || !std::isfinite(duration) ||
    duration < 1 || duration > 600 || capacity < 100 || capacity > 1000000 ||
    pause_after < 0 || pause_ms < 0 || pause_ms > 2000)
  {
    std::cerr << "Use a fresh absolute output path and bounded duration/capacity/pause" <<
      std::endl;
    rclcpp::shutdown();
    return 2;
  }
  auto trace = std::make_shared<Trace>(capacity);
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> subscriptions;
  size_t raw_count = 0;
  bool paused = false;
  for (int key = 0; key < 3; ++key) {
    auto qos = rclcpp::QoS(1000).reliable();
    if (key == 1) {
      qos.best_effort();
    }
    const auto topic = key == 2 ? "/openvins/input/imu" : "/livox/imu";
    auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
      topic, qos,
      [trace, key, &raw_count, &paused, pause_after, pause_ms](
        sensor_msgs::msg::Imu::ConstSharedPtr message, const rclcpp::MessageInfo & info)
      {
        Receipt row{};
        row.stream = key;
        row.mono = Now(CLOCK_MONOTONIC);
        row.real = Now(CLOCK_REALTIME);
        row.stamp = static_cast<int64_t>(message->header.stamp.sec) * 1000000000LL +
        message->header.stamp.nanosec;
        const auto & rmw = info.get_rmw_message_info();
        row.source = rmw.source_timestamp;
        row.received = rmw.received_timestamp;
        row.sequence = rmw.publication_sequence_number;
        std::copy(
          std::begin(rmw.publisher_gid.data), std::end(rmw.publisher_gid.data),
          row.gid.begin());
        row.values = {message->linear_acceleration.x, message->linear_acceleration.y,
          message->linear_acceleration.z, message->angular_velocity.x,
          message->angular_velocity.y, message->angular_velocity.z};
        {
          std::lock_guard<std::mutex> lock(trace->mutex);
          if (trace->rows.size() < trace->maximum) {
            trace->rows.push_back(row);
          } else {
            ++trace->dropped_rows;
          }
        }
        if (key == 0 && ++raw_count >= static_cast<size_t>(pause_after) &&
        pause_after > 0 && pause_ms > 0 && !paused)
        {
          paused = true;
          trace->pause_start = Now(CLOCK_MONOTONIC);
          std::this_thread::sleep_for(std::chrono::milliseconds(pause_ms));
          trace->pause_end = Now(CLOCK_MONOTONIC);
        }
      });
    if (notifications) {
      sub->set_on_new_message_callback([trace, key](size_t count) {trace->Notify(key, count);});
    }
    subscriptions.push_back(sub);
  }
  size_t image_count = 0, info_count = 0;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
  if (camera_load) {
    image_sub = node->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", rclcpp::QoS(32).reliable(),
      [&image_count](sensor_msgs::msg::Image::ConstSharedPtr) {++image_count;});
    info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", rclcpp::QoS(32).reliable(),
      [&info_count](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {++info_count;});
  }
  const auto deadline = Now(CLOCK_MONOTONIC) + static_cast<int64_t>(duration * 1e9);
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(50), [&executor, deadline]() {
      if (Now(CLOCK_MONOTONIC) >= deadline) {
        executor.cancel();
      }
    });
  executor.add_node(node);
  {
    std::ofstream ready(output + ".ready");
    ready << "subscriptions ready; 0=raw reliable, 1=raw best effort, 2=guarded reliable\n";
    if (!ready) {
      throw std::runtime_error("cannot write readiness marker");
    }
  }
  executor.spin();
  for (const auto & sub : subscriptions) {
    sub->clear_on_new_message_callback();
  }
  subscriptions.clear();
  image_sub.reset();
  info_sub.reset();
  executor.remove_node(node);
  timer.reset();
  node.reset();
  rclcpp::shutdown();
  trace->Write(output);
  std::cout << "recorded=" << trace->rows.size() << " notifications=" << trace->events.size() <<
    " discarded=" << trace->dropped_rows << "/" << trace->dropped_events <<
    " images=" << image_count << " infos=" << info_count << std::endl;
  return 0;
}
