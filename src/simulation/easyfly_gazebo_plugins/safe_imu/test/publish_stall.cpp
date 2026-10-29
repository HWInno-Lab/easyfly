// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Test-only interposer: delay exactly one real /livox/imu rcl_publish call.
#include <rcl/publisher.h>
#include <sensor_msgs/msg/imu.hpp>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <string>
#include <thread>
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t *publisher,
  const void *message, rmw_publisher_allocation_t *allocation)
{
  using Publish = rcl_ret_t (*)(const rcl_publisher_t *, const void *,
    rmw_publisher_allocation_t *);
  static auto real = reinterpret_cast<Publish>(dlsym(RTLD_NEXT, "rcl_publish"));
  static std::atomic<unsigned> calls{0};
  const char *topic = rcl_publisher_get_topic_name(publisher);
  const char *prefix = std::getenv("EASYFLY_TEST_IMU_STALL_PREFIX");
  if (prefix && topic && std::strcmp(topic, "/livox/imu") == 0 && ++calls == 200) {
    const auto *imu = static_cast<const sensor_msgs::msg::Imu *>(message);
    const auto stamp = static_cast<long long>(imu->header.stamp.sec) * 1000000000LL +
      imu->header.stamp.nanosec;
    const int delay = std::atoi(std::getenv("EASYFLY_TEST_IMU_STALL_MS"));
    if (delay < 1 || delay > 15000) {std::abort();}
    auto now = [] {return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();};
    std::ofstream(std::string(prefix) + ".begin") << stamp << ' ' << now() << '\n';
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    std::ofstream(std::string(prefix) + ".end") << stamp << ' ' << now() << '\n';
  }
  if (!real) {std::abort();}
  return real(publisher, message, allocation);
}
