// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#ifndef EASYFLY_TRANSPORT__UPDATE_SIGNAL_HPP_
#define EASYFLY_TRANSPORT__UPDATE_SIGNAL_HPP_
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
namespace easyfly_transport {
// Keep notifications raised during RunUpdate until the following wait observes them.
// Separate from ConnectionManager::updateMutex: callbacks can trigger recursively.
class UpdateSignal {
 public:
  std::uint64_t Generation() {
    std::lock_guard<std::mutex> lock(mutex_);return generation_;
  }
  void Notify() {
    {std::lock_guard<std::mutex> lock(mutex_);++generation_;}
    ready_.notify_all();
  }
  bool Wait(std::uint64_t observed) {
    std::unique_lock<std::mutex> lock(mutex_);
    return ready_.wait_for(lock,std::chrono::milliseconds(100),
      [&]{return generation_!=observed;});
  }
 private:
  std::mutex mutex_;
  std::condition_variable ready_;
  std::uint64_t generation_=0;
};
}
#endif
