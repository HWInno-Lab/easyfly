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

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include "import_worker.hpp"
namespace
{
template<typename P> bool until(P predicate)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) {return true;} std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return predicate();
}
struct Gate
{
  std::mutex mutex; std::condition_variable wake; bool open{false};
  void release() {std::lock_guard<std::mutex> lock(mutex); open = true; wake.notify_all();}
};
struct Release {Gate & gate; ~Release() {gate.release();}};
}  // namespace
TEST(ImportWorker, BlockedDiskWorkDoesNotBlockSubmitAndQueueIsBounded)
{
  Gate gate; std::atomic<int> calls{0}, replies{0};
  ImportWorker worker([&](const auto &) {
      ++calls; std::unique_lock<std::mutex> lock(gate.mutex);
      gate.wake.wait(lock, [&] {return gate.open;}); return ImportWorker::Api::Response{};
    });
  Release release{gate};
  auto job = std::make_shared<ImportWorker::Api::Request>();
  auto reply = [&](const auto &) {++replies;};
  ASSERT_TRUE(worker.submit(job, reply)); ASSERT_TRUE(until([&] {return calls == 1;}));
  for (int i = 0; i < 8; ++i) {
    EXPECT_TRUE(worker.submit(job, reply));
  }
  EXPECT_FALSE(worker.submit(job, reply)); EXPECT_EQ(calls, 1);
  auto oversized = std::make_shared<ImportWorker::Api::Request>();
  oversized->content = std::string(65537, 'x');
  EXPECT_FALSE(worker.submit(oversized, reply));
  gate.release(); ASSERT_TRUE(until([&] {return replies == 9;})); EXPECT_EQ(calls, 9);
}
