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

#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <thread>
#include <utility>
#include "import_worker.hpp"
struct ImportWorker::Impl
{
  struct Job {Api::Request::SharedPtr request; std::function<void(Api::Response)> reply;};
  Handler handler;
  std::mutex mutex;
  std::condition_variable wake;
  std::deque<Job> queue;
  bool closing{false};
  std::thread thread;
  explicit Impl(Handler h)
  : handler(std::move(h)), thread([this] {run();}) {}
  ~Impl()
  {
    {std::lock_guard<std::mutex> lock(mutex); closing = true; queue.clear(); wake.notify_all();}
    if (thread.joinable()) {thread.join();}
  }
  void run()
  {
    std::unique_lock<std::mutex> lock(mutex);
    while (!closing) {
      wake.wait(lock, [this] {return closing || !queue.empty();});
      if (closing) {return;}
      auto job = std::move(queue.front()); queue.pop_front(); lock.unlock();
      try {job.reply(handler(*job.request));} catch (const std::exception &) {
        // Missing reply remains uncertain. Never repeat the handler or compensate its writes.
      }
      lock.lock();
    }
  }
};
ImportWorker::ImportWorker(Handler h)
: impl_(std::make_unique<Impl>(std::move(h))) {}
ImportWorker::~ImportWorker() = default;
bool ImportWorker::submit(Api::Request::SharedPtr request, std::function<void(Api::Response)> reply)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->closing || impl_->queue.size() >= 8 || !request || request->content.size() > 65536) {
    return false;
  }
  impl_->queue.push_back({std::move(request), std::move(reply)}); impl_->wake.notify_all();
  return true;
}
