// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Derived from Gazebo Classic gazebo11_11.10.2 transport/IOManager.cc.
// The externally visible IOManager ABI and private field layout are preserved.
#include <dlfcn.h>
#include <openssl/evp.h>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <boost/thread/thread.hpp>
#include <gazebo/transport/IOManager.hh>
#include "easyfly_transport/compat.hpp"

extern "C" const char * easyfly_transport_build_tag()
{
  return "EasyFly-Gazebo11.10.2-lifetime-v1";
}

namespace gazebo::transport
{
class IOManagerPrivate
{
public:
  boost::asio::io_service * io_service = nullptr;
  boost::asio::io_service::work * work = nullptr;
  std::atomic_int count{0};
  boost::thread * thread = nullptr;
};
}  // namespace gazebo::transport

namespace
{
using State = gazebo::transport::IOManagerPrivate;
std::atomic_bool validated{false};

void Dispose(State * state)
{
  // Executed only by a different thread after stop(). join() is mandatory.
  if (state->thread) {
    state->thread->join();
    delete state->thread;
  }
  delete state->work;
  delete state->io_service;
  delete state;
}

class Reaper
{
public:
  Reaper()
  : worker_([this] {Run();}) {}

  ~Reaper()
  {
    if (!Finish()) {
      std::cerr << "EasyFly transport: live managers at process teardown\n";
      std::abort();  // Explicit failure, never free a still-running IO service.
    }
  }

  void Created()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      std::cerr << "EasyFly transport: manager created after shutdown\n";
      std::abort();
    }
    ++stats_.live;
  }

  void Destroyed()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    --stats_.live;
    ++stats_.completed;
    condition_.notify_all();
  }

  void Retire(State * state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // A live manager keeps Finish() from closing the worker.
    queue_.push_back(state);
    --stats_.live;
    ++stats_.deferred;
    ++stats_.pending;
    condition_.notify_all();
  }

  void Drain()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] {return stats_.pending == 0;});
  }

  bool Finish()
  {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      // Server owners can be gone while the last WeakBinder callback still
      // owns a Connection. Its destructor has not queued retirement yet:
      // pending == 0 alone is not quiescence. Keep the reaper open until both
      // lifecycle counts reach zero. A retained/stuck owner remains a failure.
      if (!condition_.wait_for(lock, std::chrono::seconds(5), [this] {
          return stats_.live == 0 && stats_.pending == 0;
        })) {
        return false;
      }
      closed_ = true;
      condition_.notify_all();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    return true;
  }

  easyfly_transport::Statistics Statistics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }

private:
  void Run()
  {
    for (;; ) {
      State * state;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {return closed_ || !queue_.empty();});
        if (queue_.empty()) {
          return;
        }
        state = queue_.front();
        queue_.pop_front();
      }
      // Do not hold the queue mutex while waiting for callbacks to return.
      Dispose(state);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        --stats_.pending;
        ++stats_.completed;
        condition_.notify_all();
      }
    }
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<State *> queue_;
  easyfly_transport::Statistics stats_{0, 0, 0, 0};
  bool closed_{false};
  std::thread worker_;
};

Reaper & Cleanup()
{
  static Reaper reaper;
  return reaper;
}
}  // namespace

namespace easyfly_transport
{
bool ValidateLibrary(const std::string & expected_sha)
{
  // Verify Connection really belongs to this complete patched transport image.
  void * transport = dlopen("libgazebo_transport.so.11", RTLD_NOW | RTLD_LOCAL);
  if (!transport) {
    return false;
  }
  Dl_info connection_info{}, patched_info{};
  void * connection = dlsym(transport, "_ZN6gazebo9transport10ConnectionD1Ev");
  bool local = connection && dladdr(connection, &connection_info) &&
    dladdr(reinterpret_cast<void *>(&easyfly_transport_build_tag), &patched_info) &&
    connection_info.dli_fbase == patched_info.dli_fbase;
  dlclose(transport);
  if (!local) {
    return false;
  }
  // ABI guard on the actual loaded system core, not just its version string.
  void * handle = dlopen("libgazebo.so.11", RTLD_NOW | RTLD_LOCAL);
  if (!handle) {
    return false;
  }
  Dl_info info{};
  void * symbol = dlsym(handle, "_ZN6gazebo6ServerD1Ev");
  bool found = symbol && dladdr(symbol, &info) && info.dli_fname;
  std::string path = found ? info.dli_fname : "";
  dlclose(handle);
  std::ifstream input(path, std::ios::binary);
  if (!found || !input) {
    return false;
  }
  std::unique_ptr<EVP_MD_CTX, decltype(& EVP_MD_CTX_free)> digest(
    EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!digest || EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) != 1) {
    return false;
  }
  std::array<char, 8192> buffer{};
  while (input.read(buffer.data(), buffer.size()) || input.gcount() > 0) {
    if (EVP_DigestUpdate(digest.get(), buffer.data(), input.gcount()) != 1) {
      return false;
    }
  }
  if (!input.eof()) {
    return false;
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> bytes{};
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(digest.get(), bytes.data(), &length) != 1) {
    return false;
  }
  std::ostringstream hex;
  for (unsigned int i = 0; i < length; ++i) {
    hex << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[i]);
  }
  if (hex.str() != expected_sha || expected_sha != kCoreSha) {
    return false;
  }
  validated.store(true);
  (void)Cleanup();  // Construct before any Gazebo Server or transport object.
  return true;
}

void Drain() {Cleanup().Drain();}
bool Finish() {return Cleanup().Finish();}
Statistics GetStatistics() {return Cleanup().Statistics();}
}  // namespace easyfly_transport

namespace gazebo::transport
{
IOManager::IOManager()
: dataPtr(nullptr)
{
  if (!validated.load()) {
    std::cerr << "EasyFly transport: unsupported/unvalidated library\n";
    std::abort();
  }
  auto state = std::make_unique<IOManagerPrivate>();
  auto io = std::make_unique<boost::asio::io_service>();
  auto work = std::make_unique<boost::asio::io_service::work>(*io);
  // The run target owns no reference to IOManager itself.
  auto thread = std::make_unique<boost::thread>([service = io.get()] {service->run();});
  state->io_service = io.release();
  state->work = work.release();
  state->thread = thread.release();
  Cleanup().Created();
  this->dataPtr = state.release();
}

IOManager::~IOManager()
{
  State * state = this->dataPtr;
  state->io_service->stop();
  this->dataPtr = nullptr;
  if (state->thread && state->thread->get_id() == boost::this_thread::get_id()) {
    Cleanup().Retire(state);
    return;
  }
  Dispose(state);
  Cleanup().Destroyed();
}

void IOManager::Stop()
{
  // reset() while run() is active is not valid; stop() alone is sufficient.
  this->dataPtr->io_service->stop();
  if (this->dataPtr->thread &&
    this->dataPtr->thread->get_id() != boost::this_thread::get_id())
  {
    this->dataPtr->thread->join();
    delete this->dataPtr->thread;
    this->dataPtr->thread = nullptr;
  }
  // On the worker, only request stop. Destruction transfers ALL state to the
  // reaper, which joins before freeing work/service/state. No detach or leak.
}

boost::asio::io_service & IOManager::GetIO() {return *this->dataPtr->io_service;}
void IOManager::IncCount() {this->dataPtr->count++;}
void IOManager::DecCount() {this->dataPtr->count--;}
unsigned int IOManager::GetCount() const {return this->dataPtr->count;}
}  // namespace gazebo::transport
