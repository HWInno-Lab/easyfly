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
#ifndef EASYFLY_TRANSPORT__SIGNAL_RELAY_HPP_
#define EASYFLY_TRANSPORT__SIGNAL_RELAY_HPP_
#include <atomic>
#include <cerrno>
#include <csignal>
#include <functional>
#include <stdexcept>
#include <thread>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace easyfly_transport
{
inline volatile sig_atomic_t signal_write_fd = -1;
// A self-pipe also handles library threads which explicitly unblock signals.
// The handler only preserves errno and performs one nonblocking write(2).
class SignalRelay
{
public:
  static void notify(int signal)
  {
    const int saved = errno;
    const unsigned char value = static_cast<unsigned char>(signal);
    const int fd = signal_write_fd;
    if (fd >= 0) {const auto ignored = write(fd, &value, 1); (void)ignored;}
    errno = saved;
  }
  SignalRelay()
  {
    if (signal_write_fd >= 0 || pipe2(pipe_, O_NONBLOCK | O_CLOEXEC) != 0) {
      throw std::runtime_error("could not create exclusive simulator signal pipe");
    }
    signal_write_fd = pipe_[1];
    struct sigaction action{}; action.sa_handler = notify; sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, &previous_int_) != 0 ||
      sigaction(SIGTERM, &action, &previous_term_) != 0)
    {
      throw std::runtime_error("could not install simulator signal handoff");
    }
  }
  SignalRelay(const SignalRelay &) = delete;
  SignalRelay & operator=(const SignalRelay &) = delete;
  void start(std::function<void()> shutdown)
  {
    if (thread_.joinable()) {throw std::logic_error("signal relay already started");}
    // Gazebo ROS init may have registered its handlers during ParseArgs.
    struct sigaction action{}; action.sa_handler = notify; sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, nullptr) != 0 || sigaction(SIGTERM, &action, nullptr) != 0) {
      throw std::runtime_error("could not restore simulator signal handoff");
    }
    thread_ = std::thread([this, shutdown] {
      bool delivered = false;
      while (!finished_.load()) {
        pollfd descriptor{pipe_[0], POLLIN, 0};
        if (poll(&descriptor, 1, 20) <= 0) {continue;}
        unsigned char values[64];
        const auto count = read(pipe_[0], values, sizeof(values));
        for (ssize_t i = 0; i < count; ++i) {
          if ((values[i] == SIGINT || values[i] == SIGTERM) && !delivered) {
            delivered = true; shutdown();
          }
        }
      }
    });
  }
  void finish()
  {
    finished_.store(true);
    if (thread_.joinable()) {thread_.join();}
  }
  ~SignalRelay()
  {
    finish();
    signal_write_fd = -1;
    sigaction(SIGINT, &previous_int_, nullptr); sigaction(SIGTERM, &previous_term_, nullptr);
    close(pipe_[0]); close(pipe_[1]);
  }
private:
  int pipe_[2]{-1, -1};
  struct sigaction previous_int_{}, previous_term_{};
  std::atomic<bool> finished_{false};
  std::thread thread_;
};
}  // namespace easyfly_transport
#endif  // EASYFLY_TRANSPORT__SIGNAL_RELAY_HPP_
