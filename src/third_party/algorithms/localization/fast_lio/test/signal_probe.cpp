// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Test-only native process probe. No sensor input, flight or production hook.
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <mutex>
#include <pthread.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>

namespace {
void note(int fd, const char *event) {
  char line[160];
  const int size = std::snprintf(line, sizeof(line), "%s tid=%ld\n", event,
    static_cast<long>(::syscall(SYS_gettid)));
  if (size > 0) {::write(fd, line, static_cast<size_t>(size));}
}
void *probe(void *) {
  const char *path = std::getenv("EASYFLY_SIGNAL_PROBE_LOG");
  if (!path) {return nullptr;}
  const int fd = ::open(path, O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC, 0600);
  if (fd < 0) {return nullptr;}
  for (int i = 0; i < 1000 && !rclcpp::ok(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!rclcpp::ok()) {note(fd, "initialization_failed"); return nullptr;}
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // Deliberately interrupt a thread holding a lock that shutdown needs. ROS's
  // dedicated signal thread can wait until this thread resumes and releases it.
  // A synchronous user signal handler instead deadlocks on that same thread.
  auto mutex = std::make_shared<std::mutex>();
  rclcpp::on_shutdown([mutex, fd] {
    note(fd, "callback_enter");
    std::lock_guard<std::mutex> lock(*mutex);
    note(fd, "callback_acquired");
  });
  note(fd, "ready");
  if (std::getenv("EASYFLY_SIGNAL_PROBE_CRITICAL")) {
    std::lock_guard<std::mutex> lock(*mutex);
    note(fd, "critical_signal");
    ::pthread_kill(::pthread_self(), SIGINT);
    note(fd, "signal_returned");
  }
  return nullptr;
}
__attribute__((constructor)) void start_probe() {
  if (!std::getenv("EASYFLY_SIGNAL_PROBE_LOG")) {return;}
  pthread_t worker;
  if (::pthread_create(&worker, nullptr, probe, nullptr) == 0) {::pthread_detach(worker);}
}
}
