// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <future>
#include <thread>
#include <gazebo/transport/Connection.hh>
#include "easyfly_transport/compat.hpp"

using namespace std::chrono_literals;

// Finish closes the process-wide reaper: keep this in its own test process.
TEST(TransportFinish, WaitsForLastActualConnectionCallbackBeforeClosing)
{
  ASSERT_TRUE(easyfly_transport::ValidateLibrary());
  const int listener = socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(listener, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
  ASSERT_EQ(listen(listener, 1), 0);
  socklen_t length = sizeof(address);
  ASSERT_EQ(getsockname(listener, reinterpret_cast<sockaddr *>(&address), &length), 0);
  std::thread peer([listener] {
      const int client = accept(listener, nullptr, nullptr);
      if (client >= 0) {
        send(client, "00000000", 8, MSG_NOSIGNAL);
        close(client);
      }
      close(listener);
    });
  std::promise<void> entered, release;
  auto ready = entered.get_future();
  auto go = release.get_future();
  auto connection = gazebo::transport::ConnectionPtr(new gazebo::transport::Connection);
  if (!connection->Connect("127.0.0.1", ntohs(address.sin_port))) {
    shutdown(listener, SHUT_RDWR);
    peer.join();
    FAIL() << "Local connection failed";
  }
  connection->AsyncRead([&](const std::string &) {entered.set_value(); go.wait();});
  const auto status = ready.wait_for(3s);
  if (status != std::future_status::ready) {
    release.set_value();
    connection.reset();
    peer.join();
    FAIL() << "Actual transport callback did not enter";
  }
  peer.join();
  connection.reset();  // The actual WeakBinder callback is now the sole owner.
  const auto before = easyfly_transport::GetStatistics();
  EXPECT_EQ(before.live, 1u);
  EXPECT_EQ(before.pending, 0u);  // Nothing has reached the reaper queue yet.
  std::promise<void> finishing;
  auto started = finishing.get_future();
  auto finish = std::async(std::launch::async, [&] {
      finishing.set_value();
      return easyfly_transport::Finish();
    });
  started.wait();
  // This is an intentionally outstanding callback, not a sleep-based release.
  EXPECT_EQ(finish.wait_for(50ms), std::future_status::timeout);
  release.set_value();
  ASSERT_EQ(finish.wait_for(3s), std::future_status::ready);
  EXPECT_TRUE(finish.get());
  // Also clean up after the expected failure on the old implementation.
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (easyfly_transport::GetStatistics().live != 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_TRUE(easyfly_transport::Finish());
  const auto after = easyfly_transport::GetStatistics();
  EXPECT_EQ(after.live, 0u);
  EXPECT_EQ(after.pending, 0u);
  EXPECT_EQ(after.deferred, before.deferred + 1);
  EXPECT_EQ(after.completed, before.completed + 1);
}
