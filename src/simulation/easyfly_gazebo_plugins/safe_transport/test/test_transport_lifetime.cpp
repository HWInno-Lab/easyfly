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

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include <gazebo/transport/Connection.hh>
#include <gazebo/transport/IOManager.hh>
#include "easyfly_transport/compat.hpp"

using namespace std::chrono_literals;

class TransportLifetime : public ::testing::Test
{
protected:
  void SetUp() override {ASSERT_TRUE(easyfly_transport::ValidateLibrary());}
  void TearDown() override
  {
    easyfly_transport::Drain();
    auto stats = easyfly_transport::GetStatistics();
    EXPECT_EQ(stats.live, 0u);
    EXPECT_EQ(stats.pending, 0u);
  }
};

TEST_F(TransportLifetime, RejectsUnreviewedLibraryFingerprint)
{
  EXPECT_FALSE(easyfly_transport::ValidateLibrary("unreviewed"));
}

TEST_F(TransportLifetime, ExternalStopAndDestructionJoin)
{
  auto before = easyfly_transport::GetStatistics();
  {
    gazebo::transport::IOManager manager;
    manager.IncCount();
    EXPECT_EQ(manager.GetCount(), 1u);
    manager.DecCount();
    EXPECT_EQ(manager.GetCount(), 0u);
    manager.Stop();
    manager.Stop();
  }
  auto after = easyfly_transport::GetStatistics();
  EXPECT_EQ(after.completed, before.completed + 1);
  EXPECT_EQ(after.deferred, before.deferred);
}

TEST_F(TransportLifetime, WorkerStopRetainsResourcesUntilExternalDestruction)
{
  gazebo::transport::IOManager manager;
  std::promise<void> stopped;
  auto ready = stopped.get_future();
  manager.GetIO().post(
    [&] {
      manager.Stop();
      stopped.set_value();
    });
  ASSERT_EQ(ready.wait_for(3s), std::future_status::ready);
  manager.Stop();  // External caller still performs the actual join.
}

TEST_F(TransportLifetime, SelfDestructionKeepsServiceUntilCallbackReturns)
{
  auto before = easyfly_transport::GetStatistics();
  auto * manager = new gazebo::transport::IOManager;
  std::promise<void> destroyed;
  std::promise<void> release;
  auto destroyed_future = destroyed.get_future();
  auto go = release.get_future();
  manager->GetIO().post(
    [&] {
      delete manager;
      destroyed.set_value();
      // The IO run frame is deliberately still active.
      go.wait();
    });
  auto status = destroyed_future.wait_for(3s);
  if (status != std::future_status::ready) {
    release.set_value();
    FAIL() << "Self destruction did not return";
  }
  auto during = easyfly_transport::GetStatistics();
  EXPECT_EQ(during.live, 0u);
  EXPECT_EQ(during.pending, 1u);
  EXPECT_EQ(during.completed, before.completed);
  auto drain = std::async(std::launch::async, [] {easyfly_transport::Drain();});
  EXPECT_EQ(drain.wait_for(30ms), std::future_status::timeout);
  release.set_value();
  ASSERT_EQ(drain.wait_for(3s), std::future_status::ready);
  drain.get();
  auto after = easyfly_transport::GetStatistics();
  EXPECT_EQ(after.deferred, before.deferred + 1);
  EXPECT_EQ(after.completed, before.completed + 1);
}

TEST_F(TransportLifetime, ExternalDestructionWaitsForActiveCallback)
{
  auto * manager = new gazebo::transport::IOManager;
  std::promise<void> entered;
  std::promise<void> release;
  auto ready = entered.get_future();
  auto go = release.get_future();
  manager->GetIO().post([&] {entered.set_value(); go.wait();});
  auto status = ready.wait_for(3s);
  if (status != std::future_status::ready) {
    release.set_value();
    delete manager;
    FAIL() << "Callback did not enter";
  }
  auto destroy = std::async(std::launch::async, [manager] {delete manager;});
  EXPECT_EQ(destroy.wait_for(30ms), std::future_status::timeout);
  release.set_value();
  ASSERT_EQ(destroy.wait_for(3s), std::future_status::ready);
  destroy.get();
}

TEST_F(TransportLifetime, FinishRefusesLiveManagerWithoutStoppingCleanup)
{
  {
    gazebo::transport::IOManager manager;
    EXPECT_FALSE(easyfly_transport::Finish());
  }
  // Further managers still work after the refused, incomplete shutdown.
  gazebo::transport::IOManager second;
}

TEST_F(TransportLifetime, ActualLastConnectionReleasedByInstalledWeakBinder)
{
  auto before = easyfly_transport::GetStatistics();
  int listener = socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(listener, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
  ASSERT_EQ(listen(listener, 1), 0);
  socklen_t length = sizeof(address);
  ASSERT_EQ(getsockname(listener, reinterpret_cast<sockaddr *>(&address), &length), 0);
  std::thread peer([listener] {
      int client = accept(listener, nullptr, nullptr);
      if (client >= 0) {
        send(client, "00000000", 8, MSG_NOSIGNAL);
        close(client);
      }
      close(listener);
    });
  std::promise<void> entered;
  std::promise<void> release;
  auto ready = entered.get_future();
  auto go = release.get_future();
  auto connection = gazebo::transport::ConnectionPtr(new gazebo::transport::Connection);
  bool connected = connection->Connect("127.0.0.1", ntohs(address.sin_port));
  if (!connected) {
    shutdown(listener, SHUT_RDWR);
    peer.join();
    FAIL() << "Local fixture connection failed";
  }
  connection->AsyncRead([&](const std::string &) {entered.set_value(); go.wait();});
  auto status = ready.wait_for(3s);
  if (status != std::future_status::ready) {
    release.set_value();
    connection.reset();
    peer.join();
    FAIL() << "Actual IO callback did not enter";
  }
  peer.join();
  connection.reset();  // Installed WeakBinder now owns the sole strong ref.
  release.set_value();
  // Drain can start just before the callback queues destruction. Wait for the
  // live/pending lifecycle transition, not an arbitrary fixed sleep.
  auto deadline = std::chrono::steady_clock::now() + 3s;
  while (easyfly_transport::GetStatistics().live != 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  easyfly_transport::Drain();
  auto after = easyfly_transport::GetStatistics();
  EXPECT_EQ(after.deferred, before.deferred + 1);
  EXPECT_EQ(after.completed, before.completed + 1);
}

TEST_F(TransportLifetime, ConcurrentLastConnectionsRetireManagerOnce)
{
  for (int iteration = 0; iteration < 200; ++iteration) {
    auto * first = new gazebo::transport::Connection;
    auto * second = new gazebo::transport::Connection;
    std::atomic<int> ready{0};
    std::atomic<bool> release{false};
    auto drop = [&](gazebo::transport::Connection * connection) {
        ++ready;
        while (!release.load()) {std::this_thread::yield();}
        delete connection;
      };
    std::thread one(drop, first), two(drop, second);
    while (ready.load() != 2) {std::this_thread::yield();}
    release = true;
    one.join(); two.join();
    easyfly_transport::Drain();
    ASSERT_EQ(easyfly_transport::GetStatistics().live, 0u) << iteration;
    ASSERT_EQ(easyfly_transport::GetStatistics().pending, 0u) << iteration;
  }
}

TEST_F(TransportLifetime, ConcurrentConstructionSharesOneManager)
{
  for (int iteration = 0; iteration < 100; ++iteration) {
    std::atomic<int> ready{0};
    std::atomic<bool> release{false};
    auto construct = [&] {
        auto connection = std::make_unique<gazebo::transport::Connection>();
        ++ready;
        while (!release.load()) {std::this_thread::yield();}
      };
    std::thread one(construct), two(construct);
    while (ready.load() != 2) {std::this_thread::yield();}
    const auto live = easyfly_transport::GetStatistics().live;
    release = true;
    one.join(); two.join();
    ASSERT_EQ(live, 1u) << iteration;
    easyfly_transport::Drain();
    ASSERT_EQ(easyfly_transport::GetStatistics().live, 0u) << iteration;
  }
}
