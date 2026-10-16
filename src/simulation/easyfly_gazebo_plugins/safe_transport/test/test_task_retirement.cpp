// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <future>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <gtest/gtest.h>
#include <gazebo/Master.hh>
#include <gazebo/transport/transport.hh>
#include <tbb/task.h>
#include "easyfly_transport/compat.hpp"
using namespace std::chrono_literals;
namespace
{
struct Listener
{
  int fd{-1}; unsigned short port{};
  Listener()
  {
    fd=socket(AF_INET,SOCK_STREAM,0);
    sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(fd<0 || bind(fd,reinterpret_cast<sockaddr *>(&a),sizeof(a)) || listen(fd,1))
      throw std::runtime_error("loopback listener failed");
    socklen_t size=sizeof(a);getsockname(fd,reinterpret_cast<sockaddr *>(&a),&size);
    port=ntohs(a.sin_port);
  }
  ~Listener(){if(fd>=0)close(fd);}
};
struct Handler
{
  // Reverse member destruction releases self first. A self-wait would also
  // retain extra, reproducing the observed live=1/pending=0 shutdown footprint.
  gazebo::transport::ConnectionPtr extra, self;
  std::promise<void> *entered;
  std::shared_future<void> release;
  void operator()(const std::string &data) const
  {
    EXPECT_EQ(data,"x");EXPECT_NE(tbb::task::current_context(),nullptr);
    entered->set_value();release.wait();
  }
};
bool OnlyOwners(const gazebo::transport::ConnectionPtr &connection,long count)
{
  const auto end=std::chrono::steady_clock::now()+3s;
  while(connection.use_count()!=count && std::chrono::steady_clock::now()<end)
    std::this_thread::yield();
  return connection.use_count()==count;
}
}
// Finish is terminal, so this regression has its own executable.
TEST(TransportTaskRetirement, PreservesOwnerWaitAndAvoidsReadTaskSelfWait)
{
  ASSERT_TRUE(easyfly_transport::ValidateLibrary());
  Listener master_port;close(master_port.fd);master_port.fd=-1;
  gazebo::Master master;master.Init(master_port.port);master.RunThread();
  ASSERT_TRUE(gazebo::transport::init("127.0.0.1",master_port.port,1));
  gazebo::transport::run();
  {
    Listener remote;
    auto connection=easyfly_transport::MakeConnection();
    ASSERT_TRUE(connection->Connect("127.0.0.1",remote.port));
    int peer=accept(remote.fd,nullptr,nullptr);ASSERT_GE(peer,0);
    std::promise<void> entered,release,reset_started;
    auto ready=entered.get_future();auto gate=release.get_future().share();
    connection->AsyncRead([&](const std::string &){entered.set_value();gate.wait();});
    ASSERT_EQ(send(peer,"00000001x",9,MSG_NOSIGNAL),9);
    const bool active=ready.wait_for(3s)==std::future_status::ready;
    EXPECT_TRUE(active);EXPECT_TRUE(OnlyOwners(connection,1));
    auto started=reset_started.get_future();
    auto done=std::async(std::launch::async,[held=std::move(connection),&reset_started]() mutable {
      EXPECT_EQ(tbb::task::current_context(),nullptr);
      reset_started.set_value();held.reset();
    });
    started.wait();
    // Ordinary owner reset must wait for its raw callback before returning.
    EXPECT_EQ(done.wait_for(100ms),std::future_status::timeout);
    release.set_value();EXPECT_EQ(done.wait_for(3s),std::future_status::ready);
    done.get();close(peer);
  }
  Listener remote;
  auto connection=easyfly_transport::MakeConnection();
  ASSERT_TRUE(connection->Connect("127.0.0.1",remote.port));
  int peer=accept(remote.fd,nullptr,nullptr);ASSERT_GE(peer,0);
  std::promise<void> entered,release;auto ready=entered.get_future();
  auto extra=gazebo::transport::ConnectionPtr(new gazebo::transport::Connection);
  connection->AsyncRead(Handler{extra,connection,&entered,release.get_future().share()});
  ASSERT_EQ(send(peer,"00000001x",9,MSG_NOSIGNAL),9);
  EXPECT_EQ(ready.wait_for(3s),std::future_status::ready);
  EXPECT_TRUE(OnlyOwners(connection,2));
  gazebo::transport::fini();master.Fini();easyfly_transport::ShutdownConnections();
  extra.reset();connection.reset();release.set_value();
  EXPECT_TRUE(easyfly_transport::Finish());
  const auto stats=easyfly_transport::GetStatistics();
  EXPECT_EQ(stats.live,0u);EXPECT_EQ(stats.pending,0u);EXPECT_GE(stats.deferred,1u);
  close(peer);
}
