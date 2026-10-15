// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <future>
#include <thread>
#include <stdexcept>
#include <gtest/gtest.h>
#include <gazebo/Master.hh>
#include <gazebo/transport/transport.hh>
#include "easyfly_transport/compat.hpp"
namespace
{
struct Listener
{
  int fd{-1};
  unsigned short port{};
  Listener()
  {
    fd=socket(AF_INET,SOCK_STREAM,0);
    sockaddr_in address{};
    address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if (fd<0 || bind(fd,reinterpret_cast<sockaddr *>(&address),sizeof(address)) || listen(fd,1)) {
      if (fd>=0) {close(fd);fd=-1;}
      throw std::runtime_error("loopback listener failed");
    }
    socklen_t size=sizeof(address);
    getsockname(fd,reinterpret_cast<sockaddr *>(&address),&size);
    port=ntohs(address.sin_port);
  }
  ~Listener() {if(fd>=0)close(fd);}
};
}
// Finish closes the process-wide reaper: use a dedicated test executable.
TEST(TransportPendingRead, FiniCancelsOwnedReadWhilePeerRemainsOpen)
{
  ASSERT_TRUE(easyfly_transport::ValidateLibrary());
  Listener master_port;
  close(master_port.fd);master_port.fd=-1;
  gazebo::Master master;
  master.Init(master_port.port);master.RunThread();
  ASSERT_TRUE(gazebo::transport::init("127.0.0.1",master_port.port,1));
  gazebo::transport::run();
  Listener remote;
  std::promise<int> accepted;
  auto ready=accepted.get_future();
  std::thread peer([&] {accepted.set_value(accept(remote.fd,nullptr,nullptr));});
  auto connection=gazebo::transport::ConnectionManager::Instance()->ConnectToRemoteHost(
    "127.0.0.1",remote.port);
  if (!connection) {
    shutdown(remote.fd,SHUT_RDWR);peer.join();
    gazebo::transport::fini();master.Fini();
    EXPECT_TRUE(easyfly_transport::Finish());
    FAIL()<<"local connection did not open";
  }
  const int client=ready.get();peer.join();
  // This is the same strong ConnectionPtr capture as ConnectionManager::OnAccept.
  // No header arrives, so simply dropping the manager's list cannot release it.
  connection->AsyncRead([held=connection](const std::string &) {});
  connection.reset();
  gazebo::transport::fini();master.Fini();
  const bool finished=easyfly_transport::Finish();
  EXPECT_GE(fcntl(client,F_GETFD),0);  // The peer was not closed to make Finish pass.
  close(client);
  if (!finished) {EXPECT_TRUE(easyfly_transport::Finish());}
  EXPECT_TRUE(finished)<<"Fini retained its own pending socket read";
  const auto statistics=easyfly_transport::GetStatistics();
  EXPECT_EQ(statistics.live,0u);EXPECT_EQ(statistics.pending,0u);
}
