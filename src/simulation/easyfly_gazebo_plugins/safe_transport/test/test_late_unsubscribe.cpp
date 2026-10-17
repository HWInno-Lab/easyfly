// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Reproduce GUI subscriptions released by physics after transport finalization.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdexcept>
#include <gazebo/Master.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "easyfly_transport/compat.hpp"

struct Listener
{
  int fd = -1;
  unsigned short port = 0;

  Listener()
  {
    fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (fd < 0 || bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ||
      listen(fd, 1))
    {
      throw std::runtime_error("listener failed");
    }
    socklen_t size = sizeof(address);
    getsockname(fd, reinterpret_cast<sockaddr *>(&address), &size);
    port = ntohs(address.sin_port);
  }

  ~Listener() {if (fd >= 0) {close(fd);}}
};

int main()
{
  if (!easyfly_transport::ValidateLibrary()) {return 2;}
  Listener master_port;
  close(master_port.fd);
  master_port.fd = -1;
  gazebo::Master master;
  master.Init(master_port.port);
  master.RunThread();
  if (!gazebo::transport::init("127.0.0.1", master_port.port, 1)) {return 3;}
  gazebo::transport::run();
  Listener peer;
  auto connection = easyfly_transport::MakeConnection();
  if (!connection->Connect("127.0.0.1", peer.port)) {return 4;}
  int stream = accept(peer.fd, nullptr, nullptr);
  if (stream < 0) {return 5;}
  auto link = gazebo::transport::PublicationTransportPtr(
    new gazebo::transport::PublicationTransport("/gazebo/default/late", "gazebo.msgs.Response"));
  link->Init(connection, false);
  connection.reset();
  // Server::Run finalizes transport before World::Fini releases these readers.
  gazebo::transport::fini();
  master.Fini();
  link.reset();  // Calls Unsubscribe with an already released master connection.
  gazebo::msgs::Subscribe sub;
  sub.set_topic("/gazebo/default/late");
  sub.set_msg_type("gazebo.msgs.Response");
  sub.set_host("127.0.0.1");
  sub.set_port(peer.port);
  gazebo::transport::ConnectionManager::Instance()->Unsubscribe(sub);
  gazebo::transport::ConnectionManager::Instance()->Unsubscribe(sub.topic(), sub.msg_type());
  easyfly_transport::ShutdownConnections();
  bool drained = easyfly_transport::Finish();
  close(stream);
  return drained ? 0 : 6;
}
