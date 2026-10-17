// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Real accepted TCP subscription, data delivery and partial-read shutdown.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>
#include <gazebo/Master.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "easyfly_transport/compat.hpp"

int connect_peer(const gazebo::msgs::Publish & publication)
{
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(publication.port());
  timeval timeout{3, 0};
  if (fd < 0 || inet_pton(AF_INET, publication.host().c_str(), &address.sin_addr) != 1 ||
    connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ||
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)))
  {
    if (fd >= 0) {close(fd);}
    return -1;
  }
  return fd;
}

int main()
{
  if (!easyfly_transport::ValidateLibrary()) {return 2;}
  int reservation = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (reservation < 0 || bind(reservation, reinterpret_cast<sockaddr *>(&address),
    sizeof(address))) {return 3;}
  socklen_t size = sizeof(address);
  if (getsockname(reservation, reinterpret_cast<sockaddr *>(&address), &size)) {return 4;}
  close(reservation);
  gazebo::Master master;
  master.Init(ntohs(address.sin_port));
  master.RunThread();
  if (!gazebo::transport::init("127.0.0.1", ntohs(address.sin_port), 1)) {return 5;}
  gazebo::transport::run();
  auto node = gazebo::transport::NodePtr(new gazebo::transport::Node);
  node->Init("default");
  auto publisher = node->Advertise<gazebo::msgs::Response>("~/incoming_shutdown");
  gazebo::msgs::Publish endpoint;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  do {
    std::list<gazebo::msgs::Publish> publications;
    gazebo::transport::ConnectionManager::Instance()->GetAllPublishers(publications);
    for (const auto & publication : publications) {
      if (publication.topic() == publisher->GetTopic()) {endpoint = publication;}
    }
    if (!endpoint.has_port()) {std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  } while (!endpoint.has_port() && std::chrono::steady_clock::now() < deadline);
  if (!endpoint.has_port()) {return 6;}
  int peer = connect_peer(endpoint);
  if (peer < 0) {return 7;}
  gazebo::msgs::Subscribe subscription;
  subscription.set_topic(publisher->GetTopic());
  subscription.set_msg_type("gazebo.msgs.Response");
  subscription.set_host("127.0.0.1");
  subscription.set_port(1);
  subscription.set_latching(false);
  auto data = gazebo::msgs::Package("sub", subscription);
  std::ostringstream wire;
  wire << std::hex << std::setw(8) << std::setfill('0') << data.size() << data;
  auto bytes = wire.str();
  if (send(peer, bytes.data(), bytes.size(), MSG_NOSIGNAL) !=
    static_cast<ssize_t>(bytes.size())) {return 8;}
  if (!publisher->WaitForConnection(gazebo::common::Time(2, 0))) {return 9;}
  gazebo::msgs::Response response;
  response.set_id(42);
  response.set_request("native incoming test");
  response.set_response("success");
  publisher->Publish(response, true);
  char header[8];
  if (recv(peer, header, sizeof(header), MSG_WAITALL) != sizeof(header)) {return 10;}
  auto length = std::stoul(std::string(header, sizeof(header)), nullptr, 16);
  if (length > 4096) {return 11;}
  std::string body(length, '\0');
  if (recv(peer, body.data(), body.size(), MSG_WAITALL) !=
    static_cast<ssize_t>(body.size())) {return 12;}
  gazebo::msgs::Response received;
  if (!received.ParseFromString(body) || received.id() != 42) {return 13;}
  int partial = connect_peer(endpoint);
  if (partial < 0 || send(partial, "0000", 4, MSG_NOSIGNAL) != 4) {return 14;}
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  gazebo::transport::fini();
  master.Fini();
  node->Fini();
  publisher->Fini();
  publisher.reset();
  node.reset();
  easyfly_transport::ShutdownConnections();
  bool drained = easyfly_transport::Finish();
  close(peer);
  close(partial);
  return drained ? 0 : 15;
}
