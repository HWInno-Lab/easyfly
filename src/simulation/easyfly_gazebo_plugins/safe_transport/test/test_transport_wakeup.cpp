// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// A local callback republishes while RunUpdate is active: notification must persist.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <gazebo/Master.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "easyfly_transport/compat.hpp"

struct Relay {
  gazebo::transport::PublisherPtr publisher;
  std::mutex mutex;
  std::condition_variable ready;
  int count=0;
  bool ordered=true;
  void Receive(ConstIntPtr & message) {
    std::lock_guard<std::mutex> lock(mutex);
    ordered &= message->data()==count;
    ++count;
    if(count<20) {gazebo::msgs::Int next;next.set_data(count);publisher->Publish(next);}
    else ready.notify_all();
  }
};
int main() {
  if(!easyfly_transport::ValidateLibrary())return 2;
  int fd=socket(AF_INET,SOCK_STREAM,0);sockaddr_in addr{};
  addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  if(fd<0 || bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr)))return 3;
  socklen_t size=sizeof(addr);getsockname(fd,reinterpret_cast<sockaddr*>(&addr),&size);
  auto port=ntohs(addr.sin_port);close(fd);
  gazebo::Master master;master.Init(port);master.RunThread();
  if(!gazebo::transport::init("127.0.0.1",port,1))return 4;
  gazebo::transport::run();
  auto node=gazebo::transport::NodePtr(new gazebo::transport::Node);node->Init("default");
  Relay relay;relay.publisher=node->Advertise<gazebo::msgs::Int>("~/wakeup_chain",32);
  auto subscription=node->Subscribe("~/wakeup_chain",&Relay::Receive,&relay);
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!relay.publisher->HasConnections() && std::chrono::steady_clock::now()<deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  auto start=std::chrono::steady_clock::now();
  gazebo::msgs::Int seed;seed.set_data(0);relay.publisher->Publish(seed);
  bool complete;
  {std::unique_lock<std::mutex> lock(relay.mutex);
   complete=relay.ready.wait_for(lock,std::chrono::seconds(4),[&]{return relay.count==20;});}
  const auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  std::cerr<<"ordered="<<relay.ordered<<" count="<<relay.count<<" elapsed_s="<<elapsed<<"\n";
  gazebo::transport::fini();master.Fini();subscription.reset();relay.publisher.reset();node.reset();
  easyfly_transport::ShutdownConnections();const bool drained=easyfly_transport::Finish();
  // 20 pre-wait lost notifications used to incur approximately 20 x 100 ms.
  return complete && relay.ordered && elapsed<.5 && drained ? 0 : 8;
}
