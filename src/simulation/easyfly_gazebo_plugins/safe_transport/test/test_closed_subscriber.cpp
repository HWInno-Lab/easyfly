// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gazebo/transport/Publication.hh>
#include <gazebo/transport/SubscriptionTransport.hh>
#include <gazebo/transport/Connection.hh>
#include <gazebo/msgs/msgs.hh>
#include <easyfly_transport/compat.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iostream>
class LocalCallback : public gazebo::transport::CallbackHelper {
public:
 unsigned calls=0;
 LocalCallback():CallbackHelper(true) {}
 bool HandleMessage(gazebo::transport::MessagePtr) override {++calls;return true;}
 bool HandleData(const std::string &, boost::function<void(uint32_t)> done,uint32_t id) override {
  ++calls;if(done)done(id);return true;
 }
 bool IsLocal() const override {return true;}
};
int main() {
 if(!easyfly_transport::ValidateLibrary())return 7;
 int listener=socket(AF_INET,SOCK_STREAM,0);
 sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
 if(bind(listener,reinterpret_cast<sockaddr*>(&addr),sizeof(addr)) || listen(listener,1))return 2;
 socklen_t length=sizeof(addr);getsockname(listener,reinterpret_cast<sockaddr*>(&addr),&length);
 auto connection=easyfly_transport::MakeConnection();
 if(!connection->Connect("127.0.0.1",ntohs(addr.sin_port)))return 3;
 int peer=accept(listener,nullptr,nullptr);if(peer<0)return 4;
 auto publication=gazebo::transport::PublicationPtr(new gazebo::transport::Publication("/retired_peer","gazebo.msgs.Request"));
 for(unsigned i=1;i<=2;++i) {
  gazebo::msgs::RequestPtr message(new gazebo::msgs::Request);
  message->set_id(i);message->set_request("original_latched_message");
  publication->SetPrevMsg(i,message);
 }
 auto subscription=gazebo::transport::SubscriptionTransportPtr(new gazebo::transport::SubscriptionTransport);
 subscription->Init(connection,true);
 connection->Shutdown();close(peer);close(listener);
 std::cerr<<"actual peer closed before two retained latched messages\n";
 publication->AddSubscription(subscription);
 std::cerr<<"latched callbacks remaining="<<publication->GetCallbackCount()<<"\n";
 if(publication->GetCallbackCount()!=0)return 5;
 // A retained external link must reject repeated delivery after retirement.
 gazebo::msgs::RequestPtr next(new gazebo::msgs::Request);
 next->set_id(3);next->set_request("after_disconnect");
 if(subscription->HandleMessage(next) || subscription->HandleMessage(next))return 8;
 // The public add/remove path can also encounter an already retired link.
 subscription->SetLatching(false);
 publication->AddSubscription(subscription);
 publication->RemoveSubscription("127.0.0.1",ntohs(addr.sin_port));
 if(publication->GetCallbackCount()!=0)return 9;
 // A remote unsubscribe must neither cast/dereference a local callback as a
 // transport link nor erase that unrelated live local subscriber.
 auto local=boost::shared_ptr<LocalCallback>(new LocalCallback);
 publication->AddSubscription(local);
 if(local->calls!=2)return 10;
 publication->AddSubscription(subscription);
 publication->RemoveSubscription("127.0.0.1",ntohs(addr.sin_port));
 if(publication->GetCallbackCount()!=1 || !local->HandleMessage(next))return 11;
 local.reset();
 publication.reset();subscription.reset();connection.reset();
 easyfly_transport::ShutdownConnections();if(!easyfly_transport::Finish())return 6;
 return 0;
}
