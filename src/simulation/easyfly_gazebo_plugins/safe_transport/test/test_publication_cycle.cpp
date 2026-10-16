// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Native reproduction of the captured response Publication/Publisher cycle.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <gazebo/Master.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "easyfly_transport/compat.hpp"
struct Listener {
 int fd;unsigned short port;
 Listener(){fd=socket(AF_INET,SOCK_STREAM,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);if(fd<0||bind(fd,(sockaddr*)&a,sizeof(a))||listen(fd,1))throw std::runtime_error("listener failed");socklen_t n=sizeof(a);getsockname(fd,(sockaddr*)&a,&n);port=ntohs(a.sin_port);}
 ~Listener(){if(fd>=0)close(fd);}
};
int main(){
 if(!easyfly_transport::ValidateLibrary())return 2;
 Listener masterPort;close(masterPort.fd);masterPort.fd=-1;
 gazebo::Master master;master.Init(masterPort.port);master.RunThread();
 if(!gazebo::transport::init("127.0.0.1",masterPort.port,1))return 3;
 gazebo::transport::run();
 auto node=gazebo::transport::NodePtr(new gazebo::transport::Node);node->Init("default");
 std::vector<gazebo::transport::PublisherPtr> publishers;
 for(int i=0;i<4;++i)publishers.push_back(node->Advertise<gazebo::msgs::Response>("~/publication_cycle"));
 auto publication=gazebo::transport::TopicManager::Instance()->FindPublication(publishers[0]->GetTopic());
 if(!publication||publication->PublisherCount()!=4)return 4;
 Listener peer;auto connection=easyfly_transport::MakeConnection();
 if(!connection->Connect("127.0.0.1",peer.port))return 5;
 int fd=accept(peer.fd,nullptr,nullptr);if(fd<0)return 6;
 auto link=gazebo::transport::SubscriptionTransportPtr(new gazebo::transport::SubscriptionTransport);
 link->Init(connection,false);publication->AddSubscription(link);
 boost::weak_ptr<gazebo::transport::Publication> weak_publication=publication;
 boost::weak_ptr<gazebo::transport::Publisher> weak_publisher=publishers[0];
 boost::weak_ptr<gazebo::transport::Connection> weak_connection=connection;
 std::vector<unsigned int> ids;for(const auto& p:publishers)ids.push_back(p->Id());
 connection.reset();link.reset();publication.reset();
 // Real Gazebo shutdown finalizes transport before physics/sensor publishers.
 gazebo::transport::fini();master.Fini();node->Fini();node.reset();
 for(const auto& p:publishers) {
  p->Fini(); // Finalization must remain idempotent.
  if(p->HasConnections() || p->GetRemoteSubscriptionCount()!=0)return 9;
  gazebo::msgs::Response response;response.set_id(1);response.set_request("finalized");response.set_response("success");
  p->Publish(response);p->SendMessage(); // No callback or queued write after Fini.
 }
 publishers.clear();easyfly_transport::ShutdownConnections();
 bool expired=weak_publication.expired()&&weak_publisher.expired()&&weak_connection.expired();
 auto stats=easyfly_transport::GetStatistics();
 std::cerr<<"expired="<<expired<<" publication_owners="<<weak_publication.use_count()<<" publisher_owners="<<weak_publisher.use_count()<<" connection_owners="<<weak_connection.use_count()<<" live="<<stats.live<<" pending="<<stats.pending<<"\n";
 // Only for restoring the old-library repro after measuring the failed state.
 auto retained=weak_publication.lock();if(retained)for(auto id:ids)retained->RemovePublisher(id);retained.reset();
 close(fd);bool drained=easyfly_transport::Finish();std::cerr<<"final_drained="<<drained<<"\n";
 return expired&&drained?0:8;
}
