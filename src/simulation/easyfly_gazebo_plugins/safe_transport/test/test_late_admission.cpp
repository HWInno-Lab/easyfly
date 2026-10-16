// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Real TCP backlog gates a native connection until after Gazebo transport Fini.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <future>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <gazebo/Master.hh>
#include <gazebo/transport/transport.hh>
#include "easyfly_transport/compat.hpp"
using namespace std::chrono_literals;
struct Listener {
 int fd;unsigned short port;
 explicit Listener(int backlog=0){fd=socket(AF_INET,SOCK_STREAM,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);if(fd<0||bind(fd,(sockaddr*)&a,sizeof(a))||listen(fd,backlog))throw std::runtime_error("listener failed");socklen_t n=sizeof(a);getsockname(fd,(sockaddr*)&a,&n);port=ntohs(a.sin_port);}
 ~Listener(){if(fd>=0)close(fd);}
};
bool pending(unsigned short port) {
 std::ifstream f("/proc/net/tcp");std::string line;
 std::ostringstream suffix;suffix<<':'<<std::uppercase<<std::hex<<port;
 while(std::getline(f,line)){std::istringstream s(line);std::string index,local,remote,state;s>>index>>local>>remote>>state;if(state=="02"&&remote=="0100007F"+suffix.str())return true;}
 return false;
}
int main(){
 if(!easyfly_transport::ValidateLibrary())return 2;
 Listener masterPort;close(masterPort.fd);masterPort.fd=-1;
 gazebo::Master master;master.Init(masterPort.port);master.RunThread();
 if(!gazebo::transport::init("127.0.0.1",masterPort.port,1))return 3;
 gazebo::transport::run();
 Listener remote(0);int filler=socket(AF_INET,SOCK_STREAM,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(remote.port);
 if(connect(filler,(sockaddr*)&a,sizeof(a)))return 4;
 auto manager=gazebo::transport::ConnectionManager::Instance();
 auto connecting=std::async(std::launch::async,[&]{return manager->ConnectToRemoteHost("127.0.0.1",remote.port);});
 auto deadline=std::chrono::steady_clock::now()+3s;
 while(!pending(remote.port)&&connecting.wait_for(0ms)!=std::future_status::ready&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(1ms);
 if(!pending(remote.port)){std::cerr<<"fixture failed: native SYN was not gated\n";return 5;}
 std::cerr<<"native connection is SYN_SENT before Fini\n";
 gazebo::transport::fini();master.Fini();
 std::cerr<<"Fini returned, initialized="<<manager->IsInitialized()<<"\n";
 const int first=accept(remote.fd,nullptr,nullptr);if(first<0)return 6;
 if(connecting.wait_for(4s)!=std::future_status::ready){std::cerr<<"fixture failed: backlog release did not complete native connect\n";return 7;}
 auto late=connecting.get();bool returned=bool(late);boost::weak_ptr<gazebo::transport::Connection> weak=late;
 late.reset();easyfly_transport::ShutdownConnections();
 const bool expired=weak.expired();const auto stats=easyfly_transport::GetStatistics();
 std::cerr<<"late_returned="<<returned<<" weak_expired="<<expired<<" remaining_owners="<<weak.use_count()<<" live="<<stats.live<<" pending="<<stats.pending<<"\n";
 // Cleanup a reproduced old-library leak through the real public removal API.
 auto retained=weak.lock();if(retained)manager->RemoveConnection(retained);retained.reset();
 close(first);close(filler);
 const bool drained=easyfly_transport::Finish();std::cerr<<"final_drained="<<drained<<"\n";
 return !returned&&expired&&drained?0:8;
}
