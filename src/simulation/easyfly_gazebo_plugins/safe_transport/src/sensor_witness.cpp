// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Read-only native physics sensor origin. No ROS publisher or control client.
#include "easyfly_transport/compat.hpp"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
namespace {
volatile std::sig_atomic_t stop=0;
void Stop(int){stop=1;}
struct Row {int64_t stamp,mono;double x,y,z,qx,qy,qz,qw,min,max;};
class Observer {
 public:
 std::mutex mutex;std::vector<Row> rows;size_t discarded=0;
 void receive(ConstLaserScanStampedPtr& msg){
  const auto& p=msg->scan().world_pose();const auto& v=p.position();const auto& q=p.orientation();
  Row row{int64_t(msg->time().sec())*1000000000LL+msg->time().nsec(),
   std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(),
   v.x(),v.y(),v.z(),q.x(),q.y(),q.z(),q.w(),msg->scan().range_min(),msg->scan().range_max()};
  std::lock_guard<std::mutex> lock(mutex);
  if(rows.size()<30000)rows.push_back(row);else ++discarded;
 }
};
}
int main(int argc,char** argv){
 if(argc!=2 || !std::filesystem::path(argv[1]).is_absolute() || std::filesystem::exists(argv[1]))return 2;
 if(!easyfly_transport::ValidateLibrary())return 2;
 std::signal(SIGINT,Stop);std::signal(SIGTERM,Stop);
 // This observer needs only transport; client::setup would also create an
 // unrelated ModelDatabase lifecycle and install process-wide signal state.
 if(!gazebo::transport::init("",0,5))return 3;
 gazebo::transport::run();
 auto node=gazebo::transport::NodePtr(new gazebo::transport::Node());
 if(!node->TryInit(gazebo::common::Time(5,0)))stop=1;
 bool ambiguous=false;
 Observer observer;gazebo::transport::SubscriberPtr sub;
 auto deadline=std::chrono::steady_clock::now()+std::chrono::minutes(60);
 while(!stop && std::chrono::steady_clock::now()<deadline){
  if(!sub){
   auto topics=gazebo::transport::getAdvertisedTopics("gazebo.msgs.LaserScanStamped");
   if(topics.size()>1){ambiguous=true;break;}
   if(topics.size()==1){sub=node->Subscribe(topics.front(),&Observer::receive,&observer);std::cout<<topics.front()<<std::endl;}
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
 }
 std::cerr<<"native_witness_shutdown: unsubscribe\n";
 sub.reset();node->Fini();node.reset();
 std::cerr<<"native_witness_shutdown: transport_fini\n";
 gazebo::transport::fini();
 easyfly_transport::ShutdownConnections();
 if(!easyfly_transport::Finish()){
  std::cerr<<"native_witness_shutdown: retained transport owners\n";return 4;
 }
 const auto statistics=easyfly_transport::GetStatistics();
 std::cerr<<"native_witness_shutdown: drained live="<<statistics.live
          <<" pending="<<statistics.pending<<" deferred_total="<<statistics.deferred
          <<" completed_total="<<statistics.completed<<'\n';
 // deferred is a cumulative count of successfully scheduled retirements.
 // Finish has joined them; only live and pending represent retained work.
 if(statistics.live || statistics.pending || ambiguous)return 5;
 std::lock_guard<std::mutex> lock(observer.mutex);std::ofstream out(argv[1]);
 out<<"# read_only_native_scan_world_pose capacity=30000 discarded="<<observer.discarded<<'\n';
 out<<"stamp_ns,mono_ns,x,y,z,qx,qy,qz,qw,range_min,range_max\n"<<std::setprecision(17);
 for(auto& r:observer.rows)out<<r.stamp<<','<<r.mono<<','<<r.x<<','<<r.y<<','<<r.z<<','<<r.qx<<','<<r.qy<<','<<r.qz<<','<<r.qw<<','<<r.min<<','<<r.max<<'\n';
 out.close();return !out || observer.rows.empty() || observer.discarded?1:0;
}
