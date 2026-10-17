// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Actual native producer -> transport -> relay around the original overflow.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "Imu.pb.h"
#include "easyfly_transport/compat.hpp"

struct Capture
{
  std::mutex mutex;
  int count{0};
  int64_t last{0}, sequence{-1};
  bool continuous{true};
  void Receive(const boost::shared_ptr<const sensor_msgs::msgs::Imu> & sample)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!count) {std::cerr << "first_native_sample " << sample->ShortDebugString() << std::endl;}
    if (count && (sample->time_usec() - last != 1000 || sample->seq() != sequence + 1)) {
      continuous = false;
    }
    last = sample->time_usec(); sequence = sample->seq(); ++count;
  }
};
int main(int argc, char ** argv)
{
  const std::string plugin = argc > 1 ? argv[1] : NATIVE_IMU_LIBRARY;
  int reservation = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (reservation < 0 || bind(reservation, reinterpret_cast<sockaddr *>(&address),
    sizeof(address))) {return 2;}
  socklen_t size = sizeof(address);
  if (getsockname(reservation, reinterpret_cast<sockaddr *>(&address), &size)) {return 3;}
  close(reservation);
  const auto uri = "http://127.0.0.1:" + std::to_string(ntohs(address.sin_port));
  setenv("GAZEBO_MASTER_URI", uri.c_str(), true);
  gazebo::common::Console::SetQuiet(false);
  if (!easyfly_transport::ValidateLibrary() || !gazebo::setupServer()) {return 4;}
  auto world = gazebo::loadWorld("worlds/empty.world");
  if (!world) {return 5;}
  world->Physics()->SetMaxStepSize(.001);
  world->Physics()->SetRealTimeUpdateRate(1000);
  world->SetSimTime(gazebo::common::Time(2146, 0));
  world->SetPaused(true); world->Run(0);
  const std::string model =
    "<sdf version='1.6'><model name='overflow_probe'><pose>0 0 .2 0 0 0</pose>"
    "<link name='imu_link'><inertial><mass>1</mass><inertia><ixx>.01</ixx>"
    "<iyy>.01</iyy><izz>.01</izz></inertia></inertial><collision name='body'>"
    "<geometry><box><size>.2 .2 .2</size></box></geometry></collision></link>"
    "<plugin name='native' filename='" + plugin + "'>"
    "<robotNamespace>default</robotNamespace><linkName>imu_link</linkName>"
    "<imuTopic>/imu_native</imuTopic></plugin></model></sdf>";
  sdf::SDF sdf;
  sdf.SetFromString(model); world->InsertModelSDF(sdf);
  for (int i=0; i<1000 && !world->ModelByName("overflow_probe"); ++i) {
    world->Step(1); std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto aircraft = world->ModelByName("overflow_probe");
  if (!aircraft) {return 7;}
  // Synchronous fixture insertion may update once at dt=0. Warm the native
  // source before attaching the relay; the measured stream starts afterwards.
  for (int i=0; i<25; ++i) {world->Step(1);}
  sdf::SDF relay_sdf;
  relay_sdf.SetFromString(
    "<sdf version='1.6'><model name='relay_description'>"
    "<plugin name='relay' filename='" + std::string(IMU_RELAY_LIBRARY) + "'>"
    "<robotNamespace>default</robotNamespace><inputTopic>/imu_native</inputTopic>"
    "<outputTopic>/imu</outputTopic></plugin></model></sdf>");
  auto relay = gazebo::ModelPlugin::Create(IMU_RELAY_LIBRARY, "relay");
  if (!relay) {return 8;}
  relay->Load(aircraft,
    relay_sdf.Root()->GetElement("model")->GetElement("plugin"));
  auto node = gazebo::transport::NodePtr(new gazebo::transport::Node);
  node->Init("default");
  Capture raw, filtered;
  auto a = node->Subscribe("~/overflow_probe/imu_native", &Capture::Receive, &raw);
  auto b = node->Subscribe("~/overflow_probe/imu", &Capture::Receive, &filtered);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  // Pacing lets the real async transport dispatch each native sample; no mock samples.
  for (int i = 0; i < 6000; ++i) {
    world->Step(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  bool passed;
  {
    std::scoped_lock lock(raw.mutex, filtered.mutex);
    std::cout << "raw_count=" << raw.count << " raw_usec=" << raw.last <<
      " filtered_count=" << filtered.count << " filtered_usec=" << filtered.last <<
      " continuous=" << filtered.continuous << std::endl;
    passed = raw.count > 4000 && filtered.count > 4000 && filtered.continuous &&
      filtered.last > 2150000000LL;
  }
  a.reset(); b.reset(); node->Fini(); node.reset(); relay.reset(); aircraft.reset(); world.reset();
  gazebo::shutdown(); easyfly_transport::ShutdownConnections();
  return easyfly_transport::Finish() && passed ? 0 : 6;
}
