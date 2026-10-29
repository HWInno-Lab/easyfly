// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Read-only independent model truth, through native Gazebo transport.
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t stopped = 0;
void Stop(int) {stopped = 1;}
struct Row {int64_t stamp, mono; double x, y, z, qx, qy, qz, qw;};
class Observer {
public:
  std::mutex mutex;
  std::vector<Row> rows;
  size_t discarded = 0;
  void Receive(ConstPosesStampedPtr & msg) {
    for (const auto & p : msg->pose()) {
      if (p.name() != "easyfly_iris") {continue;}
      const auto & v = p.position(); const auto & q = p.orientation();
      Row row{static_cast<int64_t>(msg->time().sec())*1000000000LL + msg->time().nsec(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count(),
        v.x(), v.y(), v.z(), q.x(), q.y(), q.z(), q.w()};
      std::lock_guard<std::mutex> lock(mutex);
      if (rows.size() < 300000) {rows.push_back(row);} else {++discarded;}
    }
  }
};
}
int main(int argc, char ** argv) {
  if (argc != 2 || !std::filesystem::path(argv[1]).is_absolute() ||
      std::filesystem::exists(argv[1])) {return 2;}
  gazebo::client::setup();
  std::signal(SIGINT, Stop); std::signal(SIGTERM, Stop);
  auto node = gazebo::transport::NodePtr(new gazebo::transport::Node()); node->Init();
  Observer observer;
  gazebo::transport::SubscriberPtr sub;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(60);
  while (!stopped && std::chrono::steady_clock::now() < deadline) {
    if (!sub) {
      std::string topic;
      for (const auto & name : gazebo::transport::getAdvertisedTopics("gazebo.msgs.PosesStamped")) {
        if (name.size() >= 10 && name.compare(name.size()-10, 10, "/pose/info") == 0) {
          if (!topic.empty()) {gazebo::client::shutdown(); return 2;}
          topic = name;
        }
      }
      if (!topic.empty()) {sub = node->Subscribe(topic, &Observer::Receive, &observer);
        std::cout << "truth_topic=" << topic << std::endl;}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  sub.reset(); node->Fini(); node.reset(); gazebo::client::shutdown();
  std::lock_guard<std::mutex> lock(observer.mutex);
  std::ofstream file(argv[1]);
  file << "# CLOCK_MONOTONIC capacity=300000 discarded=" << observer.discarded << '\n';
  file << "stamp_ns,mono_ns,x,y,z,qx,qy,qz,qw\n" << std::setprecision(17);
  for (const auto & r : observer.rows) {file << r.stamp << ',' << r.mono << ',' << r.x << ',' <<
    r.y << ',' << r.z << ',' << r.qx << ',' << r.qy << ',' << r.qz << ',' << r.qw << '\n';}
  file.close();
  std::cout << "truth_count=" << observer.rows.size() << std::endl;
  return !file || observer.rows.empty() || observer.discarded ? 1 : 0;
}
