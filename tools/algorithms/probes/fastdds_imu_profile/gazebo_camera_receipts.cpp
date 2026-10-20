// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Independent Gazebo transport observer; no sensor/plugin modification.
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <openssl/sha.h>

#include <array>
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

namespace
{
volatile std::sig_atomic_t stopped = 0;
void Stop(int) {stopped = 1;}

struct Row
{
  int64_t stamp, mono;
  uint32_t width, height, step;
  int format;
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest;
};

class Observer
{
public:
  std::mutex mutex;
  std::vector<Row> rows;
  size_t discarded = 0;

  Observer() {rows.reserve(20000);}

  void Receive(ConstImageStampedPtr & message)
  {
    Row row{};
    row.mono = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    row.stamp = static_cast<int64_t>(message->time().sec()) * 1000000000LL +
      message->time().nsec();
    const auto & image = message->image();
    row.width = image.width();
    row.height = image.height();
    row.step = image.step();
    row.format = image.pixel_format();
    SHA256(
      reinterpret_cast<const unsigned char *>(image.data().data()),
      image.data().size(), row.digest.data());
    std::lock_guard<std::mutex> lock(mutex);
    if (rows.size() < 20000) {
      rows.push_back(row);
    } else {
      ++discarded;
    }
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2 || !std::filesystem::path(argv[1]).is_absolute() ||
    std::filesystem::exists(argv[1]))
  {
    std::cerr << "Expected fresh absolute output CSV" << std::endl;
    return 2;
  }
  const std::string output(argv[1]);
  gazebo::client::setup();
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  auto node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  Observer observer;
  gazebo::transport::SubscriberPtr subscription;
  std::string topic;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(290);
  while (!stopped && std::chrono::steady_clock::now() < deadline) {
    if (!subscription) {
      for (const auto & name : gazebo::transport::getAdvertisedTopics("gazebo.msgs.ImageStamped")) {
        if (name.find("/color/image") != std::string::npos) {
          if (!topic.empty()) {
            std::cerr << "Ambiguous native color image topics" << std::endl;
            gazebo::client::shutdown();
            return 2;
          }
          topic = name;
        }
      }
      if (!topic.empty()) {
        subscription = node->Subscribe(topic, &Observer::Receive, &observer);
        std::cout << "topic=" << topic << std::endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  subscription.reset();
  node->Fini();
  node.reset();
  gazebo::client::shutdown();
  std::lock_guard<std::mutex> lock(observer.mutex);
  std::ofstream file(output);
  file << "# CLOCK_MONOTONIC capacity=20000 discarded=" << observer.discarded << '\n';
  file << "stamp_ns,mono_ns,width,height,step,format,sha256\n";
  for (const auto & row : observer.rows) {
    file << row.stamp << ',' << row.mono << ',' << row.width << ',' << row.height << ',' <<
      row.step << ',' << row.format << ',';
    for (auto byte : row.digest) {
      file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    file << std::dec << '\n';
  }
  file.close();
  std::cout << "recorded=" << observer.rows.size() << " discarded=" << observer.discarded <<
    std::endl;
  return !file || observer.rows.empty() ? 1 : 0;
}
