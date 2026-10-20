// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Read-only WorldUpdate boundary timestamps; bounded memory, flush on unload.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <vector>

namespace gazebo {
class EasyflyStepTrace : public WorldPlugin {
  struct Row {int64_t begin, end, sim;};
  std::vector<Row> rows_;
  size_t discarded_ = 0;
  int64_t begin_ = 0, sim_ = 0;
  event::ConnectionPtr begin_connection_, end_connection_;
  static int64_t Now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
public:
  void Load(physics::WorldPtr, sdf::ElementPtr) override {
    rows_.reserve(150000);
    begin_connection_ = event::Events::ConnectWorldUpdateBegin(
      [this](const common::UpdateInfo & info) {
        begin_ = Now(); sim_ = info.simTime.sec*int64_t(1000000000)+info.simTime.nsec;
      });
    end_connection_ = event::Events::ConnectWorldUpdateEnd([this]() {
      if (rows_.size() < 150000) rows_.push_back({begin_, Now(), sim_});
      else ++discarded_;
    });
  }
  ~EasyflyStepTrace() override {
    begin_connection_.reset(); end_connection_.reset();
    const char * path = std::getenv("EASYFLY_STEP_TRACE");
    if (!path) return;
    std::ofstream out(path);
    out << "# CLOCK_MONOTONIC capacity=150000 discarded=" << discarded_ << "\n";
    out << "begin_ns,end_ns,sim_ns\n";
    for (const auto & r : rows_) out << r.begin << ',' << r.end << ',' << r.sim << '\n';
  }
};
GZ_REGISTER_WORLD_PLUGIN(EasyflyStepTrace)
}
