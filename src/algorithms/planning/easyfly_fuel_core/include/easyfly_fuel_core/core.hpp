// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <string>
#include <cstdint>
namespace easyfly::fuel {
using Vec=Eigen::Vector3d;
struct Snapshot {
 Eigen::Vector3i size;
 Vec origin,min_bound,max_bound;
 double resolution{.2};
 std::vector<int8_t> raw,inflated;
};
struct Options {
 bool camera_viewpoints{false};
 Vec sensor_offset{Vec(0.,0.,.13)};
 double horizontal_half_angle{.7},vertical_half_angle{.5},sensor_range{3.},speed{.3};
};
struct Decision {
 std::string reason;
 Vec goal{Vec::Zero()};double yaw{0};
 uint64_t updates{0};
 std::vector<std::vector<Vec>> frontiers,dormant;
 std::vector<Vec> viewpoints,global_tour,refined_tour,native_path,visible_frontier;
};
// One instance per process: official ViewNode/LKH contain shared static state.
class Session {
 public:
 Session();explicit Session(const Options&);~Session();Session(const Session&)=delete;Session& operator=(const Session&)=delete;
 Decision decide(const Snapshot&,const Vec& position,const Vec& velocity,double yaw,const std::string& job_directory);
 private:struct Impl;std::unique_ptr<Impl> impl_;
};
}
