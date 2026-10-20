// Copyright 2026 EasyFly Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <stdexcept>
#include <cerrno>

namespace easyfly_camera_calib
{
struct Mount
{
  std::string parent, child;
  std::array<double, 3> translation{}, rotation_rpy{};
};
struct Intrinsics
{
  int width{}, height{};
  double fx{}, fy{}, cx{}, cy{};
  std::string distortion_model;
  std::vector<double> d;
};
struct Calibration
{
  std::vector<Mount> mounts;
  Intrinsics camera;
  std::string camera_frame;
  bool has_camera{false};
};
inline void require(bool ok, const std::string & message)
{
  if (!ok) {throw std::invalid_argument(message);}
}
inline void fields(const YAML::Node & node, const std::set<std::string> & allowed)
{
  require(node.IsMap(), "expected calibration mapping");
  std::set<std::string> seen;
  for (const auto & pair : node) {
    require(pair.first.IsScalar(), "invalid calibration key");
    const auto key = pair.first.as<std::string>();
    require(
      seen.insert(key).second && allowed.count(
        key), "duplicate/unknown calibration field: " + key);
  }
}
inline bool validFrame(const std::string & frame)
{
  if (frame.empty() || frame.size() > 128 || frame.front() == '/' || frame.back() == '/') {
    return false;
  }
  for (char c : frame) {
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
      (c >= '0' && c <= '9') || c == '_' || c == '/')) {return false;}}
  return frame.find("//") == std::string::npos;
}
inline double numeric(const YAML::Node & node, double low, double high)
{
  require(node.IsScalar(), "missing numeric calibration field");
  auto value = node.as<double>();
  require(
    std::isfinite(value) && value >= low && value <= high,
    "calibration number outside bounds");
  return value;
}
inline std::array<double, 3> vector3(const YAML::Node & node, double limit)
{
  require(node.IsSequence() && node.size() == 3, "calibration vector must have 3 entries");
  return {numeric(node[0], -limit, limit), numeric(node[1], -limit, limit), numeric(
      node[2], -limit,
      limit)};
}
inline Mount mount(const YAML::Node & node)
{
  fields(node, {"parent", "child", "translation", "rotation_rpy"});
  Mount value{node["parent"].as<std::string>(), node["child"].as<std::string>(),
    vector3(node["translation"], 100), vector3(node["rotation_rpy"], 3.141592653589793)};
  require(
    validFrame(value.parent) && validFrame(value.child) && value.parent != value.child,
    "invalid/self calibration frame");
  return value;
}
inline void validateGraph(const std::vector<Mount> & mounts)
{
  require(!mounts.empty() && mounts.size() <= 32, "expected 1..32 transforms");
  std::map<std::string, std::string> parents;
  for (const auto & m : mounts) {
    require(parents.emplace(m.child, m.parent).second, "duplicate TF child");
  }
  for (const auto & m : mounts) {
    std::set<std::string> seen;
    auto current = m.child;
    while (parents.count(current)) {
      require(seen.insert(current).second, "calibration TF cycle"); current = parents.at(current);
    }
  }
}
inline Calibration parseCalibration(const std::string & text)
{
  require(
    !text.empty() && text.size() <= 65536 && text.find(
      '\0') == std::string::npos, "calibration must be <=64KiB");
  auto documents = YAML::LoadAll(text); require(
    documents.size() == 1, "single calibration YAML required");
  const auto root = documents.front(); fields(root, {"sensors"});
  const auto sensors = root["sensors"]; fields(sensors, {"camera", "lidar"});
  Calibration result;
  if (const auto camera = sensors["camera"]) {
    fields(camera, {"mount", "optical", "intrinsics"});
    auto m = mount(camera["mount"]); result.mounts.push_back(m);
    const auto optical = camera["optical"];
    fields(optical, {"color_frame", "depth_frame", "translation", "rotation_rpy"});
    auto t = vector3(optical["translation"], 100), r = vector3(
      optical["rotation_rpy"],
      3.141592653589793);
    for (const auto & key : {"color_frame", "depth_frame"}) {
      auto frame = optical[key].as<std::string>(); require(
        validFrame(
          frame) && frame != m.child, "invalid optical frame");
      result.mounts.push_back({m.child, frame, t, r});
    }
    result.camera_frame = optical["color_frame"].as<std::string>();
    const auto intr = camera["intrinsics"];
    fields(intr, {"width", "height", "fx", "fy", "cx", "cy", "distortion_model", "d"});
    auto & i = result.camera; i.width = intr["width"].as<int>();
    i.height = intr["height"].as<int>();
    require(
      i.width > 0 && i.width <= 8192 && i.height > 0 && i.height <= 8192,
      "invalid image dimensions");
    i.fx = numeric(intr["fx"], .001, 100000); i.fy = numeric(intr["fy"], .001, 100000);
    i.cx = numeric(intr["cx"], 0, i.width); i.cy = numeric(intr["cy"], 0, i.height);
    i.distortion_model = intr["distortion_model"].as<std::string>();
    require(
      i.distortion_model == "plumb_bob" && intr["d"].IsSequence() && intr["d"].size() == 5,
      "first calibration format supports plumb_bob with 5 coefficients");
    for (const auto & v : intr["d"]) {
      i.d.push_back(numeric(v, -100, 100));
    }
    result.has_camera = true;
  }
  if (const auto lidar = sensors["lidar"]) {
    fields(lidar, {"mount", "imu_extrinsic"});
    const auto lidar_mount = mount(lidar["mount"]); result.mounts.push_back(lidar_mount);
    if (const auto imu = lidar["imu_extrinsic"]) {
      fields(imu, {"child", "translation", "rotation_rpy"});
      const auto t = vector3(imu["translation"], 100);
      const auto r = vector3(imu["rotation_rpy"], 3.141592653589793);
      // Older configs only validated the extrinsic and did not publish an IMU TF.
      if (imu["child"]) {
        const auto child = imu["child"].as<std::string>();
        require(validFrame(child) && child != lidar_mount.child, "invalid IMU frame");
        result.mounts.push_back({lidar_mount.child, child, t, r});
      }
    }
  }
  validateGraph(result.mounts); return result;
}
inline Calibration loadCalibration(const std::string & path)
{
  const int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NONBLOCK); require(
    fd >= 0,
    "cannot open calibration");
  struct Guard {int fd; ~Guard() {close(fd);}} guard{fd};
  struct stat info {};
  require(
    !fstat(fd, &info) && S_ISREG(
      info.st_mode) && info.st_size <= 65536, "calibration must be regular file <=64KiB");
  std::string text(65537, '\0'); size_t count = 0;
  while (count < text.size()) {
    const auto size = read(fd, text.data() + count, text.size() - count);
    if (size < 0 && errno == EINTR) {continue;}
    require(size >= 0, "calibration read failed");
    if (!size) {break;} count += static_cast<size_t>(size);
  }
  require(count <= 65536, "calibration too large"); text.resize(count);
  return parseCalibration(text);
}
}  // namespace easyfly_camera_calib
