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
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace easyfly::motion_prediction
{
struct Options
{
  double horizon{.5}, max_age{.5}, max_speed{10}, max_jump{5};
  double min_confidence{.2}, base_variance{.04}, variance_growth{1};
  size_t max_tracks{256};
  void validate() const
  {
    if (!std::isfinite(horizon) || horizon < 0 || horizon > 2 ||
      !std::isfinite(max_age) || max_age < .01 || max_age > 5 ||
      !std::isfinite(max_speed) || max_speed <= 0 || max_speed > 100 ||
      !std::isfinite(max_jump) || max_jump <= 0 || max_jump > 100 ||
      !std::isfinite(min_confidence) || min_confidence < 0 || min_confidence > 1 ||
      !std::isfinite(base_variance) || base_variance < 0 || base_variance > 100 ||
      !std::isfinite(variance_growth) || variance_growth < 0 || variance_growth > 100 ||
      max_tracks < 1 || max_tracks > 1024)
    {
      throw std::invalid_argument("invalid prediction options");
    }
  }
};
struct Observation
{
  int32_t id{-1};
  int64_t stamp_ns{0};
  std::string frame{"odom"}, class_name;
  double confidence{0};
  bool position_valid{false};
  std::array<double, 3> position{};
};
struct Frame
{
  int64_t stamp_ns{0};
  std::string frame{"odom"};
  bool well_formed{true};
  std::vector<Observation> observations;
};
struct Prediction
{
  Observation observation;
  std::array<double, 3> position{}, velocity{};
  double age{0}, horizon{0}, variance{0};
  bool valid{false};
  std::string reason;
};
// Source/reference time uses ROS nanoseconds; receipt/progress watchdog uses
// steady nanoseconds. Never subtract across these distinct clock domains.
class Predictor
{
public:
  explicit Predictor(Options options = {})
  : options_(options) {options_.validate();}
  size_t size() const {return tracks_.size();}
  bool observe(const Frame & frame, int64_t ros_now, int64_t steady_now)
  {
    if (!clock(ros_now, steady_now)) {return reject();}
    const double age = seconds(ros_now - frame.stamp_ns);
    if (!frame.well_formed || frame.frame != "odom" || frame.stamp_ns <= 0 ||
      frame.stamp_ns <= last_frame_stamp_ || age < 0 || age > options_.max_age ||
      frame.observations.size() > options_.max_tracks) {return reject();}
    std::set<int32_t> ids;
    for (const auto & o : frame.observations) {
      if (o.id < 0 || !ids.insert(o.id).second || o.stamp_ns != frame.stamp_ns ||
        o.frame != "odom" ||
        !o.position_valid || o.class_name.size() > 128 || !std::isfinite(o.confidence) ||
        o.confidence < options_.min_confidence || o.confidence > 1 ||
        !std::all_of(o.position.begin(), o.position.end(), [](double x) {return std::isfinite(x);}))
      {
        return reject();
      }
    }
    last_frame_stamp_ = frame.stamp_ns;
    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
      // Identity disappearance or expiration breaks the velocity history.
      if (!ids.count(it->first) || expired(it->second, ros_now, steady_now)) {
        it = tracks_.erase(it);
      } else {++it;}
    }
    for (const auto & o : frame.observations) {
      auto it = tracks_.find(o.id);
      if (it == tracks_.end() || it->second.observation.class_name != o.class_name) {
        tracks_[o.id] = Track{o, {}, steady_now, false, "warming_up"}; continue;
      }
      auto & track = it->second;
      const double dt = seconds(o.stamp_ns - track.observation.stamp_ns);
      std::array<double, 3> velocity{};
      double distance2 = 0;
      for (size_t axis = 0; axis < 3; ++axis) {
        const double delta = o.position[axis] - track.observation.position[axis];
        distance2 += delta * delta; velocity[axis] = delta / dt;
      }
      const double distance = std::sqrt(distance2);
      const bool bounded = dt > 0 && std::isfinite(distance) && distance <= options_.max_jump &&
        distance / dt <= options_.max_speed;
      track = Track{o, bounded ? velocity : std::array<double, 3>{}, steady_now, bounded,
        bounded ? "constant_velocity_model" : "motion_discontinuity"};
    }
    return true;
  }
  std::vector<Prediction> predict(int64_t ros_now, int64_t steady_now)
  {
    if (!clock(ros_now, steady_now)) {reject();return {};}
    std::vector<Prediction> result;
    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
      const auto & track = it->second;
      Prediction p; p.observation = track.observation; p.velocity = track.velocity;
      p.age = seconds(ros_now - track.observation.stamp_ns); p.horizon = options_.horizon;
      p.valid = track.ready && !expired(track, ros_now, steady_now);
      p.reason = expired(track, ros_now, steady_now) ? "expired" : track.reason;
      const double interval = std::max(0.0, p.age) + p.horizon;
      p.variance = options_.base_variance + options_.variance_growth * interval * interval;
      for (size_t axis = 0; axis < 3; ++axis) {
        p.position[axis] = track.observation.position[axis] + track.velocity[axis] * interval;
      }
      result.push_back(p);
      if (p.reason == "expired") {it = tracks_.erase(it);} else {++it;}
    }
    return result;
  }

private:
  struct Track
  {
    Observation observation;
    std::array<double, 3> velocity;
    int64_t receipt_ns;
    bool ready;
    std::string reason;
  };
  static double seconds(int64_t ns) {return static_cast<double>(ns) * 1e-9;}
  bool expired(const Track & track, int64_t ros_now, int64_t steady_now) const
  {
    const double age = seconds(ros_now - track.observation.stamp_ns);
    const double receipt_age = seconds(steady_now - track.receipt_ns);
    return age < 0 || age > options_.max_age || receipt_age < 0 || receipt_age > options_.max_age;
  }
  bool reject() {tracks_.clear();return false;}
  bool clock(int64_t ros_now, int64_t steady_now)
  {
    if (ros_now <= 0 || steady_now < 0) {tracks_.clear();return false;}
    const bool rollback = last_ros_ > 0 && (ros_now < last_ros_ || steady_now < last_steady_);
    if (rollback) {tracks_.clear();last_frame_stamp_ = 0;}
    if (last_ros_ == 0 || ros_now != last_ros_ || rollback) {last_progress_steady_ = steady_now;}
    last_ros_ = ros_now; last_steady_ = steady_now;
    if (rollback) {return false;}
    if (seconds(steady_now - last_progress_steady_) > options_.max_age) {
      tracks_.clear();return false;
    }
    return true;
  }
  Options options_;
  std::map<int32_t, Track> tracks_;
  int64_t last_frame_stamp_{0}, last_ros_{0}, last_steady_{0}, last_progress_steady_{0};
};
}  // namespace easyfly::motion_prediction
