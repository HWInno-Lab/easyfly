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

#ifndef EASYFLY_PRECISION_LANDING__POLICY_HPP_
#define EASYFLY_PRECISION_LANDING__POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>
#include <limits>
#include <string>

namespace easyfly_precision_landing
{
struct Config
{
  double horizontal_speed{std::numeric_limits<double>::quiet_NaN()};
  double descent_speed{std::numeric_limits<double>::quiet_NaN()};
  double align_tolerance{std::numeric_limits<double>::quiet_NaN()};
  double handoff_height{std::numeric_limits<double>::quiet_NaN()};
  double descent_floor_height{std::numeric_limits<double>::quiet_NaN()};
  double max_height{std::numeric_limits<double>::quiet_NaN()};
  double max_radius{std::numeric_limits<double>::quiet_NaN()};
  double plane_z{std::numeric_limits<double>::quiet_NaN()};
  double plane_tolerance{0.15};
  double tilt_limit{0.175};
  double target_jump{0.25};
  double target_max_age{0.3};
  double acquire_timeout{5.0};
  double lost_timeout{2.0};
  int settle_observations{5};
  bool moving_target{false};
  double moving_max_speed{0.06};
  double moving_fit_error{0.04};
  double moving_land_timeout{8.0};
  // Declared pad clearance after aircraft footprint; not tag edge size.
  double moving_usable_radius{0.65};

  bool valid() const
  {
    const double positive[] = {
      horizontal_speed, descent_speed, align_tolerance, handoff_height, descent_floor_height,
      max_height, max_radius, plane_tolerance, tilt_limit, target_jump,
      target_max_age, acquire_timeout, lost_timeout};
    for (const auto value : positive) {
      if (!std::isfinite(value) || value <= 0) {return false;}
    }
    if (moving_target && (!std::isfinite(moving_max_speed) ||
      !std::isfinite(moving_fit_error) || !std::isfinite(moving_land_timeout) ||
      !std::isfinite(moving_usable_radius) || moving_max_speed <= 0 || moving_max_speed > 0.1 ||
      moving_fit_error <= 0 || moving_fit_error > 0.05 ||
      moving_land_timeout < 2 || moving_land_timeout > 10 ||
      moving_usable_radius > 1.0 || moving_usable_radius <=
      align_tolerance + 2 * moving_fit_error + moving_max_speed * moving_land_timeout))
    {return false;}
    return std::isfinite(plane_z) && horizontal_speed <= 1.0 && descent_speed <= 0.5 &&
           align_tolerance <= 0.3 && handoff_height >= 0.15 && handoff_height <= 1.0 &&
           descent_floor_height >= 0.15 && descent_floor_height < handoff_height &&
           max_height > handoff_height && max_height <= 10.0 && max_radius <= 5.0 &&
           tilt_limit < 0.7853981634 && target_max_age <= 1.0 &&
           acquire_timeout <= 30.0 && lost_timeout <= 10.0 &&
           settle_observations >= 2 && settle_observations <= 100;
  }
};

struct Pose
{
  double x{0}, y{0}, z{0};
};

struct Observation
{
  // Caller must check frame, ID, source time and steady receipt freshness.
  bool valid{false};
  int64_t stamp_ns{0};
  double x{0}, y{0}, z{0};
  double normal_x{0}, normal_y{0}, normal_z{0};
};

struct Decision
{
  std::string phase{"acquire"};
  std::string reason;
  double x{0}, y{0}, z{0};
  double horizontal_error{0}, height_above_target{0};
  bool target_valid{false}, handoff{false}, failed{false};
};

// Pure policy. The flight controller owns admission, cancellation, state safety,
// actual outputs and Land. Nothing here can publish or reclaim control.
class Policy
{
public:
  explicit Policy(const Config & config)
  : config_(config) {}

  void start(const Pose & pose, double steady_seconds)
  {
    anchor_ = pose;
    command_ = pose;
    started_ = steady_seconds;
    lost_since_ = 0;
    watermark_ = 0;
    settled_ = 0;
    reacquired_ = 0;
    observed_ = false;
    losing_ = false;
    have_board_ = false;
    descending_ = false;
    motion_.clear();
    motion_ready_ = false;
    motion_failed_ = false;
  }

  Decision step(
    const Pose & pose, const Observation & observation, double steady_seconds,
    double dt)
  {
    Decision out;
    out.x = command_.x;
    out.y = command_.y;
    out.z = command_.z;
    if (!config_.valid() || !std::isfinite(steady_seconds) || steady_seconds < started_ ||
      !finite(pose) || !std::isfinite(dt) || dt <= 0 ||
      std::hypot(pose.x - anchor_.x, pose.y - anchor_.y) > config_.max_radius ||
      pose.z - config_.plane_z > config_.max_height || pose.z < config_.plane_z)
    {
      out.failed = true;
      out.phase = "failed";
      out.reason = "invalid_configuration_or_flight_boundary";
      return out;
    }
    if (config_.moving_target && observation.valid && observation.stamp_ns > watermark_ &&
      finite({observation.x, observation.y, observation.z}))
    {
      observe_motion(observation);
    }
    if (motion_failed_) {
      out.failed = true;
      out.phase = "failed";
      out.reason = "moving_target_exceeds_declared_motion_profile";
      return out;
    }
    const bool good = observation.valid && observation.stamp_ns > 0 &&
      observation.stamp_ns >= watermark_ &&
      finite({observation.x, observation.y, observation.z}) &&
      std::isfinite(observation.normal_x) && std::isfinite(observation.normal_y) &&
      std::isfinite(observation.normal_z) &&
      std::abs(
      observation.normal_x * observation.normal_x +
      observation.normal_y * observation.normal_y +
      observation.normal_z * observation.normal_z - 1.0) <= 1e-4 &&
      std::abs(observation.normal_z) >= std::cos(config_.tilt_limit) &&
      std::abs(observation.z - config_.plane_z) <= config_.plane_tolerance &&
      std::hypot(observation.x - anchor_.x, observation.y - anchor_.y) <= config_.max_radius &&
      (config_.moving_target || !have_board_ ||
      std::hypot(observation.x - board_.x, observation.y - board_.y) <=
      config_.target_jump);
    if (!good) {
      if (!losing_) {
        // Stop immediately at the trusted measured position, not a descending
        // target below it. The caller already ensures OFFBOARD is still owned.
        command_ = pose;
        descending_ = false;
        losing_ = true;
        lost_since_ = steady_seconds;
      }
      settled_ = 0;
      reacquired_ = 0;
      out.phase = observed_ ? "lost_hold" : "acquire";
      out.x = command_.x;
      out.y = command_.y;
      out.z = command_.z;
      const bool expired = observed_ ? steady_seconds - lost_since_ >= config_.lost_timeout :
        steady_seconds - started_ >= config_.acquire_timeout;
      if (expired) {
        out.failed = true;
        out.phase = "failed";
        out.reason = observed_ ? "landing_target_lost" : "landing_target_acquisition_timeout";
      }
      return out;
    }
    const bool had_observation = observed_;
    observed_ = true;
    out.target_valid = true;
    out.horizontal_error = std::hypot(pose.x - observation.x, pose.y - observation.y);
    out.height_above_target = pose.z - observation.z;
    if (out.height_above_target < 0 || out.height_above_target > config_.max_height) {
      out.failed = true;
      out.phase = "failed";
      out.reason = "invalid_target_relative_height";
      return out;
    }
    if (losing_ && had_observation) {
      if (observation.stamp_ns > watermark_) {
        watermark_ = observation.stamp_ns;
        ++reacquired_;
      }
      // A single good frame does not terminate a loss episode or move its
      // captured hold point. Both counting and timeout use independent evidence.
      if (steady_seconds - lost_since_ >= config_.lost_timeout) {
        out.failed = true;
        out.phase = "failed";
        out.reason = "landing_target_reacquisition_timeout";
        return out;
      }
      if (reacquired_ < 5) {
        out.phase = "reacquire_hold";
        return out;
      }
      settled_ = 0;
      reacquired_ = 0;
    }
    losing_ = false;
    if (observation.stamp_ns > watermark_) {
      watermark_ = observation.stamp_ns;
      if (!have_board_) {
        board_ = {observation.x, observation.y, observation.z};
        have_board_ = true;
      }
      // Board anchor is never moved: small successive jumps cannot accumulate
      // into a moving landing target in this static-board first version.
      if (out.horizontal_error <= config_.align_tolerance) {++settled_;} else {settled_ = 0;}
    }
    if (out.horizontal_error > config_.align_tolerance) {settled_ = 0;}
    dt = std::min(dt, 0.1);
    const double dx = observation.x - command_.x, dy = observation.y - command_.y;
    const double distance = std::hypot(dx, dy);
    const double step = std::min(distance, config_.horizontal_speed * dt);
    if (distance > 0) {
      command_.x += dx * step / distance;
      command_.y += dy * step / distance;
    }
    const bool aligned = settled_ >= config_.settle_observations &&
      (!config_.moving_target || motion_ready_);
    out.phase = aligned ? "descend" : "align";
    if (aligned && out.height_above_target <= config_.handoff_height) {
      out.handoff = true;
      out.phase = "land_handoff";
      command_ = pose;
    } else if (aligned) {
      descending_ = true;
      // The command floor is below the handoff gate, allowing measured height
      // to enter the explicit band without relying on tracking-error overshoot.
      command_.z = std::max(
        observation.z + config_.descent_floor_height,
        command_.z - config_.descent_speed * dt);
    } else {
      // Capture once when interrupting descent; otherwise retain the established
      // hold altitude. Recopying measured Z every tick can chase a transient drop.
      if (descending_) {command_.z = pose.z;}
      descending_ = false;
    }
    out.x = command_.x;
    out.y = command_.y;
    out.z = command_.z;
    return out;
  }

private:
  void observe_motion(const Observation & observation)
  {
    const double stamp = static_cast<double>(observation.stamp_ns) * 1e-9;
    if (!motion_.empty() && stamp - motion_.back().first > config_.target_max_age) {
      motion_.clear();
      motion_ready_ = false;
    }
    motion_.emplace_back(stamp, Pose{observation.x, observation.y, observation.z});
    while (motion_.size() > 128 ||
      (motion_.size() > 2 && stamp - motion_.front().first > 1.0))
    {motion_.pop_front();}
    if (motion_.size() < 8 || stamp - motion_.front().first < 0.7) {return;}
    double t = 0, x = 0, y = 0;
    const double origin = motion_.front().first;
    for (const auto & row : motion_) {
      t += row.first - origin;
      x += row.second.x;
      y += row.second.y;
    }
    const double count = static_cast<double>(motion_.size());
    t /= count; x /= count; y /= count;
    double denominator = 0, vx = 0, vy = 0;
    for (const auto & row : motion_) {
      const double dt = row.first - origin - t;
      denominator += dt * dt;
      vx += dt * (row.second.x - x);
      vy += dt * (row.second.y - y);
    }
    vx /= denominator; vy /= denominator;
    if (std::hypot(vx, vy) > config_.moving_max_speed) {motion_failed_ = true;}
    for (const auto & row : motion_) {
      const double dt = row.first - origin - t;
      if (std::hypot(row.second.x - x - vx * dt, row.second.y - y - vy * dt) >
        config_.moving_fit_error) {motion_failed_ = true;}
    }
    motion_ready_ = !motion_failed_;
  }

  std::deque<std::pair<double, Pose>> motion_;
  bool motion_ready_{false}, motion_failed_{false};
  static bool finite(const Pose & pose)
  {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z);
  }

  Config config_;
  Pose anchor_, command_, board_;
  double started_{0}, lost_since_{0};
  int64_t watermark_{0};
  int settled_{0};
  int reacquired_{0};
  bool observed_{false}, losing_{false}, have_board_{false};
  bool descending_{false};
};
}  // namespace easyfly_precision_landing
#endif  // EASYFLY_PRECISION_LANDING__POLICY_HPP_
