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
//
// Ground-plane acceptance and tracking for semantic_map.
//
// The tracker deliberately works on a complete, normalized plane model rather
// than a single world-frame z value.  This keeps the policy independent of the
// absolute origin of the odometry frame and makes it possible to reject a
// single-frame RANSAC plane which is close to the vehicle (for example a sparse
// horizontal return at the aircraft height).

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace easyfly::ground
{

struct GroundPlaneModel
{
  // Normalized plane equation: normal dot p + offset = 0.
  // The normal is always oriented upwards (normal[2] > 0).
  std::array<double, 3> normal{{0.0, 0.0, 1.0}};
  double offset{0.0};

  bool valid() const
  {
    const double norm = std::sqrt(
      normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    return std::isfinite(offset) && std::isfinite(normal[0]) &&
           std::isfinite(normal[1]) && std::isfinite(normal[2]) &&
           norm > std::numeric_limits<double>::epsilon() && normal[2] > 0.0;
  }

  double zAt(const double x, const double y) const
  {
    if (!valid() || std::abs(normal[2]) < 1e-9) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return -(normal[0] * x + normal[1] * y + offset) / normal[2];
  }

  double signedDistance(const double x, const double y, const double z) const
  {
    return normal[0] * x + normal[1] * y + normal[2] * z + offset;
  }
};

struct GroundSpatialSupport
{
  std::size_t points{0};
  std::size_t cells{0};
  std::size_t eligible_cells{0};
  std::array<std::size_t, 8> sector_cells{};

  bool qualified() const
  {
    return points >= 200 && cells >= 120 && eligible_cells >= cells &&
           eligible_cells > 0 && static_cast<double>(cells) / eligible_cells >= 0.90 &&
           *std::min_element(sector_cells.begin(), sector_cells.end()) >= 5;
  }
};

struct GroundPlaneCandidate
{
  GroundPlaneModel model;
  std::size_t inlier_count{0};
  std::size_t sample_count{0};
  GroundSpatialSupport spatial_support;
};

struct GroundVehiclePose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  // A fresh tracker may only be initialized from the existing flight-state
  // bridge: fresh connected+pose_valid and disarmed.  This is a system
  // assumption that disarmed grants permission for ground initialization; it
  // deliberately does not depend on GPS validity or relative altitude, which
  // are not guaranteed to be bridged in GPS-denied deployments.  An armed
  // in-air restart fails closed.  Detecting an in-air disarm is a P2 concern
  // that needs landed/contact evidence; G7 does not extend this message.
  bool connected{false};
  bool pose_valid{false};
  bool armed{true};
  double state_age_s{std::numeric_limits<double>::infinity()};
};

enum class GroundPlaneUpdate
{
  kRejected,
  kPending,
  kInitialized,
  kAccepted,
  // A trusted model is deliberately held while the aircraft is armed or the
  // UAV-state bridge is disconnected/stale.  A RANSAC candidate is not
  // evidence that the ground moved, and fusing it without a valid state can
  // make the plane drift through the vehicle/obstacles.
  kFrozen,
};

class GroundPlaneTracker
{
public:
  struct Params
  {
    // Candidate quality and geometry.
    double max_tilt_rad{0.15};
    std::size_t min_inliers{50};
    double min_inlier_ratio{0.15};
    // Explicit alternative for indoor landed observations. The original
    // full-frame ratio remains unchanged and is the default policy.
    bool spatial_support_enabled{false};
    // Direct landed traces show 0.053-0.054 m between base_link and the
    // fitted ground.  Keep a 0.03 m positive floor, leaving about 0.02 m of
    // measured margin while rejecting zero/near-body-height planes.
    double min_ground_clearance{0.03};
    double max_ground_distance_below_vehicle{8.0};

    // Temporal consistency.
    int init_consecutive_frames{3};
    double init_state_timeout{1.0};
    double init_max_z_difference{0.25};
    double init_max_normal_angle_rad{0.12};
    double max_innovation_z{0.45};
    double max_innovation_angle_rad{0.15};
    double smoothing_alpha{0.25};
  };

  GroundPlaneTracker()
  : GroundPlaneTracker(Params{})
  {
  }

  explicit GroundPlaneTracker(const Params & params)
  : params_(params)
  {
    params_.init_consecutive_frames = std::max(1, params_.init_consecutive_frames);
    params_.init_state_timeout = std::max(0.0, params_.init_state_timeout);
    params_.smoothing_alpha = std::clamp(params_.smoothing_alpha, 0.0, 1.0);
  }

  GroundPlaneUpdate update(
    const GroundPlaneCandidate & candidate,
    const GroundVehiclePose & vehicle)
  {
    if (model_valid_ && (!vehicleStateFresh(vehicle) || vehicle.armed)) {
      // Keep the trusted model and discard any pending candidate while in
      // flight or while the vehicle-state bridge is disconnected/stale.  A
      // candidate is never allowed to update the model without a fresh,
      // connected, valid pose.  Returning a distinct status prevents callers
      // from reporting an expected fail-closed freeze as a rejected estimate.
      clearPending();
      return GroundPlaneUpdate::kFrozen;
    }

    if (!candidateIsPhysicallyValid(candidate, vehicle)) {
      clearPending();
      return GroundPlaneUpdate::kRejected;
    }

    if (!model_valid_) {
      if (!pending_valid_ || !consistentAtVehicle(
          pending_model_, candidate.model, vehicle,
          params_.init_max_z_difference, params_.init_max_normal_angle_rad))
      {
        pending_model_ = candidate.model;
        pending_valid_ = true;
        pending_count_ = 1;
      } else {
        pending_model_ = blend(pending_model_, candidate.model, 0.5);
        ++pending_count_;
      }

      const int required_frames = usesSpatialSupport(candidate) ?
        std::max(5, params_.init_consecutive_frames) : params_.init_consecutive_frames;
      if (pending_count_ >= required_frames) {
        model_ = pending_model_;
        model_valid_ = true;
        clearPending();
        return GroundPlaneUpdate::kInitialized;
      }
      return GroundPlaneUpdate::kPending;
    }

    if (consistentAtVehicle(
        model_, candidate.model, vehicle,
        params_.max_innovation_z, params_.max_innovation_angle_rad))
    {
      model_ = blend(model_, candidate.model, params_.smoothing_alpha);
      clearPending();
      return GroundPlaneUpdate::kAccepted;
    }

    // Do not replace the trusted model with an innovation outlier.  Without
    // independent spatial continuity evidence, even a persistent aircraft-
    // height plane is not enough to establish a new ground model.
    clearPending();
    return GroundPlaneUpdate::kRejected;
  }

  bool initialized() const {return model_valid_;}

  const GroundPlaneModel & model() const {return model_;}

  void reset()
  {
    model_valid_ = false;
    clearPending();
  }

  // Classification always uses the trusted model.  It is intentionally
  // separate from candidate update so rejected RANSAC inliers cannot remove
  // obstacle points from the current frame.
  bool isGroundPoint(
    const double x, const double y, const double z,
    const double distance_threshold) const
  {
    return model_valid_ && std::abs(model_.signedDistance(x, y, z)) <= distance_threshold;
  }

  double heightAboveGround(const double x, const double y, const double z) const
  {
    if (!model_valid_) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return z - model_.zAt(x, y);
  }

private:
  bool usesSpatialSupport(const GroundPlaneCandidate & candidate) const
  {
    return candidate.sample_count > 0 &&
           static_cast<double>(candidate.inlier_count) / candidate.sample_count <
           params_.min_inlier_ratio;
  }

  bool vehicleStateFresh(const GroundVehiclePose & vehicle) const
  {
    return vehicle.connected && vehicle.pose_valid &&
           std::isfinite(vehicle.state_age_s) && vehicle.state_age_s >= 0.0 &&
           vehicle.state_age_s <= params_.init_state_timeout;
  }

  static GroundPlaneModel blend(
    const GroundPlaneModel & first, const GroundPlaneModel & second, const double alpha)
  {
    GroundPlaneModel result;
    result.normal = {
      (1.0 - alpha) * first.normal[0] + alpha * second.normal[0],
      (1.0 - alpha) * first.normal[1] + alpha * second.normal[1],
      (1.0 - alpha) * first.normal[2] + alpha * second.normal[2]};
    result.offset = (1.0 - alpha) * first.offset + alpha * second.offset;
    const double norm = std::sqrt(
      result.normal[0] * result.normal[0] + result.normal[1] * result.normal[1] +
      result.normal[2] * result.normal[2]);
    if (norm > std::numeric_limits<double>::epsilon()) {
      for (double & component : result.normal) {component /= norm;}
      result.offset /= norm;
    }
    if (result.normal[2] < 0.0) {
      for (double & component : result.normal) {component = -component;}
      result.offset = -result.offset;
    }
    return result;
  }

  static double normalAngle(
    const GroundPlaneModel & first, const GroundPlaneModel & second)
  {
    double dot = first.normal[0] * second.normal[0] +
      first.normal[1] * second.normal[1] +
      first.normal[2] * second.normal[2];
    dot = std::clamp(dot, -1.0, 1.0);
    return std::acos(dot);
  }

  static bool consistentAtVehicle(
    const GroundPlaneModel & first, const GroundPlaneModel & second,
    const GroundVehiclePose & vehicle, const double max_z_difference,
    const double max_normal_angle)
  {
    const double first_z = first.zAt(vehicle.x, vehicle.y);
    const double second_z = second.zAt(vehicle.x, vehicle.y);
    return first.valid() && second.valid() && std::isfinite(first_z) &&
           std::isfinite(second_z) && std::abs(first_z - second_z) <= max_z_difference &&
           normalAngle(first, second) <= max_normal_angle;
  }

  bool candidateIsPhysicallyValid(
    const GroundPlaneCandidate & candidate, const GroundVehiclePose & vehicle) const
  {
    if (!model_valid_ && (!vehicleStateFresh(vehicle) || vehicle.armed)) {
      return false;
    }
    if (!candidate.model.valid() || candidate.sample_count == 0 ||
      candidate.inlier_count < params_.min_inliers ||
      !std::isfinite(vehicle.x) || !std::isfinite(vehicle.y) ||
      !std::isfinite(vehicle.z) || normalAngle(candidate.model, GroundPlaneModel{}) >
      params_.max_tilt_rad)
    {
      return false;
    }

    const double candidate_z = candidate.model.zAt(vehicle.x, vehicle.y);
    const double clearance = vehicle.z - candidate_z;
    if (usesSpatialSupport(candidate) &&
      (!params_.spatial_support_enabled || !candidate.spatial_support.qualified() ||
      vehicle.armed || !vehicleStateFresh(vehicle) || clearance > 0.25 ||
      normalAngle(candidate.model, GroundPlaneModel{}) > std::min(0.10, params_.max_tilt_rad)))
    {
      return false;
    }
    return std::isfinite(candidate_z) && clearance >= params_.min_ground_clearance &&
           clearance <= params_.max_ground_distance_below_vehicle;
  }

  void clearPending()
  {
    pending_valid_ = false;
    pending_count_ = 0;
  }

  Params params_;
  GroundPlaneModel model_;
  bool model_valid_{false};
  GroundPlaneModel pending_model_;
  bool pending_valid_{false};
  int pending_count_{0};
};

}  // namespace easyfly::ground
