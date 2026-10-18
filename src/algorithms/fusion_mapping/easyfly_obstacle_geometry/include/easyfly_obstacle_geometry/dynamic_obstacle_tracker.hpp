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

#ifndef EASYFLY_OBSTACLE_GEOMETRY__DYNAMIC_OBSTACLE_TRACKER_HPP_
#define EASYFLY_OBSTACLE_GEOMETRY__DYNAMIC_OBSTACLE_TRACKER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_set>
#include <utility>
#include <vector>

namespace easyfly::obstacle_geometry
{

// This helper deliberately contains no ROS/PCL types.  It is used by the
// semantic-map callback after clustering, and can therefore be exercised with
// deterministic geometry/timestamps without a running ROS graph.
struct DynamicObstacleObservation
{
  double center_x{0.0};
  double center_y{0.0};
  double center_z{0.0};
  double size_x{0.0};
  double size_y{0.0};
  double size_z{0.0};
};

struct DynamicObstacleEstimate
{
  bool valid{false};
  // valid describes the current observation/track, while this flag says that
  // track_id is a message-representable stable ID.  Once the int32 ID space
  // is exhausted, a track remains usable for geometry/velocity but must not
  // receive a wrapped stable ID.
  bool stable_id_valid{false};
  bool is_dynamic{false};
  std::size_t track_id{0};
  double velocity_x{0.0};
  double velocity_y{0.0};
  double velocity_z{0.0};
};

struct DynamicObstaclePredictionPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct DynamicObstaclePredictionEnvelope
{
  bool valid{false};
  double half_x{0.0};
  double half_y{0.0};
  double uncertainty{0.0};
};

// Bounded, frame-level diagnostics.  These counters are intentionally plain
// integers so the ROS node can emit one throttled summary instead of logging
// one line for every cluster.  They are also useful in ROS-free tests when a
// sparse LiDAR frame is rejected by one of the safety gates.
struct DynamicObstacleTrackerStats
{
  std::size_t observations{0};
  std::size_t matched{0};
  std::size_t new_tracks{0};
  std::size_t rejected_by_distance{0};
  std::size_t rejected_by_size{0};
  std::size_t rejected_by_ambiguity{0};
  std::size_t rejected_by_direction{0};
  std::size_t rejected_by_deadband{0};
  std::size_t rejected_by_dt{0};
  std::size_t rejected_by_speed{0};
  // A large current-frame cluster may remain a valid static obstacle box, but
  // it is not a physically bounded target for the short-horizon dynamic
  // predictor.  Keep this counter separate from association shape rejection
  // so runtime diagnostics can distinguish the two safety gates.
  std::size_t rejected_by_bbox{0};
  std::size_t dormant_matches{0};
  std::size_t confirmed_dynamic{0};
  std::size_t prediction_points{0};
};

struct DynamicObstacleTrackerConfig
{
  // Association is in the same world frame as the clustered boxes.  It is a
  // distance gate, not a scene coordinate or an acceptance threshold.
  double association_gate{1.5};
  // Ambiguity is split into independent units: a geometric distance margin
  // in metres and a normalized assignment-cost margin.
  double association_ambiguity_distance{0.10};
  double association_cost_margin{0.10};
  // Sparse spinning-LiDAR boxes can change their vertical extent substantially
  // while the horizontal footprint remains stable.  Association is therefore
  // primarily planar, with an independent bounded Z gate and Z-size ratio.
  double association_z_gate{1.0};
  double association_xy_size_ratio{6.0};
  double association_z_size_ratio{8.0};
  double min_dt{0.03};
  double max_dt{1.5};
  double speed_deadband{0.20};
  double max_speed{5.0};
  double min_direction_cosine{0.60};
  std::size_t min_confirmations{2};
  double velocity_ema_alpha{0.50};
  double min_displacement{0.15};
  std::size_t dormant_max_misses{2};

  double prediction_horizon{1.5};
  std::size_t prediction_steps{8};
  std::size_t max_prediction_points{256};
  // A tiny LiDAR cluster must not collapse to a point obstacle.  The envelope
  // grows linearly for a short bounded horizon, but only for tracks already
  // confirmed dynamic by update().
  double prediction_min_half_x{0.20};
  double prediction_min_half_y{0.20};
  double prediction_uncertainty_growth{0.10};  // metres per second
  double prediction_max_uncertainty{0.50};
  // Dynamic prediction is intentionally narrower than the static obstacle
  // output.  This prevents a large environment/background cluster (for
  // example a 4--7 m wall slice) from becoming a moving swept obstacle while
  // preserving that cluster as a conservative static box downstream.
  double max_dynamic_bbox_size_x{2.5};
  double max_dynamic_bbox_size_y{2.5};
  double max_dynamic_bbox_size_z{2.5};
};

struct DynamicObstacleFrame
{
  // false means the frame timestamp/shape budget was not usable.  The
  // estimates are still returned, but all entries are static so callers can
  // publish a conservative observation without propagating bad velocity.
  bool valid{true};
  std::vector<DynamicObstacleEstimate> estimates;
  DynamicObstacleTrackerStats stats;
};

// ObstacleInfo.obstacle_id is int32, while the tracker deliberately uses a
// size_t internally. Never wrap a large internal id into a valid-looking
// message id. The caller can retain its frame-local fallback when this
// conversion is not representable.
inline bool tryStableObstacleId(const std::size_t track_id, std::int32_t & message_id)
{
  if (track_id > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
    return false;
  }
  message_id = static_cast<std::int32_t>(track_id);
  return true;
}

inline bool tryFiniteFloat(const double value, float & converted)
{
  if (!std::isfinite(value)) {
    return false;
  }
  converted = static_cast<float>(value);
  return std::isfinite(converted);
}

inline bool tryFiniteObstacleBox(
  const double center_x, const double center_y, const double center_z,
  const double size_x, const double size_y, const double size_z,
  float & converted_center_x, float & converted_center_y, float & converted_center_z,
  float & converted_size_x, float & converted_size_y, float & converted_size_z)
{
  if (!tryFiniteFloat(center_x, converted_center_x) ||
    !tryFiniteFloat(center_y, converted_center_y) ||
    !tryFiniteFloat(center_z, converted_center_z) ||
    !tryFiniteFloat(size_x, converted_size_x) ||
    !tryFiniteFloat(size_y, converted_size_y) ||
    !tryFiniteFloat(size_z, converted_size_z))
  {
    return false;
  }
  return converted_size_x >= 0.0F && converted_size_y >= 0.0F && converted_size_z >= 0.0F;
}

// Allocate a stable ID without ever wrapping into an already valid message
// ID.  The final int32 value is usable, but the next allocation is refused.
inline bool allocateStableObstacleId(
  std::size_t & next_id, bool & exhausted, std::size_t & assigned_id)
{
  constexpr std::size_t kMaxMessageId =
    static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max());
  if (exhausted || next_id > kMaxMessageId) {
    exhausted = true;
    return false;
  }
  assigned_id = next_id;
  if (next_id == kMaxMessageId) {
    exhausted = true;
  } else {
    ++next_id;
  }
  return true;
}

// Return the number of prediction points that may be appended without
// exceeding the downstream trusted-cloud bound.  Prediction footprints are
// indivisible; callers must either append the whole requested sweep or omit
// it, never truncate a footprint or let size_t addition wrap.
inline std::size_t boundedCompletePredictionPointCount(
  const std::size_t current_count, const std::size_t total_limit,
  const std::size_t requested_count, const std::size_t footprint_points = 9)
{
  if (footprint_points == 0 || current_count > total_limit ||
    requested_count % footprint_points != 0)
  {
    return 0;
  }
  const std::size_t remaining = total_limit - current_count;
  return requested_count <= remaining ? requested_count : 0;
}

inline double dynamicObservationBoxIoU(
  const DynamicObstacleObservation & first, const DynamicObstacleObservation & second)
{
  if (!std::isfinite(first.center_x) || !std::isfinite(first.center_y) ||
    !std::isfinite(first.center_z) || !std::isfinite(first.size_x) ||
    !std::isfinite(first.size_y) || !std::isfinite(first.size_z) ||
    !std::isfinite(second.center_x) || !std::isfinite(second.center_y) ||
    !std::isfinite(second.center_z) || !std::isfinite(second.size_x) ||
    !std::isfinite(second.size_y) || !std::isfinite(second.size_z) ||
    first.size_x < 0.0 || first.size_y < 0.0 || first.size_z < 0.0 ||
    second.size_x < 0.0 || second.size_y < 0.0 || second.size_z < 0.0)
  {
    return 0.0;
  }
  const auto min_corner = [](const double center, const double size) {
      return center - 0.5 * size;
    };
  const auto max_corner = [](const double center, const double size) {
      return center + 0.5 * size;
    };
  const double overlap_x = std::max(
    0.0, std::min(
      max_corner(first.center_x, first.size_x),
      max_corner(second.center_x, second.size_x)) -
    std::max(
      min_corner(first.center_x, first.size_x),
      min_corner(second.center_x, second.size_x)));
  const double overlap_y = std::max(
    0.0, std::min(
      max_corner(first.center_y, first.size_y),
      max_corner(second.center_y, second.size_y)) -
    std::max(
      min_corner(first.center_y, first.size_y),
      min_corner(second.center_y, second.size_y)));
  const double overlap_z = std::max(
    0.0, std::min(
      max_corner(first.center_z, first.size_z),
      max_corner(second.center_z, second.size_z)) -
    std::max(
      min_corner(first.center_z, first.size_z),
      min_corner(second.center_z, second.size_z)));
  const double intersection = overlap_x * overlap_y * overlap_z;
  const double first_volume = first.size_x * first.size_y * first.size_z;
  const double second_volume = second.size_x * second.size_y * second.size_z;
  const double union_volume = first_volume + second_volume - intersection;
  return std::isfinite(intersection) && std::isfinite(union_volume) &&
         union_volume > 1e-12 ? intersection / union_volume : 0.0;
}

inline bool dynamicObservationSizeCompatible(
  const DynamicObstacleObservation & first, const DynamicObstacleObservation & second,
  const double max_size_ratio = 4.0)
{
  if (!std::isfinite(max_size_ratio) || max_size_ratio < 1.0) {
    return false;
  }
  const std::array<double, 3> first_size{{first.size_x, first.size_y, first.size_z}};
  const std::array<double, 3> second_size{{second.size_x, second.size_y, second.size_z}};
  for (std::size_t i = 0; i < first_size.size(); ++i) {
    if (!std::isfinite(first_size[i]) || !std::isfinite(second_size[i]) ||
      first_size[i] < 0.0 || second_size[i] < 0.0)
    {
      return false;
    }
    const double larger = std::max(first_size[i], second_size[i]);
    const double smaller = std::max(0.05, std::min(first_size[i], second_size[i]));
    if (larger > max_size_ratio * smaller) {
      return false;
    }
  }
  return true;
}

inline double dynamicObservationDistance(
  const DynamicObstacleObservation & first, const DynamicObstacleObservation & second)
{
  const double dx = first.center_x - second.center_x;
  const double dy = first.center_y - second.center_y;
  const double dz = first.center_z - second.center_z;
  const double square = dx * dx + dy * dy + dz * dz;
  return std::isfinite(square) && square >= 0.0 ? std::sqrt(square) :
         std::numeric_limits<double>::infinity();
}

inline double dynamicObservationPlanarDistance(
  const DynamicObstacleObservation & first, const DynamicObstacleObservation & second)
{
  const double dx = first.center_x - second.center_x;
  const double dy = first.center_y - second.center_y;
  const double square = dx * dx + dy * dy;
  return std::isfinite(square) && square >= 0.0 ? std::sqrt(square) :
         std::numeric_limits<double>::infinity();
}

inline double dynamicObservationPlanarIoU(
  const DynamicObstacleObservation & first, const DynamicObstacleObservation & second)
{
  if (!std::isfinite(first.center_x) || !std::isfinite(first.center_y) ||
    !std::isfinite(first.size_x) || !std::isfinite(first.size_y) ||
    !std::isfinite(second.center_x) || !std::isfinite(second.center_y) ||
    !std::isfinite(second.size_x) || !std::isfinite(second.size_y) ||
    first.size_x < 0.0 || first.size_y < 0.0 ||
    second.size_x < 0.0 || second.size_y < 0.0)
  {
    return 0.0;
  }
  const auto min_corner = [](const double center, const double size) {
      return center - 0.5 * size;
    };
  const auto max_corner = [](const double center, const double size) {
      return center + 0.5 * size;
    };
  const double overlap_x = std::max(
    0.0, std::min(
      max_corner(first.center_x, first.size_x),
      max_corner(second.center_x, second.size_x)) -
    std::max(
      min_corner(first.center_x, first.size_x),
      min_corner(second.center_x, second.size_x)));
  const double overlap_y = std::max(
    0.0, std::min(
      max_corner(first.center_y, first.size_y),
      max_corner(second.center_y, second.size_y)) -
    std::max(
      min_corner(first.center_y, first.size_y),
      min_corner(second.center_y, second.size_y)));
  const double intersection = overlap_x * overlap_y;
  const double first_area = first.size_x * first.size_y;
  const double second_area = second.size_x * second.size_y;
  const double union_area = first_area + second_area - intersection;
  return std::isfinite(intersection) && std::isfinite(union_area) &&
         union_area > 1e-12 ? intersection / union_area : 0.0;
}

inline bool dynamicObservationZCompatible(
  const DynamicObstacleObservation & first, const DynamicObstacleObservation & second,
  const double z_gate, const double max_z_size_ratio)
{
  if (!std::isfinite(z_gate) || z_gate <= 0.0 ||
    !std::isfinite(max_z_size_ratio) || max_z_size_ratio < 1.0 ||
    !std::isfinite(first.center_z) || !std::isfinite(second.center_z) ||
    !std::isfinite(first.size_z) || !std::isfinite(second.size_z) ||
    first.size_z < 0.0 || second.size_z < 0.0 ||
    std::abs(first.center_z - second.center_z) > z_gate)
  {
    return false;
  }
  const double larger = std::max(first.size_z, second.size_z);
  const double smaller = std::max(0.05, std::min(first.size_z, second.size_z));
  return larger <= max_z_size_ratio * smaller;
}

inline bool dynamicObservationAssociationCompatible(
  const DynamicObstacleObservation & first, const DynamicObstacleObservation & second,
  const double max_xy_size_ratio, const double z_gate, const double max_z_size_ratio)
{
  if (!std::isfinite(max_xy_size_ratio) || max_xy_size_ratio < 1.0) {
    return false;
  }
  const std::array<double, 2> first_size{{first.size_x, first.size_y}};
  const std::array<double, 2> second_size{{second.size_x, second.size_y}};
  for (std::size_t i = 0; i < first_size.size(); ++i) {
    if (!std::isfinite(first_size[i]) || !std::isfinite(second_size[i]) ||
      first_size[i] < 0.0 || second_size[i] < 0.0)
    {
      return false;
    }
    const double larger = std::max(first_size[i], second_size[i]);
    const double smaller = std::max(0.05, std::min(first_size[i], second_size[i]));
    if (larger > max_xy_size_ratio * smaller) {
      return false;
    }
  }
  return dynamicObservationZCompatible(first, second, z_gate, max_z_size_ratio);
}

// This is a prediction-only gate.  The original observation remains valid
// for the static ObstacleInfo/occupancy path even when it is too large to be
// a plausible single moving target.  Keeping the gate here (rather than
// dropping the cluster during extraction) prevents oversized environment
// clusters from acquiring a dynamic velocity or a swept prediction.
inline bool dynamicObservationWithinPredictionBounds(
  const DynamicObstacleObservation & observation,
  const DynamicObstacleTrackerConfig & config)
{
  if (!std::isfinite(observation.size_x) || !std::isfinite(observation.size_y) ||
    !std::isfinite(observation.size_z) || observation.size_x < 0.0 ||
    observation.size_y < 0.0 || observation.size_z < 0.0 ||
    !std::isfinite(config.max_dynamic_bbox_size_x) ||
    !std::isfinite(config.max_dynamic_bbox_size_y) ||
    !std::isfinite(config.max_dynamic_bbox_size_z) ||
    config.max_dynamic_bbox_size_x <= 0.0 ||
    config.max_dynamic_bbox_size_y <= 0.0 ||
    config.max_dynamic_bbox_size_z <= 0.0)
  {
    return false;
  }
  return observation.size_x <= config.max_dynamic_bbox_size_x &&
         observation.size_y <= config.max_dynamic_bbox_size_y &&
         observation.size_z <= config.max_dynamic_bbox_size_z;
}

// A square Hungarian assignment with explicit unmatched rows/columns.  The
// semantic-map output can change cluster order or shape from frame to frame,
// so matching by output order is unsafe. Keep the assignment deterministic
// and reject locally ambiguous alternatives.
inline std::vector<int> dynamicHungarianAssignment(
  const std::vector<std::vector<double>> & costs)
{
  const std::size_t n = costs.size();
  if (n == 0) {
    return {};
  }
  for (const auto & row : costs) {
    if (row.size() != n) {
      return {};
    }
  }
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);
  for (std::size_t i = 1; i <= n; ++i) {
    p[0] = static_cast<int>(i);
    std::size_t j0 = 0;
    std::vector<double> minv(n + 1, infinity);
    std::vector<bool> used(n + 1, false);
    do {
      used[j0] = true;
      const std::size_t i0 = static_cast<std::size_t>(p[j0]);
      double delta = infinity;
      std::size_t j1 = 0;
      for (std::size_t j = 1; j <= n; ++j) {
        if (used[j]) {
          continue;
        }
        const double candidate = costs[i0 - 1][j - 1] - u[i0] - v[j];
        if (candidate < minv[j]) {
          minv[j] = candidate;
          way[j] = static_cast<int>(j0);
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      if (!std::isfinite(delta)) {
        return {};
      }
      for (std::size_t j = 0; j <= n; ++j) {
        if (used[j]) {
          u[static_cast<std::size_t>(p[j])] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const std::size_t j1 = static_cast<std::size_t>(way[j0]);
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  std::vector<int> assignment(n, -1);
  for (std::size_t j = 1; j <= n; ++j) {
    if (p[j] > 0) {
      assignment[static_cast<std::size_t>(p[j] - 1)] = static_cast<int>(j - 1);
    }
  }
  return assignment;
}

// Match a previous-frame box list to a current-frame observation list without
// allowing one current observation to stamp several previous boxes. The
// assignment is global (not output-order greedy).  Its cost uses the XY
// centre/footprint; Z remains an independent bounded compatibility gate.  A
// close alternative is rejected conservatively.
inline std::vector<int> matchCurrentDynamicObservations(
  const std::vector<DynamicObstacleObservation> & previous_observations,
  const std::vector<DynamicObstacleObservation> & current_observations,
  const std::vector<DynamicObstacleEstimate> & current_estimates,
  const double gate, const double ambiguity_distance = 0.10,
  const double cost_margin = 0.10,
  const double max_size_ratio = 4.0,
  const double z_gate = 1.0,
  const double z_size_ratio = 8.0,
  DynamicObstacleTrackerStats * stats = nullptr)
{
  std::vector<int> matches(previous_observations.size(), -1);
  if (stats != nullptr) {
    *stats = DynamicObstacleTrackerStats{};
    stats->observations = current_observations.size();
  }
  if (!std::isfinite(gate) || gate <= 0.0 ||
    current_observations.size() != current_estimates.size() ||
    previous_observations.size() > 64 || current_observations.size() > 64)
  {
    return matches;
  }
  const std::size_t output_count = previous_observations.size();
  const std::size_t current_count = current_observations.size();
  if (output_count == 0 || current_count == 0) {
    return matches;
  }
  const double safe_ambiguity_distance = std::isfinite(ambiguity_distance) &&
    ambiguity_distance >= 0.0 ? std::min(ambiguity_distance, gate) : 0.10;
  const double safe_cost_margin = std::isfinite(cost_margin) && cost_margin >= 0.0 ?
    std::min(cost_margin, 1.0) : 0.10;
  // A valid gated edge is always cheaper than leaving both observations
  // unmatched.  Keeping this above the bounded shape/distance cost prevents
  // the dummy block from discarding a second, unambiguous match.
  constexpr double kUnmatchedCost = 2.0;
  constexpr double kInvalidCost = 1e6;
  const std::size_t matrix_size = output_count + current_count;
  std::vector<std::vector<double>> costs(
    matrix_size, std::vector<double>(matrix_size, kUnmatchedCost));
  std::vector<std::vector<double>> edge_costs(
    output_count, std::vector<double>(current_count, kInvalidCost));
  std::vector<std::vector<double>> edge_distances(
    output_count, std::vector<double>(
      current_count,
      std::numeric_limits<double>::infinity()));
  for (std::size_t output_index = 0; output_index < output_count; ++output_index) {
    for (std::size_t current_index = 0; current_index < current_count; ++current_index) {
      if (!current_estimates[current_index].valid ||
        !dynamicObservationAssociationCompatible(
          previous_observations[output_index], current_observations[current_index],
          max_size_ratio, z_gate, z_size_ratio))
      {
        continue;
      }
      // Vertical centre/extent is a bounded compatibility gate only.  The
      // cost is deliberately planar because sparse spinning-LiDAR height
      // slices vary much more than the XY footprint of a moving person.
      const double distance = dynamicObservationPlanarDistance(
        previous_observations[output_index], current_observations[current_index]);
      const double iou = dynamicObservationPlanarIoU(
        previous_observations[output_index], current_observations[current_index]);
      if (!std::isfinite(distance) || distance > gate || !std::isfinite(iou)) {
        continue;
      }
      const std::array<double, 3> first_size{{
        previous_observations[output_index].size_x,
        previous_observations[output_index].size_y,
        previous_observations[output_index].size_z}};
      const std::array<double, 3> second_size{{
        current_observations[current_index].size_x,
        current_observations[current_index].size_y,
        current_observations[current_index].size_z}};
      double size_penalty = 0.0;
      for (std::size_t axis = 0; axis < first_size.size(); ++axis) {
        const double larger = std::max(first_size[axis], second_size[axis]);
        const double smaller = std::max(0.05, std::min(first_size[axis], second_size[axis]));
        if (larger > 1e-12) {
          size_penalty += (larger - std::min(larger, smaller)) / larger;
        }
      }
      const double cost = distance / gate + 0.25 * (1.0 - iou) +
        0.05 * size_penalty;
      if (std::isfinite(cost) && cost < kUnmatchedCost) {
        edge_costs[output_index][current_index] = cost;
        edge_distances[output_index][current_index] = distance;
        costs[output_index][current_index] = cost;
      }
    }
  }
  // The lower-right dummy block is free; output/current unmatched assignments
  // use the explicit kUnmatchedCost in the off-diagonal blocks.
  for (std::size_t row = output_count; row < matrix_size; ++row) {
    for (std::size_t col = current_count; col < matrix_size; ++col) {
      costs[row][col] = 0.0;
    }
  }
  const auto assignment = dynamicHungarianAssignment(costs);
  if (assignment.size() != matrix_size) {
    return matches;
  }
  std::vector<bool> ambiguous_output(output_count, false);
  std::vector<bool> ambiguous_current(current_count, false);
  for (std::size_t output_index = 0; output_index < output_count; ++output_index) {
    const int assigned = assignment[output_index];
    if (assigned < 0 || static_cast<std::size_t>(assigned) >= current_count ||
      edge_costs[output_index][static_cast<std::size_t>(assigned)] >= kUnmatchedCost)
    {
      continue;
    }
    const double assigned_cost = edge_costs[output_index][static_cast<std::size_t>(assigned)];
    for (std::size_t current_index = 0; current_index < current_count; ++current_index) {
      if (current_index != static_cast<std::size_t>(assigned) &&
        edge_costs[output_index][current_index] < kUnmatchedCost &&
        (std::abs(
          edge_distances[output_index][current_index] -
          edge_distances[output_index][static_cast<std::size_t>(assigned)]) <=
        safe_ambiguity_distance ||
        std::abs(edge_costs[output_index][current_index] - assigned_cost) <=
        safe_cost_margin))
      {
        ambiguous_output[output_index] = true;
        ambiguous_current[current_index] = true;
      }
    }
  }
  // Apply the same ambiguity rule from the current-observation side.  This is
  // needed when two previous tracks compete for one current box; checking only
  // each previous row would otherwise let the Hungarian tie survive.
  for (std::size_t current_index = 0; current_index < current_count; ++current_index) {
    int assigned_output = -1;
    for (std::size_t output_index = 0; output_index < output_count; ++output_index) {
      if (assignment[output_index] == static_cast<int>(current_index)) {
        assigned_output = static_cast<int>(output_index);
        break;
      }
    }
    if (assigned_output < 0 ||
      edge_costs[static_cast<std::size_t>(assigned_output)][current_index] >= kUnmatchedCost)
    {
      continue;
    }
    const double assigned_cost =
      edge_costs[static_cast<std::size_t>(assigned_output)][current_index];
    const double assigned_distance =
      edge_distances[static_cast<std::size_t>(assigned_output)][current_index];
    for (std::size_t output_index = 0; output_index < output_count; ++output_index) {
      if (output_index == static_cast<std::size_t>(assigned_output) ||
        edge_costs[output_index][current_index] >= kUnmatchedCost)
      {
        continue;
      }
      if (std::abs(edge_distances[output_index][current_index] - assigned_distance) <=
        safe_ambiguity_distance ||
        std::abs(edge_costs[output_index][current_index] - assigned_cost) <= safe_cost_margin)
      {
        ambiguous_output[static_cast<std::size_t>(assigned_output)] = true;
        ambiguous_output[output_index] = true;
        ambiguous_current[current_index] = true;
      }
    }
  }
  if (stats != nullptr) {
    for (const bool ambiguous : ambiguous_current) {
      if (ambiguous) {
        ++stats->rejected_by_ambiguity;
      }
    }
  }
  for (std::size_t output_index = 0; output_index < output_count; ++output_index) {
    const int assigned = assignment[output_index];
    if (assigned >= 0 && static_cast<std::size_t>(assigned) < current_count &&
      edge_costs[output_index][static_cast<std::size_t>(assigned)] < kUnmatchedCost &&
      !ambiguous_output[output_index] &&
      !ambiguous_current[static_cast<std::size_t>(assigned)])
    {
      matches[output_index] = assigned;
      if (stats != nullptr) {
        ++stats->matched;
      }
    }
  }
  if (stats != nullptr) {
    // Count bounded, per-current-observation rejection hints.  A current box
    // may have several failed candidate edges; one counter increment per box
    // keeps the diagnostic useful without making it scale with scene size.
    for (std::size_t current_index = 0; current_index < current_count; ++current_index) {
      bool has_shape_edge = false;
      bool has_distance_edge = false;
      for (std::size_t output_index = 0; output_index < output_count; ++output_index) {
        const auto & previous = previous_observations[output_index];
        const auto & current = current_observations[current_index];
        if (!dynamicObservationAssociationCompatible(
            previous, current, max_size_ratio, z_gate, z_size_ratio))
        {
          continue;
        }
        has_shape_edge = true;
        if (std::isfinite(edge_distances[output_index][current_index]) &&
          edge_distances[output_index][current_index] <= gate)
        {
          has_distance_edge = true;
          break;
        }
      }
      bool matched_current = false;
      for (const int match : matches) {
        if (match == static_cast<int>(current_index)) {
          matched_current = true;
          break;
        }
      }
      if (!matched_current && !ambiguous_current[current_index]) {
        if (!has_shape_edge) {
          ++stats->rejected_by_size;
        } else if (!has_distance_edge) {
          ++stats->rejected_by_distance;
        }
      }
    }
  }
  return matches;
}

// Reserve every representable stable ID before assigning frame-local fallback
// IDs. This guarantees uniqueness even when a malformed tracker result repeats
// an ID or when a stable ID equals an otherwise valid fallback index.
inline std::vector<std::int32_t> assignUniqueObstacleIds(
  const std::vector<std::int32_t> & fallback_ids,
  const std::vector<std::int32_t> & stable_ids,
  const std::vector<bool> & stable_valid)
{
  const std::size_t count = fallback_ids.size();
  std::vector<std::int32_t> result(count, 0);
  std::unordered_set<std::int32_t> reserved;
  for (std::size_t i = 0; i < count && i < stable_ids.size() && i < stable_valid.size(); ++i) {
    if (stable_valid[i] && stable_ids[i] >= 0) {
      reserved.insert(stable_ids[i]);
    }
  }
  std::unordered_set<std::int32_t> used;
  std::int64_t next_fallback = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const bool stable_available = i < stable_ids.size() && i < stable_valid.size() &&
      stable_valid[i] && stable_ids[i] >= 0 && used.count(stable_ids[i]) == 0;
    if (stable_available) {
      result[i] = stable_ids[i];
      used.insert(result[i]);
      continue;
    }
    std::int32_t candidate = (i < fallback_ids.size() && fallback_ids[i] >= 0) ?
      fallback_ids[i] : 0;
    if (used.count(candidate) != 0 || reserved.count(candidate) != 0) {
      while (next_fallback <= std::numeric_limits<std::int32_t>::max() &&
        (used.count(static_cast<std::int32_t>(next_fallback)) != 0 ||
        reserved.count(static_cast<std::int32_t>(next_fallback)) != 0))
      {
        ++next_fallback;
      }
      if (next_fallback > std::numeric_limits<std::int32_t>::max()) {
        return {};
      }
      candidate = static_cast<std::int32_t>(next_fallback++);
    }
    result[i] = candidate;
    used.insert(candidate);
  }
  return result;
}

class DynamicObstacleTracker
{
public:
  static constexpr std::size_t kHardMaxTracks = 64;
  static constexpr std::size_t kHardMaxObservations = 256;
  static constexpr std::size_t kHardMaxPredictionSteps = 128;
  static constexpr std::size_t kHardMaxPredictionPoints = 4096;
  static constexpr std::size_t kPredictionFootprintPoints = 9;
  // Motion evidence is deliberately bounded in time and memory.  A short
  // split/merge burst must not become a moving target merely because its
  // centroid crosses the association gate twice.
  static constexpr double kMotionEvidenceMinSpan = 0.40;
  static constexpr double kMotionEvidenceMaxSpan = 1.00;
  static constexpr std::size_t kMotionEvidenceMaxSamples = 16;
  static constexpr double kMotionShapeTolerance = 1e-6;
  static constexpr double kMotionTimeTolerance = 1e-9;

  explicit DynamicObstacleTracker(const DynamicObstacleTrackerConfig & config = {})
  : config_(sanitize(config))
  {
  }

  void reset()
  {
    tracks_.clear();
    // Keep IDs monotonic across an empty/invalid frame or a timestamp epoch
    // reset.  Reusing an old ID after a gap would let downstream consumers
    // mistake a fresh observation for the old moving object.
  }

  const DynamicObstacleTrackerConfig & config() const {return config_;}

  DynamicObstacleFrame update(
    const double stamp_s, const std::vector<DynamicObstacleObservation> & observations)
  {
    DynamicObstacleFrame result;
    result.estimates.resize(observations.size());
    result.stats.observations = observations.size();

    // A malformed frame must not be allowed to advance a valid track.  The
    // caller can still publish the raw boxes with zero velocity.
    if (!std::isfinite(stamp_s) || stamp_s < 0.0 ||
      observations.size() > kHardMaxObservations)
    {
      reset();
      result.valid = false;
      return result;
    }

    for (const auto & observation : observations) {
      if (!validObservation(observation)) {
        reset();
        result.valid = false;
        return result;
      }
    }

    // An accepted empty/clusterless frame is an explicit observation of no
    // current track.  Keep only a tiny dormant geometry history so a one/two
    // frame LiDAR dropout can be associated deterministically.  No estimate
    // or prediction is emitted during the gap, and a reappearing object must
    // start a fresh velocity-confidence run.
    if (observations.empty()) {
      for (const auto & track : tracks_) {
        if (track.have_stamp && stamp_s <= track.last_stamp) {
          // An empty frame still carries a timestamp contract.  Do not let a
          // duplicate/rollback empty cloud preserve dormant state across a
          // clock epoch.
          reset();
          return result;
        }
      }
      for (auto & track : tracks_) {
        if (++track.misses > config_.dormant_max_misses) {
          track.have_stamp = false;
        }
      }
      tracks_.erase(
        std::remove_if(
          tracks_.begin(), tracks_.end(),
          [](const Track & track) {return !track.have_stamp;}), tracks_.end());
      return result;
    }

    // A backwards/duplicate timestamp invalidates velocity evidence.  Keep
    // this frame as a fresh static baseline instead of carrying a velocity or
    // stable ID through a clock reset.
    for (const auto & track : tracks_) {
      if (track.have_stamp && stamp_s <= track.last_stamp) {
        reset();
        break;
      }
    }

    std::vector<DynamicObstacleObservation> previous_observations;
    previous_observations.reserve(tracks_.size());
    for (const auto & track : tracks_) {
      auto predicted = track.last;
      // A predicted association centre is useful only while the previous
      // frame was present.  During a dormant gap, use the last measured
      // centre: this prevents stale velocity from silently moving a track
      // through empty space.
      if (track.misses == 0 && track.has_velocity) {
        const double elapsed = stamp_s - track.last_stamp;
        if (std::isfinite(elapsed) && elapsed >= config_.min_dt &&
          elapsed <= config_.max_dt)
        {
          predicted.center_x += track.velocity[0] * elapsed;
          predicted.center_y += track.velocity[1] * elapsed;
        }
      }
      previous_observations.push_back(predicted);
    }
    // The validity vector belongs to the current observations.  Previous
    // tracks and current clusters intentionally have different dimensions;
    // passing a previous-sized vector here silently rejected every 2→1 or
    // 1→2 association in the old API contract.
    std::vector<DynamicObstacleEstimate> current_estimates(observations.size());
    for (auto & estimate : current_estimates) {
      estimate.valid = true;
    }
    DynamicObstacleTrackerStats association_stats;
    const auto previous_to_current = matchCurrentDynamicObservations(
      previous_observations, observations, current_estimates,
      config_.association_gate, config_.association_ambiguity_distance,
      config_.association_cost_margin, config_.association_xy_size_ratio,
      config_.association_z_gate, config_.association_z_size_ratio,
      &association_stats);
    result.stats.matched = association_stats.matched;
    result.stats.rejected_by_distance = association_stats.rejected_by_distance;
    result.stats.rejected_by_size = association_stats.rejected_by_size;
    result.stats.rejected_by_ambiguity = association_stats.rejected_by_ambiguity;

    std::vector<int> current_to_previous(observations.size(), -1);
    for (std::size_t previous_index = 0;
      previous_index < previous_to_current.size(); ++previous_index)
    {
      const int current_index = previous_to_current[previous_index];
      if (current_index >= 0 && static_cast<std::size_t>(current_index) < observations.size()) {
        current_to_previous[static_cast<std::size_t>(current_index)] =
          static_cast<int>(previous_index);
      }
    }

    std::vector<Track> next_tracks;
    next_tracks.reserve(std::min(observations.size(), kHardMaxTracks));
    for (std::size_t oi = 0; oi < observations.size(); ++oi) {
      const int previous_index = current_to_previous[oi];
      Track track;
      if (previous_index >= 0 && static_cast<std::size_t>(previous_index) < tracks_.size()) {
        track = tracks_[static_cast<std::size_t>(previous_index)];
        const bool was_dormant = track.misses > 0;
        if (was_dormant) {
          ++result.stats.dormant_matches;
          // Preserve no stable identity across an occlusion.  The dormant
          // track is used only to make the association one-to-one and to
          // avoid a false velocity jump; a fresh ID also prevents downstream
          // consumers from treating a new object as the old moving one.
          track.stable_id_valid = allocateStableObstacleId(
            next_track_id_, stable_id_exhausted_, track.id);
          if (!track.stable_id_valid) {
            track.id = 0;
          }
          track.velocity = {0.0, 0.0, 0.0};
          track.has_velocity = false;
          track.dynamic_confirmed = false;
          track.confirmations = 0;
          track.evidence_displacement = 0.0;
          track.direction_bad_streak = 0;
        }
        if (track.motion_history.empty()) {
          track.motion_history.push_back({track.last_stamp, track.last});
        }
        // A dormant match is useful for one-to-one association only.  Do not
        // turn the displacement across the empty-frame gap into evidence.
        if (was_dormant) {
          resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
        } else {
          const double dt = stamp_s - track.last_stamp;
          auto measured = velocity(observations[oi], track.last, dt);
          // The vertical centroid of a sparse LiDAR slice is not a stable
          // motion signal.  Keep the published/predicted velocity planar; Z
          // is still bounded by the association gate above.
          measured[2] = 0.0;
          const double speed = planarNorm(measured);
          const bool usable_dt = std::isfinite(dt) && dt >= config_.min_dt &&
            dt <= config_.max_dt;
          const bool usable_speed = usable_dt && std::isfinite(speed) &&
            speed <= config_.max_speed;
          const bool expired_history = motionHistoryExpired(track, stamp_s);
          const bool shape_consistent = passesMotionShapeGate(
            track.last, observations[oi], kMotionShapeTolerance);
          const bool direction_inconsistent = track.has_velocity &&
            planarDirectionCosine(track.velocity, measured) < config_.min_direction_cosine;
          if (!usable_dt) {
            ++result.stats.rejected_by_dt;
            resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
          } else if (!usable_speed) {
            ++result.stats.rejected_by_speed;
            resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
          } else if (speed <= config_.speed_deadband) {
            ++result.stats.rejected_by_deadband;
            // A low-speed sample is evidence for a static/uncertain object,
            // not a reason to carry the last moving velocity into prediction.
            resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
          } else if (expired_history) {
            // The bounded window expired even though this individual dt may
            // still be association-valid.  Start a new evidence run here.
            resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
          } else if (!shape_consistent) {
            // A centroid displacement no larger than the projected footprint
            // change can be explained by split/merge geometry, not rigid
            // translation.  Keep the current box and association, but clear
            // all prediction evidence.
            resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
          } else if (direction_inconsistent) {
            ++result.stats.rejected_by_direction;
            // Direction disagreement is a discontinuity, not a sample to
            // accumulate.  Restart from the current static observation.
            resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
          } else {
            const double alpha = config_.velocity_ema_alpha;
            if (!track.has_velocity) {
              track.velocity = measured;
            } else {
              for (std::size_t axis = 0; axis < 3; ++axis) {
                track.velocity[axis] = alpha * measured[axis] +
                  (1.0 - alpha) * track.velocity[axis];
              }
            }
            const bool had_velocity = track.has_velocity;
            const bool valid_ema = std::isfinite(track.velocity[0]) &&
              std::isfinite(track.velocity[1]) && std::isfinite(track.velocity[2]) &&
              planarNorm(track.velocity) <= config_.max_speed;
            if (!valid_ema) {
              ++result.stats.rejected_by_speed;
              resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
            } else {
              appendMotionSample(track, stamp_s, observations[oi]);
              const double span = stamp_s - track.motion_history.front().stamp;
              const double net_dx = observations[oi].center_x -
                track.motion_history.front().observation.center_x;
              const double net_dy = observations[oi].center_y -
                track.motion_history.front().observation.center_y;
              const double net_square = net_dx * net_dx + net_dy * net_dy;
              const double net_displacement = std::isfinite(net_square) &&
                net_square >= 0.0 ? std::sqrt(net_square) :
                std::numeric_limits<double>::infinity();
              const std::array<double, 3> window_velocity{{
                span > 0.0 ? net_dx / span : 0.0,
                span > 0.0 ? net_dy / span : 0.0,
                0.0}};
              const bool endpoint_direction_ok = !had_velocity ||
                planarDirectionCosine(track.velocity, window_velocity) >=
                config_.min_direction_cosine;
              if (!std::isfinite(span) || !std::isfinite(net_displacement) ||
                !endpoint_direction_ok)
              {
                if (std::isfinite(span) && std::isfinite(net_displacement) &&
                  !endpoint_direction_ok)
                {
                  ++result.stats.rejected_by_direction;
                }
                resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
              } else {
                track.evidence_displacement = net_displacement;
                track.has_velocity = true;
                track.direction_bad_streak = 0;
                track.confirmations = std::min(
                  config_.min_confirmations + 1, track.confirmations + 1);
                track.dynamic_confirmed = track.confirmations >= config_.min_confirmations &&
                  span + kMotionTimeTolerance >= kMotionEvidenceMinSpan &&
                  net_displacement >= config_.min_displacement;
              }
            }
          }
        }
        if (!dynamicObservationWithinPredictionBounds(observations[oi], config_)) {
          ++result.stats.rejected_by_bbox;
          // Keep the current box for static occupancy, but never carry motion
          // evidence from an oversized/environmental cluster into prediction.
          resetMotionEvidenceAtSample(track, stamp_s, observations[oi]);
        }
        track.misses = 0;
      } else {
        if (next_tracks.size() >= kHardMaxTracks) {
          continue;
        }
        track.stable_id_valid = allocateStableObstacleId(
          next_track_id_, stable_id_exhausted_, track.id);
        if (!track.stable_id_valid) {
          // Keep the track itself for one-to-one geometry and velocity
          // continuity, but do not expose a wrapped/reused stable ID.
          track.id = 0;
        }
        track.has_velocity = false;
        track.dynamic_confirmed = false;
        track.confirmations = 0;
        track.evidence_displacement = 0.0;
        track.direction_bad_streak = 0;
        track.motion_history.clear();
        track.motion_history.push_back({stamp_s, observations[oi]});
        track.misses = 0;
        if (!dynamicObservationWithinPredictionBounds(observations[oi], config_)) {
          ++result.stats.rejected_by_bbox;
        }
        ++result.stats.new_tracks;
      }
      track.last = observations[oi];
      track.last_stamp = stamp_s;
      track.have_stamp = true;
      next_tracks.push_back(track);
      auto & estimate = result.estimates[oi];
      estimate.valid = true;
      estimate.stable_id_valid = track.stable_id_valid;
      estimate.track_id = next_tracks.back().id;
      estimate.is_dynamic = track.dynamic_confirmed && track.has_velocity;
      if (estimate.is_dynamic) {
        estimate.velocity_x = next_tracks.back().velocity[0];
        estimate.velocity_y = next_tracks.back().velocity[1];
        estimate.velocity_z = next_tracks.back().velocity[2];
      }
      if (estimate.is_dynamic) {
        ++result.stats.confirmed_dynamic;
      }
    }
    tracks_ = std::move(next_tracks);
    return result;
  }

  DynamicObstaclePredictionEnvelope predictionEnvelope(
    const DynamicObstacleObservation & observation, const double time_s) const
  {
    DynamicObstaclePredictionEnvelope result;
    if (!validObservation(observation) ||
      !dynamicObservationWithinPredictionBounds(observation, config_) ||
      !std::isfinite(time_s) || time_s < 0.0)
    {
      return result;
    }
    const double uncertainty = std::min(
      config_.prediction_max_uncertainty,
      config_.prediction_uncertainty_growth * time_s);
    const double half_x = std::max(
      0.5 * observation.size_x,
      config_.prediction_min_half_x) + uncertainty;
    const double half_y = std::max(
      0.5 * observation.size_y,
      config_.prediction_min_half_y) + uncertainty;
    if (!std::isfinite(uncertainty) || !std::isfinite(half_x) ||
      !std::isfinite(half_y))
    {
      return result;
    }
    result.valid = true;
    result.half_x = half_x;
    result.half_y = half_y;
    result.uncertainty = uncertainty;
    return result;
  }

  // Build a bounded forward occupancy sweep for the current frame.  Only
  // confirmed dynamic estimates are accepted; static/ambiguous/invalid
  // estimates produce no points.  The caller may append these points to its
  // trusted nonground cloud.
  std::vector<DynamicObstaclePredictionPoint> predict(
    const std::vector<DynamicObstacleObservation> & observations,
    const std::vector<DynamicObstacleEstimate> & estimates,
    const double reference_x = 0.0, const double reference_y = 0.0,
    const double reference_z = 0.0) const
  {
    std::vector<DynamicObstaclePredictionPoint> points;
    if (observations.size() != estimates.size() ||
      observations.size() > kHardMaxObservations ||
      config_.max_prediction_points == 0 || config_.prediction_steps == 0 ||
      config_.prediction_horizon <= 0.0)
    {
      return points;
    }

    std::vector<std::size_t> dynamic_indices;
    dynamic_indices.reserve(observations.size());
    for (std::size_t i = 0; i < observations.size(); ++i) {
      if (estimates[i].valid && estimates[i].is_dynamic &&
        validObservation(observations[i]) &&
        dynamicObservationWithinPredictionBounds(observations[i], config_) &&
        std::isfinite(estimates[i].velocity_x) &&
        std::isfinite(estimates[i].velocity_y) &&
        std::isfinite(estimates[i].velocity_z) &&
        norm(
          {estimates[i].velocity_x, estimates[i].velocity_y,
            estimates[i].velocity_z}) <= config_.max_speed)
      {
        dynamic_indices.push_back(i);
      }
    }
    if (dynamic_indices.empty()) {
      return points;
    }

    // The tracker has no planner-specific cost map.  Use the current UAV
    // position supplied by the node as a conservative collision proxy, with
    // the origin fallback kept deterministic for ROS-free tests.  Sort near
    // objects first so a tight budget gives every near target one complete
    // footprint before spending another step on a distant target.
    const double safe_reference_x = std::isfinite(reference_x) ? reference_x : 0.0;
    const double safe_reference_y = std::isfinite(reference_y) ? reference_y : 0.0;
    const double safe_reference_z = std::isfinite(reference_z) ? reference_z : 0.0;
    const auto reference_distance = [&](const std::size_t index) {
        const double dx = observations[index].center_x - safe_reference_x;
        const double dy = observations[index].center_y - safe_reference_y;
        const double dz = observations[index].center_z - safe_reference_z;
        const double squared = dx * dx + dy * dy + dz * dz;
        return std::isfinite(squared) && squared >= 0.0 ? squared :
               std::numeric_limits<double>::infinity();
      };
    std::stable_sort(
      dynamic_indices.begin(), dynamic_indices.end(),
      [&](const std::size_t first, const std::size_t second) {
        const double first_distance = reference_distance(first);
        const double second_distance = reference_distance(second);
        if (first_distance != second_distance) {
          return first_distance < second_distance;
        }
        return first < second;
      });

    // A prediction step is a complete nine-point conservative planar
    // cross-section: centre, four axis points and four corners.  Allocate
    // Allocate *steps*, never raw points, so a tight global budget cannot
    // publish a partial footprint. If fewer than nine points are available,
    // the safe result is no prediction rather than an incomplete shape.
    constexpr std::size_t kFootprintPoints = kPredictionFootprintPoints;
    const std::size_t total_budget = std::min(
      config_.max_prediction_points, kHardMaxPredictionPoints);
    const std::size_t total_steps = total_budget / kFootprintPoints;
    if (total_steps == 0) {
      return points;
    }
    const std::size_t track_count = dynamic_indices.size();
    const std::size_t temporal_steps = std::min(
      config_.prediction_steps, kHardMaxPredictionSteps);
    const std::size_t base_steps = total_steps / track_count;
    const std::size_t remainder = total_steps % track_count;
    for (std::size_t rank = 0; rank < dynamic_indices.size(); ++rank) {
      const std::size_t track_steps = std::min(
        std::min(config_.prediction_steps, kHardMaxPredictionSteps),
        base_steps + (rank < remainder ? 1 : 0));
      if (track_steps == 0) {
        continue;
      }
      const auto & observation = observations[dynamic_indices[rank]];
      const auto & estimate = estimates[dynamic_indices[rank]];
      for (std::size_t step = 1; step <= track_steps; ++step) {
        // The first allocated steps are intentionally near the current
        // observation.  With a tight budget do not spend the only complete
        // footprint at the horizon endpoint and leave the near path open.
        const double t = config_.prediction_horizon *
          static_cast<double>(step) / static_cast<double>(temporal_steps);
        const double cx = observation.center_x + estimate.velocity_x * t;
        const double cy = observation.center_y + estimate.velocity_y * t;
        const double cz = observation.center_z + estimate.velocity_z * t;
        if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(cz)) {
          return {};
        }
        const auto step_envelope = predictionEnvelope(observation, t);
        if (!step_envelope.valid) {
          return {};
        }
        const std::array<std::array<double, 3>, kFootprintPoints> step_footprint{{
          {{0.0, 0.0, 0.0}},
          {{-step_envelope.half_x, 0.0, 0.0}},
          {{step_envelope.half_x, 0.0, 0.0}},
          {{0.0, -step_envelope.half_y, 0.0}},
          {{0.0, step_envelope.half_y, 0.0}},
          {{-step_envelope.half_x, -step_envelope.half_y, 0.0}},
          {{-step_envelope.half_x, step_envelope.half_y, 0.0}},
          {{step_envelope.half_x, -step_envelope.half_y, 0.0}},
          {{step_envelope.half_x, step_envelope.half_y, 0.0}}}};
        for (std::size_t pi = 0; pi < step_footprint.size(); ++pi) {
          DynamicObstaclePredictionPoint point;
          point.x = cx + step_footprint[pi][0];
          point.y = cy + step_footprint[pi][1];
          point.z = cz;
          if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
          {
            return {};
          }
          points.push_back(point);
        }
      }
    }
    return points;
  }

private:
  struct MotionSample
  {
    double stamp{0.0};
    DynamicObstacleObservation observation;
  };

  struct Track
  {
    std::size_t id{0};
    DynamicObstacleObservation last;
    std::array<double, 3> velocity{{0.0, 0.0, 0.0}};
    double last_stamp{0.0};
    std::size_t confirmations{0};
    double evidence_displacement{0.0};
    std::size_t direction_bad_streak{0};
    std::size_t misses{0};
    std::vector<MotionSample> motion_history;
    bool have_stamp{false};
    bool has_velocity{false};
    bool dynamic_confirmed{false};
    bool stable_id_valid{false};
  };

  static void clearMotionEvidence(Track & track)
  {
    track.velocity = {0.0, 0.0, 0.0};
    track.has_velocity = false;
    track.dynamic_confirmed = false;
    track.confirmations = 0;
    track.evidence_displacement = 0.0;
    track.direction_bad_streak = 0;
    track.motion_history.clear();
  }

  static void resetMotionEvidenceAtSample(
    Track & track, const double stamp, const DynamicObstacleObservation & observation)
  {
    clearMotionEvidence(track);
    track.motion_history.push_back({stamp, observation});
  }

  static bool motionHistoryExpired(const Track & track, const double stamp)
  {
    if (track.motion_history.empty()) {
      return true;
    }
    // Expire only a gap since the newest sample.  The oldest sample is
    // intentionally allowed to slide out below, otherwise steady motion
    // would be reset every time a one-second window advanced.
    const double gap = stamp - track.motion_history.back().stamp;
    return !std::isfinite(gap) || gap > kMotionEvidenceMaxSpan + kMotionTimeTolerance;
  }

  static void appendMotionSample(
    Track & track, const double stamp, const DynamicObstacleObservation & observation)
  {
    track.motion_history.push_back({stamp, observation});
    while (track.motion_history.size() > 1u &&
      stamp - track.motion_history.front().stamp > kMotionEvidenceMaxSpan)
    {
      track.motion_history.erase(track.motion_history.begin());
    }
    while (track.motion_history.size() > kMotionEvidenceMaxSamples) {
      track.motion_history.erase(track.motion_history.begin());
    }
  }

  static bool passesMotionShapeGate(
    const DynamicObstacleObservation & previous,
    const DynamicObstacleObservation & current,
    const double tolerance)
  {
    const double dx = current.center_x - previous.center_x;
    const double dy = current.center_y - previous.center_y;
    const double displacement_square = dx * dx + dy * dy;
    if (!std::isfinite(displacement_square) || displacement_square <= 0.0) {
      return false;
    }
    const double displacement = std::sqrt(displacement_square);
    const double ux = dx / displacement;
    const double uy = dy / displacement;
    const double previous_half_width =
      std::abs(ux) * previous.size_x * 0.5 + std::abs(uy) * previous.size_y * 0.5;
    const double current_half_width =
      std::abs(ux) * current.size_x * 0.5 + std::abs(uy) * current.size_y * 0.5;
    const double delta_half_width = current_half_width - previous_half_width;
    if (!std::isfinite(displacement) || !std::isfinite(delta_half_width)) {
      return false;
    }
    return displacement > std::abs(delta_half_width) + tolerance;
  }

  static DynamicObstacleTrackerConfig sanitize(DynamicObstacleTrackerConfig config)
  {
    if (!std::isfinite(config.association_gate) || config.association_gate <= 0.0) {
      config.association_gate = 1.5;
    }
    config.association_gate = std::min(config.association_gate, 20.0);
    if (!std::isfinite(config.association_ambiguity_distance) ||
      config.association_ambiguity_distance < 0.0)
    {
      config.association_ambiguity_distance = 0.10;
    }
    config.association_ambiguity_distance = std::min(
      config.association_ambiguity_distance, config.association_gate);
    if (!std::isfinite(config.association_cost_margin) ||
      config.association_cost_margin < 0.0)
    {
      config.association_cost_margin = 0.10;
    }
    config.association_cost_margin = std::min(config.association_cost_margin, 1.0);
    if (!std::isfinite(config.association_z_gate) || config.association_z_gate <= 0.0) {
      config.association_z_gate = 1.0;
    }
    config.association_z_gate = std::min(config.association_z_gate, 5.0);
    if (!std::isfinite(config.association_xy_size_ratio) ||
      config.association_xy_size_ratio < 1.0)
    {
      config.association_xy_size_ratio = 6.0;
    }
    config.association_xy_size_ratio = std::min(config.association_xy_size_ratio, 20.0);
    if (!std::isfinite(config.association_z_size_ratio) ||
      config.association_z_size_ratio < 1.0)
    {
      config.association_z_size_ratio = 8.0;
    }
    config.association_z_size_ratio = std::min(config.association_z_size_ratio, 40.0);
    if (!std::isfinite(config.min_dt) || config.min_dt <= 0.0) {config.min_dt = 0.03;}
    if (!std::isfinite(config.max_dt) || config.max_dt < config.min_dt) {config.max_dt = 1.5;}
    config.max_dt = std::min(config.max_dt, 60.0);
    if (!std::isfinite(config.speed_deadband) || config.speed_deadband < 0.0) {
      config.speed_deadband = 0.20;
    }
    if (!std::isfinite(config.max_speed) || config.max_speed <= config.speed_deadband) {
      config.max_speed = 5.0;
    }
    config.max_speed = std::min(config.max_speed, 100.0);
    if (!std::isfinite(config.min_direction_cosine) ||
      config.min_direction_cosine <= 0.0)
    {
      // A non-positive threshold would permit a full reversal (cosine -1)
      // and is therefore never a safe configuration.
      config.min_direction_cosine = 0.60;
    }
    config.min_direction_cosine = std::clamp(config.min_direction_cosine, 0.0, 1.0);
    config.min_confirmations = std::clamp<std::size_t>(config.min_confirmations, 1, 32);
    if (!std::isfinite(config.velocity_ema_alpha) ||
      config.velocity_ema_alpha <= 0.0 || config.velocity_ema_alpha > 1.0)
    {
      config.velocity_ema_alpha = 0.50;
    }
    if (!std::isfinite(config.min_displacement) || config.min_displacement <= 0.0) {
      config.min_displacement = 0.15;
    }
    config.min_displacement = std::min(config.min_displacement, 5.0);
    config.dormant_max_misses = std::min<std::size_t>(config.dormant_max_misses, 2);
    if (!std::isfinite(config.prediction_horizon) || config.prediction_horizon <= 0.0) {
      config.prediction_horizon = 1.5;
    }
    config.prediction_horizon = std::min(config.prediction_horizon, 10.0);
    config.prediction_steps = std::clamp<std::size_t>(
      config.prediction_steps, 1, kHardMaxPredictionSteps);
    config.max_prediction_points = std::min(
      config.max_prediction_points, kHardMaxPredictionPoints);
    if (!std::isfinite(config.prediction_min_half_x) ||
      config.prediction_min_half_x <= 0.0)
    {
      config.prediction_min_half_x = 0.20;
    }
    config.prediction_min_half_x = std::min(config.prediction_min_half_x, 5.0);
    if (!std::isfinite(config.prediction_min_half_y) ||
      config.prediction_min_half_y <= 0.0)
    {
      config.prediction_min_half_y = 0.20;
    }
    config.prediction_min_half_y = std::min(config.prediction_min_half_y, 5.0);
    if (!std::isfinite(config.prediction_uncertainty_growth) ||
      config.prediction_uncertainty_growth < 0.0)
    {
      config.prediction_uncertainty_growth = 0.10;
    }
    config.prediction_uncertainty_growth =
      std::min(config.prediction_uncertainty_growth, 5.0);
    if (!std::isfinite(config.prediction_max_uncertainty) ||
      config.prediction_max_uncertainty < 0.0)
    {
      config.prediction_max_uncertainty = 0.50;
    }
    config.prediction_max_uncertainty = std::min(config.prediction_max_uncertainty, 5.0);
    if (!std::isfinite(config.max_dynamic_bbox_size_x) ||
      config.max_dynamic_bbox_size_x <= 0.0)
    {
      config.max_dynamic_bbox_size_x = 2.5;
    }
    if (!std::isfinite(config.max_dynamic_bbox_size_y) ||
      config.max_dynamic_bbox_size_y <= 0.0)
    {
      config.max_dynamic_bbox_size_y = 2.5;
    }
    if (!std::isfinite(config.max_dynamic_bbox_size_z) ||
      config.max_dynamic_bbox_size_z <= 0.0)
    {
      config.max_dynamic_bbox_size_z = 2.5;
    }
    // Keep this predictor-only gate finite and no looser than the static
    // cluster hard cap.  A deployment may choose a smaller bound, but a
    // malformed YAML value cannot silently turn the dynamic path into a
    // whole-scene motion predictor.
    config.max_dynamic_bbox_size_x = std::min(config.max_dynamic_bbox_size_x, 8.0);
    config.max_dynamic_bbox_size_y = std::min(config.max_dynamic_bbox_size_y, 8.0);
    config.max_dynamic_bbox_size_z = std::min(config.max_dynamic_bbox_size_z, 8.0);
    return config;
  }

  static bool validObservation(const DynamicObstacleObservation & observation)
  {
    return std::isfinite(observation.center_x) && std::isfinite(observation.center_y) &&
           std::isfinite(observation.center_z) && std::isfinite(observation.size_x) &&
           std::isfinite(observation.size_y) && std::isfinite(observation.size_z) &&
           observation.size_x >= 0.0 && observation.size_y >= 0.0 &&
           observation.size_z >= 0.0;
  }

  static double distance3d(
    const DynamicObstacleObservation & first, const DynamicObstacleObservation & second)
  {
    const double dx = first.center_x - second.center_x;
    const double dy = first.center_y - second.center_y;
    const double dz = first.center_z - second.center_z;
    const double square = dx * dx + dy * dy + dz * dz;
    return std::isfinite(square) && square >= 0.0 ? std::sqrt(square) :
           std::numeric_limits<double>::infinity();
  }

  static std::array<double, 3> velocity(
    const DynamicObstacleObservation & current,
    const DynamicObstacleObservation & previous, const double dt)
  {
    if (!std::isfinite(dt) || dt <= 0.0) {
      return {0.0, 0.0, 0.0};
    }
    return {{
      (current.center_x - previous.center_x) / dt,
      (current.center_y - previous.center_y) / dt,
      (current.center_z - previous.center_z) / dt}};
  }

  static double norm(const std::array<double, 3> & value)
  {
    const double square = value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
    return std::isfinite(square) && square >= 0.0 ? std::sqrt(square) :
           std::numeric_limits<double>::infinity();
  }

  static double planarNorm(const std::array<double, 3> & value)
  {
    const double square = value[0] * value[0] + value[1] * value[1];
    return std::isfinite(square) && square >= 0.0 ? std::sqrt(square) :
           std::numeric_limits<double>::infinity();
  }

  static double directionCosine(
    const std::array<double, 3> & first, const std::array<double, 3> & second)
  {
    const double first_norm = norm(first);
    const double second_norm = norm(second);
    if (!std::isfinite(first_norm) || !std::isfinite(second_norm) ||
      first_norm <= 0.0 || second_norm <= 0.0)
    {
      return -1.0;
    }
    const double dot = first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
    return std::isfinite(dot) ? dot / (first_norm * second_norm) : -1.0;
  }

  static double planarDirectionCosine(
    const std::array<double, 3> & first, const std::array<double, 3> & second)
  {
    const double first_norm = planarNorm(first);
    const double second_norm = planarNorm(second);
    if (!std::isfinite(first_norm) || !std::isfinite(second_norm) ||
      first_norm <= 0.0 || second_norm <= 0.0)
    {
      return -1.0;
    }
    const double dot = first[0] * second[0] + first[1] * second[1];
    return std::isfinite(dot) ? dot / (first_norm * second_norm) : -1.0;
  }

  DynamicObstacleTrackerConfig config_;
  std::vector<Track> tracks_;
  std::size_t next_track_id_{0};
  bool stable_id_exhausted_{false};
};

}  // namespace easyfly::obstacle_geometry

#endif  // EASYFLY_OBSTACLE_GEOMETRY__DYNAMIC_OBSTACLE_TRACKER_HPP_
