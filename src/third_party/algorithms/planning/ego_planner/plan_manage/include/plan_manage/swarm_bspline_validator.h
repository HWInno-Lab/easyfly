#ifndef PLAN_MANAGE__SWARM_BSPLINE_VALIDATOR_H_
#define PLAN_MANAGE__SWARM_BSPLINE_VALIDATOR_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <Eigen/Core>
#include <limits>
#include <vector>

namespace ego_planner
{
namespace swarm_validation
{

// These limits are deliberately independent of a received vector length.
// They are applied before any size_t conversion, Eigen allocation, or knot
// indexing in the ROS callbacks.
constexpr int kMaxSwarmId = 64;
constexpr int kMinBsplineOrder = 1;
constexpr int kMaxBsplineOrder = 7;
constexpr std::size_t kMaxControlPoints = 4096;
constexpr std::size_t kMaxKnots =
    kMaxControlPoints + static_cast<std::size_t>(kMaxBsplineOrder) + 1;
constexpr std::size_t kMaxSwarmTrajectories =
    static_cast<std::size_t>(kMaxSwarmId) + 1;
constexpr double kMaxTrajectoryDuration = 300.0;
// Keep ingress behaviour deterministic when a malformed parameter reaches a
// callback.  The horizon is a distance in the planner's local world frame;
// callers still apply their own distance gate after this common sanity check.
constexpr double kMaxPlanningHorizon = 1000.0;
// The planner's world/odom operating envelope is bounded to one kilometre.
// This is intentionally much larger than the SITL scene while preventing
// finite-but-physical-impossible coordinates from reaching Eigen arithmetic.
constexpr double kMaxControlPointAbs = 1000.0;
constexpr double kMaxDerivedVelocityAbs = 10000.0;
constexpr double kMaxDerivedAccelerationAbs = 100000.0;
constexpr int kConsumedBsplineOrder = 3;
constexpr std::size_t kConsumedMinControlPoints = 4;
constexpr std::int64_t kMaxTrajectoryId =
    static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max());

enum class ValidationError
{
  kOk,
  kDroneId,
  kOrder,
  kTrajectoryId,
  kStartTime,
  kControlPointsEmpty,
  kControlPointsTooLarge,
  kControlPointsTooShort,
  kControlPointNonFinite,
  kControlPointOutOfRange,
  kKnotsEmpty,
  kKnotsTooLarge,
  kKnotCountMismatch,
  kKnotNonFinite,
  kKnotNotStrictlyIncreasing,
  kInternalKnotRepeat,
  kDerivativeDenominator,
  kDerivativeNonFinite,
  kDerivativeOutOfRange,
  kInvalidDuration,
  kYawCountMismatch,
  kYawTooLarge,
  kYawNonFinite,
  kYawDt,
  kMultiEmpty,
  kMultiTooLarge,
  kMultiDroneId
};

struct ValidationResult
{
  ValidationError error{ValidationError::kOk};
  double duration{0.0};

  bool valid() const { return error == ValidationError::kOk; }
};

inline const char * validationErrorName(const ValidationError error)
{
  switch (error)
  {
    case ValidationError::kOk: return "ok";
    case ValidationError::kDroneId: return "drone_id";
    case ValidationError::kOrder: return "order";
    case ValidationError::kTrajectoryId: return "traj_id";
    case ValidationError::kStartTime: return "start_time";
    case ValidationError::kControlPointsEmpty: return "control_points_empty";
    case ValidationError::kControlPointsTooLarge: return "control_points_too_large";
    case ValidationError::kControlPointsTooShort: return "control_points_too_short";
    case ValidationError::kControlPointNonFinite: return "control_point_nonfinite";
    case ValidationError::kControlPointOutOfRange: return "control_point_out_of_range";
    case ValidationError::kKnotsEmpty: return "knots_empty";
    case ValidationError::kKnotsTooLarge: return "knots_too_large";
    case ValidationError::kKnotCountMismatch: return "knot_count_mismatch";
    case ValidationError::kKnotNonFinite: return "knot_nonfinite";
    case ValidationError::kKnotNotStrictlyIncreasing: return "knot_not_strictly_increasing";
    case ValidationError::kInternalKnotRepeat: return "internal_knot_repeat";
    case ValidationError::kDerivativeDenominator: return "derivative_denominator";
    case ValidationError::kDerivativeNonFinite: return "derivative_nonfinite";
    case ValidationError::kDerivativeOutOfRange: return "derivative_out_of_range";
    case ValidationError::kInvalidDuration: return "invalid_duration";
    case ValidationError::kYawCountMismatch: return "yaw_count_mismatch";
    case ValidationError::kYawTooLarge: return "yaw_too_large";
    case ValidationError::kYawNonFinite: return "yaw_nonfinite";
    case ValidationError::kYawDt: return "yaw_dt";
    case ValidationError::kMultiEmpty: return "multi_empty";
    case ValidationError::kMultiTooLarge: return "multi_too_large";
    case ValidationError::kMultiDroneId: return "multi_drone_id";
  }
  return "unknown";
}

inline bool validPlanningHorizon(const double horizon) noexcept
{
  return std::isfinite(horizon) && horizon > 0.0 &&
         horizon <= kMaxPlanningHorizon;
}

template<typename BsplineMessage>
ValidationResult validateBspline(const BsplineMessage & msg)
{
  if (msg.drone_id < 0 || msg.drone_id > kMaxSwarmId)
  {
    return {ValidationError::kDroneId, 0.0};
  }
  if (msg.order < kMinBsplineOrder || msg.order > kMaxBsplineOrder)
  {
    return {ValidationError::kOrder, 0.0};
  }
  if (msg.traj_id <= 0 || msg.traj_id > kMaxTrajectoryId)
  {
    return {ValidationError::kTrajectoryId, 0.0};
  }
  if (msg.start_time.sec < 0 || msg.start_time.nanosec >= 1000000000u ||
      (msg.start_time.sec == 0 && msg.start_time.nanosec == 0))
  {
    return {ValidationError::kStartTime, 0.0};
  }

  const std::size_t point_count = msg.pos_pts.size();
  if (point_count == 0)
  {
    return {ValidationError::kControlPointsEmpty, 0.0};
  }
  if (point_count > kMaxControlPoints)
  {
    return {ValidationError::kControlPointsTooLarge, 0.0};
  }
  if (point_count < static_cast<std::size_t>(msg.order + 1))
  {
    return {ValidationError::kControlPointsTooShort, 0.0};
  }

  const std::size_t knot_count = msg.knots.size();
  if (knot_count == 0)
  {
    return {ValidationError::kKnotsEmpty, 0.0};
  }
  if (knot_count > kMaxKnots)
  {
    return {ValidationError::kKnotsTooLarge, 0.0};
  }
  const std::size_t expected_knot_count =
      point_count + static_cast<std::size_t>(msg.order) + 1;
  if (knot_count != expected_knot_count)
  {
    return {ValidationError::kKnotCountMismatch, 0.0};
  }

  for (const auto & point : msg.pos_pts)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
    {
      return {ValidationError::kControlPointNonFinite, 0.0};
    }
    if (std::abs(point.x) > kMaxControlPointAbs ||
        std::abs(point.y) > kMaxControlPointAbs ||
        std::abs(point.z) > kMaxControlPointAbs)
    {
      return {ValidationError::kControlPointOutOfRange, 0.0};
    }
  }

  for (std::size_t i = 0; i < knot_count; ++i)
  {
    if (!std::isfinite(msg.knots[i]))
    {
      return {ValidationError::kKnotNonFinite, 0.0};
    }
    if (i > 0 && msg.knots[i] < msg.knots[i - 1])
    {
      return {ValidationError::kKnotNotStrictlyIncreasing, 0.0};
    }
  }

  // Repeated knots are valid for a non-uniform B-spline.  The local
  // derivative implementation only requires its actual denominators to be
  // positive; checking those denominators also keeps a zero repeated span
  // from reaching Eigen/B-spline evaluation.
  for (std::size_t i = 0; i + 1 < point_count; ++i)
  {
    const std::size_t denominator_index =
        i + static_cast<std::size_t>(msg.order) + 1;
    if (denominator_index >= knot_count ||
        msg.knots[denominator_index] <= msg.knots[i + 1])
    {
      return {ValidationError::kInvalidDuration, 0.0};
    }
  }

  if (msg.yaw_pts.size() > kMaxControlPoints)
  {
    return {ValidationError::kYawTooLarge, 0.0};
  }
  for (const auto yaw : msg.yaw_pts)
  {
    if (!std::isfinite(yaw))
    {
      return {ValidationError::kYawNonFinite, 0.0};
    }
  }
  if (!msg.yaw_pts.empty() && msg.yaw_pts.size() != point_count)
  {
    return {ValidationError::kYawCountMismatch, 0.0};
  }
  if (!std::isfinite(msg.yaw_dt) ||
      (!msg.yaw_pts.empty() && msg.yaw_dt <= 0.0))
  {
    return {ValidationError::kYawDt, 0.0};
  }

  // UniformBspline::getTimeSpan() uses [order, knot_count-order-1].
  const double domain_start = msg.knots[static_cast<std::size_t>(msg.order)];
  const double domain_end =
      msg.knots[knot_count - static_cast<std::size_t>(msg.order) - 1];
  const double duration = domain_end - domain_start;
  if (!std::isfinite(duration) || duration <= 0.0 ||
      duration > kMaxTrajectoryDuration)
  {
    return {ValidationError::kInvalidDuration, 0.0};
  }

  return {ValidationError::kOk, duration};
}

template<typename BsplineMessage>
ValidationResult validateCubicBspline(const BsplineMessage & msg)
{
  const ValidationResult basic = validateBspline(msg);
  if (!basic.valid())
  {
    return basic;
  }
  if (msg.order != kConsumedBsplineOrder)
  {
    return {ValidationError::kOrder, 0.0};
  }
  if (msg.pos_pts.size() < kConsumedMinControlPoints)
  {
    return {ValidationError::kControlPointsTooShort, 0.0};
  }

  const std::size_t point_count = msg.pos_pts.size();
  const std::size_t knot_count = msg.knots.size();
  const std::size_t order = static_cast<std::size_t>(kConsumedBsplineOrder);
  const double domain_start = msg.knots[order];
  const double domain_end = msg.knots[knot_count - order - 1];

  // UniformBspline's de Boor implementation does not guard an internal
  // repeated knot.  Boundary clamping is safe when the derivative checks
  // below pass; an internal repeat is rejected before it can yield 0/0 in a
  // later sample.
  for (std::size_t i = 1; i < knot_count; ++i)
  {
    if (msg.knots[i] == msg.knots[i - 1] &&
        msg.knots[i] > domain_start && msg.knots[i] < domain_end)
    {
      return {ValidationError::kInternalKnotRepeat, 0.0};
    }
  }

  // Validate the actual first- and second-derivative denominators used by
  // UniformBspline::getDerivative(), and bound the resulting control points.
  double previous_velocity[3] = {0.0, 0.0, 0.0};
  bool have_previous_velocity = false;
  for (std::size_t i = 0; i + 1 < point_count; ++i)
  {
    const double denominator = msg.knots[i + order + 1] - msg.knots[i + 1];
    if (!std::isfinite(denominator) || denominator <= 0.0)
    {
      return {ValidationError::kDerivativeDenominator, 0.0};
    }

    double velocity[3] = {0.0, 0.0, 0.0};
    const double coordinates[2][3] = {
      {msg.pos_pts[i].x, msg.pos_pts[i].y, msg.pos_pts[i].z},
      {msg.pos_pts[i + 1].x, msg.pos_pts[i + 1].y, msg.pos_pts[i + 1].z}};
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      velocity[axis] = static_cast<double>(kConsumedBsplineOrder) *
        (coordinates[1][axis] - coordinates[0][axis]) / denominator;
      if (!std::isfinite(velocity[axis]))
      {
        return {ValidationError::kDerivativeNonFinite, 0.0};
      }
      if (std::abs(velocity[axis]) > kMaxDerivedVelocityAbs)
      {
        return {ValidationError::kDerivativeOutOfRange, 0.0};
      }
    }

    if (have_previous_velocity)
    {
      const double acceleration_denominator =
          msg.knots[i + order] - msg.knots[i + 1];
      if (!std::isfinite(acceleration_denominator) ||
          acceleration_denominator <= 0.0)
      {
        return {ValidationError::kDerivativeDenominator, 0.0};
      }
      for (std::size_t axis = 0; axis < 3; ++axis)
      {
        const double acceleration =
            2.0 * (velocity[axis] - previous_velocity[axis]) /
            acceleration_denominator;
        if (!std::isfinite(acceleration))
        {
          return {ValidationError::kDerivativeNonFinite, 0.0};
        }
        if (std::abs(acceleration) > kMaxDerivedAccelerationAbs)
        {
          return {ValidationError::kDerivativeOutOfRange, 0.0};
        }
      }
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      previous_velocity[axis] = velocity[axis];
    }
    have_previous_velocity = true;
  }

  return basic;
}

template<typename MultiMessage>
ValidationResult validateMultiBsplines(const MultiMessage & msg)
{
  if (msg.drone_id_from < 0 || msg.drone_id_from > kMaxSwarmId)
  {
    return {ValidationError::kMultiDroneId, 0.0};
  }
  if (msg.traj.empty())
  {
    return {ValidationError::kMultiEmpty, 0.0};
  }
  if (msg.traj.size() > kMaxSwarmTrajectories)
  {
    return {ValidationError::kMultiTooLarge, 0.0};
  }

  const std::size_t expected_trajectory_count =
    static_cast<std::size_t>(msg.drone_id_from) + 1U;
  if (msg.traj.size() != expected_trajectory_count)
  {
    return {ValidationError::kMultiDroneId, 0.0};
  }

  for (std::size_t index = 0; index < msg.traj.size(); ++index)
  {
    // MultiBsplines is an indexed array, and the downstream swarm buffer is
    // indexed the same way.  Do not silently replace identity with position:
    // swapped, duplicate, missing, or out-of-range IDs are rejected before
    // any candidate buffer or message snapshot can be changed.
    if (msg.traj[index].drone_id != static_cast<int32_t>(index))
    {
      return {ValidationError::kMultiDroneId, 0.0};
    }
    const auto & trajectory = msg.traj[index];
    const ValidationResult result = validateCubicBspline(trajectory);
    if (!result.valid())
    {
      return result;
    }
  }
  return {ValidationError::kOk, 0.0};
}

inline bool finiteAndBounded(const Eigen::MatrixXd & values, const double bound)
{
  return values.size() > 0 && values.allFinite() && values.cwiseAbs().maxCoeff() <= bound;
}

inline bool finiteAndBounded(const Eigen::VectorXd & values, const double bound)
{
  return values.size() > 0 && values.allFinite() && values.cwiseAbs().maxCoeff() <= bound;
}

// Commit is deliberately the only helper that may resize/assign a swarm
// buffer.  Callers build and validate a complete candidate first; a rejected
// candidate leaves both the old slot and the vector size untouched.
template<typename Buffer, typename Candidate, typename BlankFactory>
bool commitSwarmCandidate(
    Buffer & buffer, const std::size_t id, const bool candidate_ready,
    const Candidate & candidate, BlankFactory make_blank)
{
  if (!candidate_ready || id > static_cast<std::size_t>(kMaxSwarmId))
  {
    return false;
  }
  if (buffer.size() <= id)
  {
    const std::size_t old_size = buffer.size();
    buffer.resize(id + 1);
    for (std::size_t i = old_size; i <= id; ++i)
    {
      buffer[i] = make_blank();
    }
  }
  buffer[id] = candidate;
  return true;
}

}  // namespace swarm_validation
}  // namespace ego_planner

#endif  // PLAN_MANAGE__SWARM_BSPLINE_VALIDATOR_H_
