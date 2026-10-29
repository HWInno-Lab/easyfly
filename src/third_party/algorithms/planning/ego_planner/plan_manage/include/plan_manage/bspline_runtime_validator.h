// Copyright 2026 EasyFly Project
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

#ifndef PLAN_MANAGE__BSPLINE_RUNTIME_VALIDATOR_H_
#define PLAN_MANAGE__BSPLINE_RUNTIME_VALIDATOR_H_

#include <bspline_opt/uniform_bspline.h>
#include <plan_manage/swarm_bspline_validator.h>
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ego_planner
{
namespace swarm_validation
{

enum class RuntimeValidationError
{
  kOk,
  kPositionStructure,
  kVelocityStructure,
  kAccelerationStructure,
  kPositionKnotStructure,
  kVelocityKnotStructure,
  kAccelerationKnotStructure,
  kPositionNonFinite,
  kVelocityNonFinite,
  kAccelerationNonFinite,
  kPositionKnotInvalid,
  kVelocityKnotInvalid,
  kAccelerationKnotInvalid,
  kDuration,
  kPositionSample,
  kVelocitySample,
  kAccelerationSample
};

struct RuntimeValidationResult
{
  RuntimeValidationError error {RuntimeValidationError::kOk};

  bool valid() const {return error == RuntimeValidationError::kOk;}
};

inline const char * runtimeValidationErrorName(const RuntimeValidationError error)
{
  switch (error) {
    case RuntimeValidationError::kOk: return "ok";
    case RuntimeValidationError::kPositionStructure: return "position_structure";
    case RuntimeValidationError::kVelocityStructure: return "velocity_structure";
    case RuntimeValidationError::kAccelerationStructure: return "acceleration_structure";
    case RuntimeValidationError::kPositionKnotStructure: return "position_knot_structure";
    case RuntimeValidationError::kVelocityKnotStructure: return "velocity_knot_structure";
    case RuntimeValidationError::kAccelerationKnotStructure: return
        "acceleration_knot_structure";
    case RuntimeValidationError::kPositionNonFinite: return "position_nonfinite";
    case RuntimeValidationError::kVelocityNonFinite: return "velocity_nonfinite";
    case RuntimeValidationError::kAccelerationNonFinite: return "acceleration_nonfinite";
    case RuntimeValidationError::kPositionKnotInvalid: return "position_knot_invalid";
    case RuntimeValidationError::kVelocityKnotInvalid: return "velocity_knot_invalid";
    case RuntimeValidationError::kAccelerationKnotInvalid: return "acceleration_knot_invalid";
    case RuntimeValidationError::kDuration: return "duration";
    case RuntimeValidationError::kPositionSample: return "position_sample";
    case RuntimeValidationError::kVelocitySample: return "velocity_sample";
    case RuntimeValidationError::kAccelerationSample: return "acceleration_sample";
  }
  return "unknown";
}

inline bool validRuntimeKnotVector(
  const Eigen::VectorXd & knots,
  const std::size_t expected_size)
{
  if (knots.size() != static_cast<Eigen::Index>(expected_size) || knots.size() == 0 ||
    !knots.allFinite())
  {
    return false;
  }
  for (Eigen::Index i = 1; i < knots.size(); ++i) {
    if (knots(i) < knots(i - 1)) {
      return false;
    }
  }
  return true;
}

inline RuntimeValidationResult validateCubicRuntimeTrajectory(
  UniformBspline & position, UniformBspline & velocity, UniformBspline & acceleration,
  const std::size_t expected_control_points, const std::size_t expected_knots,
  const double expected_duration)
{
  const Eigen::MatrixXd position_points = position.getControlPoint();
  const Eigen::MatrixXd velocity_points = velocity.getControlPoint();
  const Eigen::MatrixXd acceleration_points = acceleration.getControlPoint();
  if (position_points.rows() != 3 ||
    position_points.cols() != static_cast<Eigen::Index>(expected_control_points))
  {
    return {RuntimeValidationError::kPositionStructure};
  }
  if (velocity_points.rows() != 3 ||
    velocity_points.cols() != static_cast<Eigen::Index>(expected_control_points - 1))
  {
    return {RuntimeValidationError::kVelocityStructure};
  }
  if (acceleration_points.rows() != 3 ||
    acceleration_points.cols() != static_cast<Eigen::Index>(expected_control_points - 2))
  {
    return {RuntimeValidationError::kAccelerationStructure};
  }

  if (!validRuntimeKnotVector(position.getKnot(), expected_knots)) {
    return {RuntimeValidationError::kPositionKnotStructure};
  }
  if (!validRuntimeKnotVector(velocity.getKnot(), expected_knots - 2)) {
    return {RuntimeValidationError::kVelocityKnotStructure};
  }
  if (!validRuntimeKnotVector(acceleration.getKnot(), expected_knots - 4)) {
    return {RuntimeValidationError::kAccelerationKnotStructure};
  }

  if (!finiteAndBounded(position_points, kMaxControlPointAbs)) {
    return {RuntimeValidationError::kPositionNonFinite};
  }
  if (!finiteAndBounded(velocity_points, kMaxDerivedVelocityAbs)) {
    return {RuntimeValidationError::kVelocityNonFinite};
  }
  if (!finiteAndBounded(acceleration_points, kMaxDerivedAccelerationAbs)) {
    return {RuntimeValidationError::kAccelerationNonFinite};
  }

  const double position_duration = position.getTimeSum();
  const double velocity_duration = velocity.getTimeSum();
  const double acceleration_duration = acceleration.getTimeSum();
  const double duration_tolerance = 1.0e-9 * std::max(1.0, expected_duration);
  if (!std::isfinite(position_duration) || !std::isfinite(velocity_duration) ||
    !std::isfinite(acceleration_duration) || position_duration <= 0.0 ||
    velocity_duration <= 0.0 || acceleration_duration <= 0.0 ||
    std::abs(position_duration - expected_duration) > duration_tolerance ||
    std::abs(velocity_duration - expected_duration) > duration_tolerance ||
    std::abs(acceleration_duration - expected_duration) > duration_tolerance)
  {
    return {RuntimeValidationError::kDuration};
  }

  const double sample_times[3] = {0.0, position_duration * 0.5, position_duration};
  for (const double sample_time : sample_times) {
    const Eigen::VectorXd position_sample = position.evaluateDeBoorT(sample_time);
    if (!finiteAndBounded(position_sample, kMaxControlPointAbs)) {
      return {RuntimeValidationError::kPositionSample};
    }
    const Eigen::VectorXd velocity_sample = velocity.evaluateDeBoorT(sample_time);
    if (!finiteAndBounded(velocity_sample, kMaxDerivedVelocityAbs)) {
      return {RuntimeValidationError::kVelocitySample};
    }
    const Eigen::VectorXd acceleration_sample = acceleration.evaluateDeBoorT(sample_time);
    if (!finiteAndBounded(acceleration_sample, kMaxDerivedAccelerationAbs)) {
      return {RuntimeValidationError::kAccelerationSample};
    }
  }
  return {RuntimeValidationError::kOk};
}

}    // namespace swarm_validation
}  // namespace ego_planner

#endif  // PLAN_MANAGE__BSPLINE_RUNTIME_VALIDATOR_H_
