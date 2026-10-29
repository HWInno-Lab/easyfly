// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <cmath>

namespace easyfly_orbslam3
{
using Information15 = Eigen::Matrix<double, 15, 15>;
using Covariance6 = Eigen::Matrix<double, 6, 6>;

// Native tangent order is right-local IMU rotation, right-local IMU position,
// world velocity and biases. Marginalize velocity/bias uncertainty by inverting
// the full information, then move the pose covariance to fixed map axes and
// the physical base origin. This is CONDITIONAL on the current map/calibration.
inline bool conditionalBaseCovariance(
  const Information15 & information, const Eigen::Matrix3d & rotation,
  const Eigen::Vector3d & imu_to_base, Covariance6 & covariance)
{
  if (!information.allFinite() || !rotation.allFinite() || !imu_to_base.allFinite() ||
    (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-4 ||
    std::abs(rotation.determinant() - 1.) > 1e-4 ||
    (information - information.transpose()).norm() > 1e-8 * information.norm())
  {return false;}
  const Information15 symmetric = (information + information.transpose()) * .5;
  Eigen::SelfAdjointEigenSolver<Information15> spectrum(symmetric);
  if (spectrum.info() != Eigen::Success || spectrum.eigenvalues().minCoeff() <= 1e-9 ||
    spectrum.eigenvalues().maxCoeff() > 1e14 * spectrum.eigenvalues().minCoeff())
  {return false;}
  Eigen::LLT<Information15> factor(symmetric);
  if (factor.info() != Eigen::Success) {return false;}
  const Information15 full = factor.solve(Information15::Identity());
  Eigen::Matrix3d cross;
  cross << 0., -imu_to_base.z(), imu_to_base.y(), imu_to_base.z(), 0.,
    -imu_to_base.x(), -imu_to_base.y(), imu_to_base.x(), 0.;
  Covariance6 jacobian = Covariance6::Zero();
  jacobian.topLeftCorner<3, 3>() = -rotation * cross;
  jacobian.topRightCorner<3, 3>() = rotation;
  jacobian.bottomLeftCorner<3, 3>() = rotation;
  covariance = jacobian * full.topLeftCorner<6, 6>() * jacobian.transpose();
  covariance = (covariance + covariance.transpose()).eval() * .5;
  return covariance.allFinite() && Eigen::LLT<Covariance6>(covariance).info() == Eigen::Success;
}
}  // namespace easyfly_orbslam3
