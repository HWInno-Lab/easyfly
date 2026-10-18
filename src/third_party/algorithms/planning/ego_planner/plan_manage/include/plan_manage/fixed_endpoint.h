#pragma once
#include <Eigen/Core>
#include <cmath>
namespace ego_planner {
inline bool setFixedCubicEndpoint(Eigen::MatrixXd &points, double interval,
                                 const Eigen::Vector3d &target,
                                 const Eigen::Vector3d &velocity) {
  if (points.rows() != 3 || points.cols() < 7 || !points.allFinite() ||
      !std::isfinite(interval) || interval <= 0.0 ||
      !target.allFinite() || !velocity.allFinite()) return false;
  const Eigen::Index end = points.cols();
  points.col(end - 3) = target - interval * velocity;
  points.col(end - 2) = target;
  points.col(end - 1) = target + interval * velocity;
  return points.allFinite();
}
}  // namespace ego_planner
