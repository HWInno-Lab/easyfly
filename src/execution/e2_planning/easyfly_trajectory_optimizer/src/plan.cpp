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

#include "easyfly_trajectory_optimizer/plan.hpp"
#include <gcopter/geo_utils.hpp>
#include <gcopter/gcopter.hpp>
#include <fstream>
#include <iomanip>

namespace easyfly::trajectory
{
namespace
{
using Controls = Eigen::Matrix<double, 3, 6>;
constexpr double choose[6][6] =
{{1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1}, {1, 4, 6, 4, 1}, {1, 5, 10, 10, 5, 1}};
Controls bezier(const Piece<5> & p)
{
  Controls b = Controls::Zero();
  for (int k = 0; k < 6; ++k) {
    for (int j = 0; j <= k; ++j) {
      b.col(k) += p.getCoeffMat().col(5 - j) *
        std::pow(p.getDuration(), j) * choose[k][j] / choose[5][j];
    }
  }
  return b;
}
bool clear(
  const Grid & g, const Controls & b, int depth, size_t & nodes,
  Steady::time_point deadline)
{
  budget(deadline);
  if (++nodes > 100000 || !b.allFinite()) {return false;}
  if (g.freeBox(Vec(b.rowwise().minCoeff()), Vec(b.rowwise().maxCoeff()))) {return true;}
  if (depth >= 18 || !g.free(Vec(b.col(0))) || !g.free(Vec(b.col(5)))) {return false;}
  Controls values = b, left, right;
  left.col(0) = b.col(0); right.col(5) = b.col(5);
  for (int level = 1; level < 6; ++level) {
    for (int j = 0; j < 6 - level; ++j) {
      values.col(j) = .5 * (values.col(j) + values.col(j + 1));
    }
    left.col(level) = values.col(0); right.col(5 - level) = values.col(5 - level);
  }
  return clear(g, left, depth + 1, nodes, deadline) && clear(g, right, depth + 1, nodes, deadline);
}
void require(bool good, const char * message) {if (!good) {throw std::runtime_error(message);}}
}
bool collisionFree(
  const Grid & grid, const Trajectory<5> & trajectory, double from,
  Steady::time_point deadline)
{
  if (!grid.valid() || trajectory.getPieceNum() < 1 || !std::isfinite(from)) {return false;}
  size_t nodes = 0; double elapsed = 0;
  for (const auto & piece : trajectory) {
    elapsed += piece.getDuration();
    if (elapsed < from) {continue;}
    // Conservatively include the complete current piece, rather than missing
    // any short suffix by sampling. Future pieces are checked continuously.
    if (!clear(grid, bezier(piece), 0, nodes, deadline)) {return false;}
  }
  return true;
}
Plan optimize(const Request & r, Steady::time_point deadline)
{
  require(
    r.grid.valid() && r.start.allFinite() && r.goal.allFinite() && r.velocity.allFinite(),
    "invalid_request");
  require(
    std::isfinite(r.max_speed) && r.max_speed >= .1 && r.max_speed <= 3. &&
    r.velocity.norm() <= r.max_speed && r.velocity.norm() <= .3 &&
    r.max_acceleration > 0 && r.max_acceleration <= 2. && r.max_jerk > 0 && r.max_jerk <= 8.,
    "invalid_limits");
  Plan out;
  out.route = search(r.grid, r.start, r.goal, deadline);
  out.boxes = corridors(r.grid, out.route, deadline);
  std::vector<Eigen::MatrixX4d> polytopes;
  for (const auto & box : out.boxes) {
    Eigen::MatrixX4d h = Eigen::MatrixX4d::Zero(6, 4);
    for (int axis = 0; axis < 3; ++axis) {
      h(2 * axis, axis) = -1.; h(2 * axis, 3) = box.low[axis];
      h(2 * axis + 1, axis) = 1.; h(2 * axis + 1, 3) = -box.high[axis];
    }
    polytopes.push_back(h);
  }
  Eigen::Matrix3d start, end;
  start << r.start, r.velocity, Vec::Zero(); end << r.goal, Vec::Zero(), Vec::Zero();
  Eigen::VectorXd limits(5), weights(5), physical(6);
  // Reserve tracking margin for the existing PX4 position-control chain.
  // The independently checked request limits remain unchanged.
  weights << 1e5, 1e5, 1e5, 1e5, 1e5;
  physical << 1.535, 9.8, 0., 0., 0., .01;
  // This command interface forwards position and velocity, not acceleration.
  // Reoptimize the timing instead of scaling coefficients (which would change
  // the measured initial velocity). Keep every collision and endpoint check.
  double velocity_target = r.max_speed * .5;
  std::string last_failure = "tracking_dynamics_infeasible";
  for (int attempt = 0; attempt < 8; ++attempt) {
    limits << velocity_target, 1.5, .5, 5., 30.;
    gcopter::GCOPTER_PolytopeSFC optimizer;
    budget(deadline);
    require(
      optimizer.setup(
        1., start, end, polytopes, 2., .01, 16, limits, weights,
        physical), "gcopter_setup");
    out.trajectory.clear();
    out.cost = optimizer.optimize(out.trajectory, 1e-5);
    budget(deadline);
    try {
      validate(r, out, r.grid, deadline);
      if (out.maximum_acceleration <= kTrackingAcceleration &&
        out.maximum_jerk <= kTrackingJerk) {return out;}
      last_failure = "tracking_dynamics_infeasible";
    } catch (const std::runtime_error & error) {
      // Native line-search/numerical failure is not a certified trajectory.
      // Retry the native solve within the SAME attempt and time budgets; never
      // execute, rescale or relax validation of a rejected result.
      last_failure = error.what();
    }
    velocity_target = std::max(r.velocity.norm() * 1.05, velocity_target * .75);
  }
  throw std::runtime_error(last_failure);
}
void validate(const Request & r, Plan & out, const Grid & current_grid, Steady::time_point deadline)
{
  out.maximum_speed = out.maximum_acceleration = out.maximum_jerk = 0.;
  auto & t = out.trajectory;
  require(
    std::isfinite(out.cost) && t.getPieceNum() > 0 && t.getPieceNum() <= 128 &&
    std::isfinite(
      t.getTotalDuration()) && t.getTotalDuration() > .05 && t.getTotalDuration() <= 110.,
    "gcopter_failed");
  Vec last_p = r.start, last_v = r.velocity, last_a = Vec::Zero();
  for (const auto & p : t) {
    require(p.getCoeffMat().allFinite() && p.getDuration() >= .001, "nonfinite_piece");
    require(
      (p.getPos(0) - last_p).norm() < 1e-5 && (p.getVel(0) - last_v).norm() < 1e-4 &&
      (p.getAcc(0) - last_a).norm() < 1e-4, "discontinuous_piece");
    last_p = p.getPos(p.getDuration()); last_v = p.getVel(p.getDuration()); last_a = p.getAcc(
      p.getDuration());
    const double speed = p.getMaxVelRate(), acceleration_bound = p.getMaxAccRate();
    require(std::isfinite(speed) && std::isfinite(acceleration_bound), "nonfinite_dynamic_bounds");
    out.maximum_speed = std::max(out.maximum_speed, speed);
    out.maximum_acceleration = std::max(out.maximum_acceleration, acceleration_bound);
    Controls acceleration = Controls::Zero();
    for (int power = 2; power <= 5; ++power) {
      acceleration.col(7 - power) = p.getCoeffMat().col(5 - power) * power * (power - 1);
    }
    const double jerk = Piece<5>(p.getDuration(), acceleration).getMaxVelRate();
    require(std::isfinite(jerk), "nonfinite_jerk_bound");
    out.maximum_jerk = std::max(out.maximum_jerk, jerk);
  }
  require(
    (last_p - r.goal).norm() < 1e-5 && last_v.norm() < 1e-4 && last_a.norm() < 1e-4,
    "wrong_terminal");
  require(
    std::isfinite(out.maximum_speed) && out.maximum_speed <= r.max_speed &&
    std::isfinite(out.maximum_acceleration) && out.maximum_acceleration <= r.max_acceleration &&
    std::isfinite(out.maximum_jerk) && out.maximum_jerk <= r.max_jerk, "dynamic_bounds");
  require(collisionFree(current_grid, t, 0, deadline), "continuous_collision_or_unknown");
}
void writeRequest(const std::string & file, const Request & r)
{
  std::ofstream f(file);
  f << std::setprecision(17) << "EASYFLY_GCOPTER_1\n" << r.grid.size.transpose() << '\n'
    << r.grid.origin.transpose() << '\n' << r.grid.resolution << '\n' <<
    r.start.transpose() << '\n'
    << r.velocity.transpose() << '\n' << r.goal.transpose() << '\n'
    << r.max_speed << ' ' << r.max_acceleration << ' ' << r.max_jerk << '\n';
  for (auto value : r.grid.data) {
    f << int(value) << ' ';
  }
  require(f.good(), "request_write_failed");
}
Request readRequest(const std::string & file)
{
  std::ifstream f(file); std::string magic; Request r; f >> magic;
  require(magic == "EASYFLY_GCOPTER_1", "request_format");
  for (int i = 0; i < 3; ++i) {
    f >> r.grid.size[i];
  }
  for (int i = 0; i < 3; ++i) {
    f >> r.grid.origin[i];
  }
  f >> r.grid.resolution;
  for (auto * v : {&r.start, &r.velocity, &r.goal}) {
    for (int i = 0; i < 3; ++i) {
      f >> (*v)[i];
    }
  }
  f >> r.max_speed >> r.max_acceleration >> r.max_jerk;
  require(
    f.good() && r.grid.size.minCoeff() > 0 && r.grid.size.maxCoeff() <= 512 &&
    int64_t(r.grid.size.x()) * r.grid.size.y() * r.grid.size.z() <= 1000000, "request_size");
  r.grid.data.resize(r.grid.size.prod());
  for (auto & value : r.grid.data) {
    int v = -2; f >> v; require(v == -1 || v == 0 || v == 100, "request_cell"); value = v;
  }
  require(f.good() && r.grid.valid(), "request_truncated"); return r;
}
void writePlan(const std::string & file, const Plan & p)
{
  std::ofstream f(file);
  f << std::setprecision(17) << "EASYFLY_POLYNOMIAL_1\n" << p.trajectory.getPieceNum() << '\n'
    << p.cost << ' ' << p.maximum_speed << ' ' << p.maximum_acceleration <<
    ' ' << p.maximum_jerk << '\n';
  for (const auto & piece : p.trajectory) {
    f << piece.getDuration() << '\n';
    for (int axis = 0; axis < 3; ++axis) {
      for (int i = 0; i < 6; ++i) {
        f << piece.getCoeffMat()(axis, i) << ' ';
      }
    }
    f << '\n';
  }
  require(f.good(), "plan_write_failed");
}
Plan readPlan(const std::string & file)
{
  std::ifstream f(file); Plan p; std::string magic; int count = 0;
  f >> magic >> count >> p.cost >> p.maximum_speed >> p.maximum_acceleration >> p.maximum_jerk;
  require(f.good() && magic == "EASYFLY_POLYNOMIAL_1" && count > 0 && count <= 128, "plan_format");
  for (int i = 0; i < count; ++i) {
    double duration = 0.; Controls c; f >> duration;
    for (int axis = 0; axis < 3; ++axis) {
      for (int j = 0; j < 6; ++j) {
        f >> c(axis, j);
      }
    }
    require(
      f.good() && std::isfinite(
        duration) && duration >= .001 && duration <= 110 && c.allFinite(), "plan_nonfinite");
    p.trajectory.emplace_back(duration, c);
  }
  require(p.trajectory.getTotalDuration() <= 110., "plan_too_long"); return p;
}
}  // namespace easyfly::trajectory
