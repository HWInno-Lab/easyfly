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

#include "easyfly_super_planner/plan.hpp"
#include <fstream>
#include <iomanip>

namespace easyfly::superapi
{
namespace
{
using Controls = Eigen::Matrix<double, 3, 8>;
constexpr double choose(int n, int k) {
  double value=1.; for(int i=1;i<=k;++i) value=value*(n-i+1)/i; return value;
}
Controls bezier(const Piece & p)
{
  Controls b = Controls::Zero();
  for (int k = 0; k < 8; ++k) {
    for (int j = 0; j <= k; ++j) {
      b.col(k) += p.getCoeffMat().col(7 - j) *
        std::pow(p.getDuration(), j) * choose(k,j) / choose(7,j);
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
  if (depth >= 18 || !g.free(Vec(b.col(0))) || !g.free(Vec(b.col(7)))) {return false;}
  Controls values = b, left, right;
  left.col(0) = b.col(0); right.col(7) = b.col(7);
  for (int level = 1; level < 8; ++level) {
    for (int j = 0; j < 8 - level; ++j) {
      values.col(j) = .5 * (values.col(j) + values.col(j + 1));
    }
    left.col(level) = values.col(0); right.col(7 - level) = values.col(7 - level);
  }
  return clear(g, left, depth + 1, nodes, deadline) && clear(g, right, depth + 1, nodes, deadline);
}
void require(bool good, const char * message) {if (!good) {throw std::runtime_error(message);}}
}
void reserveUnknown(Grid & grid, double radius)
{
  require(grid.valid() && std::isfinite(radius) && radius>=.35 && radius<=1., "unknown_envelope");
  // Separable box dilation is conservative for a spherical envelope, linear
  // in cell count, and never clears an occupied cell. Include the full unknown
  // cube and outward cell rounding, not just its centre.
  const int extent=static_cast<int>(std::ceil(radius/grid.resolution))+1;
  std::vector<uint8_t> mask(grid.data.size()), next(mask.size());
  for (size_t i=0;i<mask.size();++i) {mask[i]=grid.data[i]==-1;}
  const int strides[]{1,grid.size.x(),grid.size.x()*grid.size.y()};
  for (int axis=0;axis<3;++axis) {
    const int stride=strides[axis], length=grid.size[axis];
    for (size_t index=0;index<mask.size();++index) {
      if ((index/stride)%length!=0) {continue;}
      int unknown=0;
      for (int k=0;k<=std::min(extent,length-1);++k) {unknown+=mask[index+k*stride];}
      for (int k=0;k<length;++k) {
        next[index+k*stride]=unknown>0;
        if (k-extent>=0) {unknown-=mask[index+(k-extent)*stride];}
        if (k+extent+1<length) {unknown+=mask[index+(k+extent+1)*stride];}
      }
    }
    mask.swap(next);
  }
  for (size_t i=0;i<mask.size();++i) {if (mask[i] && grid.data[i]==0) {grid.data[i]=-1;}}
}
bool collisionFree(
  const Grid & grid, const Trajectory & trajectory, double from,
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
void validate(const Request & r, Plan & out, const Grid & current_grid, Steady::time_point deadline)
{
  out.maximum_speed = out.maximum_acceleration = out.maximum_jerk = 0.;
  auto & t = out.trajectory;
  require(
    std::isfinite(out.cost) && t.getPieceNum() > 0 && t.getPieceNum() <= 128 &&
    std::isfinite(
      t.getTotalDuration()) && t.getTotalDuration() > .05 && t.getTotalDuration() <= 110.,
    "super_failed");
  require(std::isfinite(out.backup_start) &&
    (out.backup_start == -1. || (r.execute_committed && out.backup_start >= 0. &&
     out.backup_start < t.getTotalDuration())), "invalid_backup_metadata");
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
    for (int power = 2; power <= 7; ++power) {
      acceleration.col(9 - power) = p.getCoeffMat().col(7 - power) * power * (power - 1);
    }
    const double jerk = Piece(p.getDuration(), acceleration).getMaxVelRate();
    require(std::isfinite(jerk), "nonfinite_jerk_bound");
    out.maximum_jerk = std::max(out.maximum_jerk, jerk);
  }
  require(
    last_v.norm() < 1e-4 && last_a.norm() < 1e-4 &&
    (out.partial ? (r.execute_committed && out.backup_start >= 0. &&
      (last_p-r.start).norm() >= .25 &&
      (r.start-r.goal).norm()-(last_p-r.goal).norm() >= .25 &&
      (last_p-r.goal).norm() >= .35) : (last_p-r.goal).norm() < 1e-5),
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
  f << std::setprecision(17) << (r.execute_committed ? "EASYFLY_SUPER_REQUEST_2\n" : "EASYFLY_SUPER_REQUEST_1\n") << r.grid.size.transpose() << '\n'
    << r.grid.origin.transpose() << '\n' << r.grid.resolution << '\n' <<
    r.start.transpose() << '\n'
    << r.velocity.transpose() << '\n' << r.goal.transpose() << '\n' << r.yaw << '\n'
    << r.max_speed << ' ' << r.max_acceleration << ' ' << r.max_jerk << '\n';
  if (r.execute_committed) {f << "1\n";}
  for (auto value : r.grid.data) {
    f << int(value) << ' ';
  }
  require(f.good(), "request_write_failed");
}
Request readRequest(const std::string & file)
{
  std::ifstream f(file); std::string magic; Request r; f >> magic;
  require(magic == "EASYFLY_SUPER_REQUEST_1" || magic == "EASYFLY_SUPER_REQUEST_2", "request_format");
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
  f >> r.yaw >> r.max_speed >> r.max_acceleration >> r.max_jerk;
  if (magic == "EASYFLY_SUPER_REQUEST_2") {
    int mode=-1; f >> mode; require(mode == 1, "request_mode"); r.execute_committed=true;
  }
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
  f << std::setprecision(17) << ((p.partial || p.backup_start >= 0.) ? "EASYFLY_SEPTIC_2\n" : "EASYFLY_SEPTIC_1\n") << p.trajectory.getPieceNum() << '\n'
    << p.cost << ' ' << p.maximum_speed << ' ' << p.maximum_acceleration <<
    ' ' << p.maximum_jerk << '\n';
  if (p.partial || p.backup_start >= 0.) {f << p.backup_start << ' ' << int(p.partial) << '\n';}
  for (const auto & piece : p.trajectory) {
    f << piece.getDuration() << '\n';
    for (int axis = 0; axis < 3; ++axis) {
      for (int i = 0; i < 8; ++i) {
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
  require(f.good() && (magic == "EASYFLY_SEPTIC_1" || magic == "EASYFLY_SEPTIC_2") && count > 0 && count <= 128, "plan_format");
  if (magic == "EASYFLY_SEPTIC_2") {
    int partial=-1; f >> p.backup_start >> partial;
    require(f.good() && (partial==0 || partial==1) && std::isfinite(p.backup_start) &&
      p.backup_start>=0., "plan_backup_metadata"); p.partial=partial;
  }
  for (int i = 0; i < count; ++i) {
    double duration = 0.; Controls c; f >> duration;
    for (int axis = 0; axis < 3; ++axis) {
      for (int j = 0; j < 8; ++j) {
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
}  // namespace easyfly::superapi
