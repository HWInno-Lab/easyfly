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


#include <easyfly_trajectory_optimizer/plan.hpp>
#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <path_searching/astar2.h>
#include <nlopt.h>
#include <yaml-cpp/yaml.h>
#include <sys/resource.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <optional>

namespace et = easyfly::trajectory;
namespace fp = fast_planner;
using Vec = Eigen::Vector3d;
static YAML::Node yamlVector(const Eigen::VectorXd & value)
{
  YAML::Node out(YAML::NodeType::Sequence);
  for (int i = 0; i < value.size(); ++i) {
    out.push_back(value[i]);
  }
  return out;
}
static YAML::Node matrix(const Eigen::MatrixXd & value)
{
  YAML::Node out(YAML::NodeType::Sequence);
  for (int i = 0; i < value.rows(); ++i) {
    out.push_back(yamlVector(value.row(i).transpose()));
  }
  return out;
}
static et::Plan convert(fp::NonUniformBspline & spline)
{
  auto velocity = spline.getDerivative();auto acceleration = velocity.getDerivative();
  auto jerk = acceleration.getDerivative();
  const double span = spline.getKnotSpan(), duration = spline.getTimeSum();
  const int pieces = static_cast<int>(std::llround(duration / span));
  if (!std::isfinite(span) || span <= 0. || pieces < 1 || pieces > 128 ||
    std::abs(pieces * span - duration) > 1e-7)
  {
    throw std::runtime_error("native spline knot layout");
  }
  et::Plan result;result.cost = 0.;
  for (int i = 0; i < pieces; ++i) {
    const double start = i * span;
    Eigen::Matrix<double, 3, 6> c = Eigen::Matrix<double, 3, 6>::Zero();
    c.col(2) = jerk.evaluateDeBoorT(start + .5 * span) / 6.;
    c.col(3) = acceleration.evaluateDeBoorT(start) / 2.;
    c.col(4) = velocity.evaluateDeBoorT(start);
    c.col(5) = spline.evaluateDeBoorT(start);
    result.trajectory.emplace_back(span, c);
    // Independent representation check: coefficients must reproduce native de
    // Boor position and derivatives, including joins, without retiming/scaling.
    for (double fraction:{0., .17, .5, .83, 1.}) {
      const double t = fraction * span;
      if ((result.trajectory[i].getPos(t) - spline.evaluateDeBoorT(start + t)).norm() > 1e-7 ||
        (result.trajectory[i].getVel(t) - velocity.evaluateDeBoorT(start + t)).norm() > 1e-7 ||
        (result.trajectory[i].getAcc(t) - acceleration.evaluateDeBoorT(start + t)).norm() > 1e-7)
      {
        throw std::runtime_error("native spline conversion mismatch");
      }
    }
  }
  return result;
}
static et::Plan plan(
  const et::Request & request, double minimum_duration,
  const std::string & evidence, et::Steady::time_point deadline)
{
  const auto & grid = request.grid;
  if (!grid.valid() || grid.size.minCoeff() < 3 || grid.size.maxCoeff() > 256 ||
    grid.data.size() > 500000 || std::abs(grid.resolution - .2) > 1e-9 ||
    !request.start.allFinite() || !request.goal.allFinite() || !request.velocity.allFinite() ||
    request.velocity.norm() > .3 || !grid.free(request.start) || !grid.free(request.goal) ||
    !std::isfinite(request.max_speed) || request.max_speed < .1 || request.max_speed > 3. ||
    !std::isfinite(minimum_duration) || minimum_duration < 0. || minimum_duration > 100.)
  {
    throw std::runtime_error("FUEL native trajectory input");
  }
  fuel_support::Config cfg;
  auto set = [&](const std::string & key, const auto & value) {cfg.values[key] = value;};
  set("sdf_map/resolution", grid.resolution);set("sdf_map/ground_height", grid.origin.z());
  set("sdf_map/origin_x", grid.origin.x());set("sdf_map/origin_y", grid.origin.y());
  const char * axis[]{"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    set(std::string("sdf_map/map_size_") + axis[i], grid.size[i] * grid.resolution);
    set(std::string("sdf_map/box_min_") + axis[i], grid.origin[i]);
    set(
      std::string(
        "sdf_map/box_max_") + axis[i], grid.origin[i] + grid.size[i] * grid.resolution - 1e-6);
  }
  set("sdf_map/obstacles_inflation", 0.);set("sdf_map/optimistic", false);
  set("sdf_map/signed_dist", true);set("sdf_map/max_ray_length", 8.);
  set("sdf_map/virtual_ceil_height", 1000.);
  auto map = std::make_shared<fp::SDFMap>();map->initMap(cfg);
  map->importSnapshot(grid.data, grid.data);map->buildSnapshotESDF();et::budget(deadline);
  auto environment = std::make_shared<fp::EDTEnvironment>();environment->setMap(map);
  environment->init();
  set("astar/resolution_astar", grid.resolution);set("astar/lambda_heu", 1.);
  set("astar/max_search_time", .15);set("astar/allocate_num", 100000);
  fp::Astar astar;astar.init(cfg, environment);astar.setResolution(grid.resolution);
  if (astar.search(request.start, request.goal) != fp::Astar::REACH_END) {
    throw std::runtime_error("native FUEL A* has no route");
  }
  auto path = astar.getPath();
  if (path.size() < 2) {throw std::runtime_error("native FUEL path too short");}
  path.front() = request.start;path.back() = request.goal;
  std::vector<double> arc{0.};
  for (size_t i = 1; i < path.size(); ++i) {
    arc.push_back(arc.back() + (path[i] - path[i - 1]).norm());
  }
  const double length = arc.back();
  if (length < .15 || length > 100.) {throw std::runtime_error("native FUEL route length");}
  const int samples = std::clamp(static_cast<int>(std::ceil(length / .25)) + 1, 8, 60);
  std::vector<Vec> waypoints;
  size_t segment = 1;
  for (int i = 0; i < samples; ++i) {
    const double distance = length * i / (samples - 1);
    while (segment + 1 < arc.size() && arc[segment] < distance) {++segment;}
    const double interval = arc[segment] - arc[segment - 1];
    waypoints.push_back(
      interval >
      1e-9 ? (path[segment - 1] + (path[segment] - path[segment - 1]) *
      ((distance - arc[segment - 1]) / interval)).eval() : path[segment]);
  }
  waypoints.front() = request.start;waypoints.back() = request.goal;
  set("manager/bspline_degree", 3);set("optimization/fixed_boundary", true);
  set("optimization/ld_smooth", 20.);set("optimization/ld_dist", 1e5);
  set("optimization/ld_feasi", 1e5);set("optimization/ld_start", 1.);
  set("optimization/ld_end", 1.);set("optimization/ld_guide", 2.);
  set("optimization/ld_waypt", 1.);set("optimization/ld_view", 1.);set("optimization/ld_time", 1.);
  set("optimization/dist0", .35);set("optimization/max_vel", request.max_speed * .5);
  set("optimization/max_acc", .1);set("optimization/dlmin", .1);set("optimization/wnl", 1.);
  set("optimization/algorithm1", int(NLOPT_LD_LBFGS));
  set("optimization/algorithm2", int(NLOPT_LD_LBFGS));
  for (int i = 1; i <= 4; ++i) {
    set("optimization/max_iteration_num" + std::to_string(i), 1000);set(
      "optimization/max_iteration_time" + std::to_string(
        i), .15);
  }
  double duration = std::max({4., minimum_duration, length / (request.max_speed * .5)});
  std::string reason = "native FUEL trajectory infeasible";
  std::optional<et::Plan> accepted;
  YAML::Node accepted_trace;
  double rejected_duration = 0., accepted_duration = 0.;
  int evaluations = 0;
  const bool refine_timing = length >= 8. && request.max_speed >= 1.5;
  for (int attempt = 0; attempt < 8; ++attempt) {
    if (accepted && et::Steady::now() + std::chrono::milliseconds(250) >= deadline) {break;}
    et::budget(deadline);if (duration > 105.) {break;}
    ++evaluations;
    double dt = duration / (samples - 1);
    Eigen::MatrixXd controls;
    fp::NonUniformBspline::parameterizeToBspline(
      dt, waypoints,
      {request.velocity, Vec::Zero(), Vec::Zero(), Vec::Zero()}, 3, controls);
    // Exact cubic boundary coordinates; the native optimizer freezes these
    // supplied variables, so no optimized curve is patched after the solve.
    controls.row(0) = (request.start - request.velocity * dt).transpose();
    controls.row(1) = request.start.transpose();
    controls.row(2) = (request.start + request.velocity * dt).transpose();
    for (int i = controls.rows() - 3; i < controls.rows(); ++i) {
      controls.row(i) = request.goal.transpose();
    }
    double duration_growth = 1.3;
    try {
      fp::BsplineOptimizer optimizer;optimizer.setParam(cfg);optimizer.setEnvironment(environment);
      optimizer.setBoundaryStates(
        {request.start, request.velocity, Vec::Zero()},
        {request.goal, Vec::Zero(), Vec::Zero()});
      std::vector<Vec> guide;
      for (int i = 3; i < controls.rows() - 3; ++i) {
        guide.push_back(controls.row(i).transpose());
      }
      optimizer.setGuidePath(guide);
      optimizer.optimize(
        controls, dt, fp::BsplineOptimizer::SMOOTHNESS | fp::BsplineOptimizer::DISTANCE |
        fp::BsplineOptimizer::FEASIBILITY | fp::BsplineOptimizer::GUIDE, 1, 1);
      et::budget(deadline);
      fp::NonUniformBspline spline(controls, 3, dt);auto result = convert(spline);
      // Bounded candidate evidence is distinct from the executable output.
      // Retain rejected native solutions for independent collision diagnosis.
      et::writePlan(evidence + ".attempt-" + std::to_string(attempt) + ".candidate", result);
      et::validate(request, result, grid, deadline);
      if (result.maximum_acceleration > et::kTrackingAcceleration ||
        result.maximum_jerk > et::kTrackingJerk)
      {
        // Estimate a smaller timing step near feasibility, then rerun the
        // complete native optimization with the measured boundary velocity.
        // Never rescale an existing curve or relax the independent validator.
        duration_growth = std::clamp(
          1.05 * std::max(
            std::sqrt(result.maximum_acceleration / et::kTrackingAcceleration),
            std::cbrt(result.maximum_jerk / et::kTrackingJerk)), 1.05, 1.3);
        throw std::runtime_error("native FUEL tracking dynamics");
      }
      if (result.trajectory.getTotalDuration() + 1e-7 < minimum_duration) {
        throw std::runtime_error("native FUEL yaw duration");
      }
      YAML::Node trace;trace["backend"] = "FUEL native BsplineOptimizer and NonUniformBspline";
      trace["degree"] = 3;trace["dt"] = dt;trace["attempt"] = attempt;
      trace["controls"] = matrix(controls);trace["knots"] = yamlVector(spline.getKnot());
      for (const auto & point:path) {
        trace["native_path"].push_back(yamlVector(point));
      }
      accepted = result;accepted_trace = trace;accepted_duration = duration;
      // Long, explicitly faster requests can refine the native solve between
      // the rejected and accepted durations within the SAME eight-attempt and
      // wall-clock budgets. Retain only independently validated solutions.
      if (!refine_timing || rejected_duration <= 0. ||
        accepted_duration - rejected_duration < .01 * accepted_duration) {break;}
      duration = .5 * (accepted_duration + rejected_duration);
      continue;
    } catch (const std::exception & error) {
      reason = error.what();
      std::cerr << "FUEL native attempt " << attempt << ": " << reason << '\n';
    }
    rejected_duration = duration;
    duration = accepted ? .5 * (accepted_duration + rejected_duration) : duration * duration_growth;
  }
  if (!accepted) {throw std::runtime_error(reason);}
  et::budget(deadline);
  accepted_trace["native_evaluations"] = evaluations;
  accepted_trace["timing_refinement_enabled"] = refine_timing;
  YAML::Emitter output;output.SetDoublePrecision(17);output << accepted_trace;
  std::ofstream file(evidence);file << output.c_str();
  if (!file) {throw std::runtime_error("native FUEL evidence write");}
  std::cout << "FUEL native accepted: evaluations=" << evaluations << " duration=" <<
    accepted->trajectory.getTotalDuration() << " max_speed=" << accepted->maximum_speed <<
    " acceleration=" << accepted->maximum_acceleration << " jerk=" << accepted->maximum_jerk <<
    '\n';
  return *accepted;
}
int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 4) {return 2;}
  rlimit memory{1024ULL * 1024 * 1024, 1024ULL * 1024 * 1024}, cpu{3, 3};
  if (setrlimit(RLIMIT_AS, &memory) != 0 || setrlimit(RLIMIT_CPU, &cpu) != 0) {return 3;}
  try {
    double minimum = 0.;
    if (argc == 4) {
      size_t end = 0;minimum = std::stod(argv[3], &end);if (end != std::string(argv[3]).size()) {
        throw std::runtime_error("minimum duration format");
      }
    }
    const auto request = et::readRequest(argv[1]);
    const auto result = plan(
      request, minimum, std::string(
        argv[2]) + ".native.yaml", et::Steady::now() + std::chrono::seconds(2));
    et::writePlan(argv[2], result);return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';return 1;
  }
}
