// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include "easyfly_super_planner/plan.hpp"
#include "easyfly_super_planner/headless.hpp"
#include <rog_map/rog_map.h>
#include <super_core/super_planner.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace easyfly::superapi {
namespace {
class NativeFailure : public std::runtime_error {public: using std::runtime_error::runtime_error;};
// Import an already inflated, source-stamped EasyFly map. This is a map-format
// conversion, not a point-cloud simulator. Only the explicit committed profile
// preserves unknown cells for exploration; execution still certifies known free.
class SnapshotMap final : public rog_map::ROGMap {
 public:
  const double getSystemWalltimeNow() override {
    return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
  }
  explicit SnapshotMap(const std::string& settings) {
    cfg_=rog_map::Config(settings);init();
  }

  void import(const Request& request, Steady::time_point deadline) {
    const auto& grid=request.grid;
    // Fill the padding as occupied too: CIRI must never see an unobserved
    // outside region as usable empty space when constructing a corridor.
    for(int z=-2;z<grid.size.z()+2;++z) {
      for(int y=-2;y<grid.size.y()+2;++y) {
        for(int x=-2;x<grid.size.x()+2;++x) {
          budget(deadline);
          const Eigen::Vector3i cell{x,y,z};
          const auto position=grid.center(cell);
          if(!insideLocalMap(position)) throw std::runtime_error("ROG snapshot outside storage");
          rog_map::Vec3i global;probMapPosToGlobalIndex(position,global);
          rog_map::Vec3f center;probMapGlobalIndexToPos(global,center);
          if((center-position).norm()>1e-8) throw std::runtime_error("ROG lattice mismatch");
          const int hash=getHashIndexFromGlobalIndex(global);
          if(grid.free(cell)) missPointUpdate(position,hash,4);
          else if (!request.execute_committed || !grid.inside(cell) ||
                   grid.data[grid.index(cell)] == 100) hitPointUpdate(position,hash,4);
        }
      }
    }
    for(size_t i=0;i<grid.data.size();++i) {
      const auto center=grid.center(grid.cell(static_cast<int>(i)));
      if(isKnownFree(center)!=(grid.data[i]==0) || isOccupied(center)!=(grid.data[i]==100 || (!request.execute_committed && grid.data[i]==-1)))
        throw std::runtime_error("ROG import round-trip mismatch");
    }
    robot_state_.p=request.start;robot_state_.v=request.velocity;
    robot_state_.a.setZero();robot_state_.j.setZero();
    robot_state_.yaw=request.yaw;
    robot_state_.q=Eigen::AngleAxisd(request.yaw,Eigen::Vector3d::UnitZ());
    robot_state_.rcv_time=0.;robot_state_.rcv=true;map_empty_=false;
  }
};
}

static Plan optimizeAttempt(const Request& request, Steady::time_point deadline,
                            double velocity_target, int attempt,
                            std::shared_ptr<SnapshotMap>& map) {
  if(!request.grid.valid() || !request.start.allFinite() || !request.velocity.allFinite() ||
     !request.goal.allFinite() || !std::isfinite(request.yaw) ||
     !std::isfinite(request.max_speed) || request.max_speed<.1 || request.max_speed>3. ||
     request.velocity.norm()>.3 || request.velocity.norm()>request.max_speed ||
     !std::isfinite(request.max_acceleration) || request.max_acceleration<=0. || request.max_acceleration>2. ||
     !std::isfinite(request.max_jerk) || request.max_jerk<=0. || request.max_jerk>8. ||
     !request.grid.free(request.start) ||
     (!request.grid.free(request.goal) && (!request.execute_committed ||
      !request.grid.inside(request.grid.cell(request.goal)) ||
      request.grid.data[request.grid.index(request.grid.cell(request.goal))] != -1)))
    throw std::runtime_error("invalid SUPER request");
  const auto share=ament_index_cpp::get_package_share_directory("easyfly_super_planner");
  auto config=YAML::LoadFile(share+"/config/gazebo.yaml");
  config["traj_opt"]["boundary"]["max_vel"]=velocity_target;
  config["traj_opt"]["boundary"]["max_acc"]=request.max_acceleration*.75;
  config["traj_opt"]["boundary"]["max_jerk"]=request.max_jerk*.75;
  if (request.execute_committed) {
    config["super_planner"]["frontend_in_known_free"]=false;
    config["super_planner"]["backup_in_known_free"]=true;
    if (request.max_speed > .8) {
      config["traj_opt"]["exp_traj"]["penna_acc"]=50000000.;
      config["traj_opt"]["exp_traj"]["penna_jerk"]=50000000.;
    }
    config["traj_opt"]["backup_traj"]["max_acc"]=std::min(request.max_acceleration*.75, kTrackingAcceleration*.8);
    config["traj_opt"]["backup_traj"]["max_jerk"]=std::min(request.max_jerk*.75, kTrackingJerk*.8);
    config["traj_opt"]["backup_traj"]["penna_acc"]=50000000.;
    config["traj_opt"]["backup_traj"]["penna_jerk"]=50000000.;
    // A longer visibility horizon is allowed only for a continuously known-free
    // direct corridor; otherwise the native backup retains its short horizon.
    if (request.grid.freeBox(Vec(request.start.cwiseMin(request.goal)),
                             Vec(request.start.cwiseMax(request.goal))))
      config["super_planner"]["safe_corridor_line_max_length"]=std::min(40., (request.goal-request.start).norm()+1.);

    config["super_planner"]["planning_horizon"]=std::min(40., (request.goal-request.start).norm()+2.);
    if (request.max_speed > .8) {
      config["traj_opt"]["boundary"]["max_acc"]=std::min(request.max_acceleration*.75, kTrackingAcceleration*.8);
      config["traj_opt"]["boundary"]["max_jerk"]=std::min(request.max_jerk*.75, kTrackingJerk*.8);
    }
  }
  const auto size=(request.grid.size.cast<double>()*request.grid.resolution).eval();
  const auto center=(request.grid.origin+size*.5).eval();
  // CIRI consumes occupied voxel CENTRES as points. Enclose each complete
  // occupied/unknown cube before asking CIRI for a separating plane. The input
  // grid already includes the vehicle envelope; this is representation error
  // with outward roundoff padding, not a reduced physical safety radius.
  config["super_planner"]["robot_r"]=std::sqrt(3.)*.5*request.grid.resolution+1e-6;
  config["rog_map"]["resolution"]=request.grid.resolution;
  config["rog_map"]["inflation_resolution"]=request.grid.resolution;
  for(int i=0;i<3;++i) {
    config["rog_map"]["map_size"][i]=size[i]+4.;
    config["rog_map"]["fix_map_origin"][i]=center[i];
    // Native A* centres its storage on the measured start. Cover the complete
    // fixed map even when the start is off-centre, without doubling all axes.
    const double span=std::max(request.start[i]-request.grid.origin[i],
      request.grid.origin[i]+size[i]-request.start[i]);
    config["astar"]["map_voxel_num"][i]=request.execute_committed ?
      2*static_cast<int>(std::ceil(span/request.grid.resolution))+20 : request.grid.size[i]+20;
  }
  // Each worker already has a private job directory. Store the exact settings
  // consumed by the official code beside its request and output.
  const auto settings=std::filesystem::current_path()/("super-settings-attempt-"+std::to_string(attempt)+".yaml");
  std::ofstream(settings)<<config;
  // Native ROG-Map permits one initialization per process. All retiming
  // attempts use the same immutable imported snapshot and measured state.
  if(!map) {
    map=std::make_shared<SnapshotMap>(settings.string());
    map->import(request,deadline);
  }
  auto interface=std::make_shared<Headless>();
  super_planner::SuperPlanner planner(settings.string(),interface,map);
  rog_map::RobotState measured;planner.getRobotState(measured);
  budget(deadline);
  const auto before=Steady::now();
  const auto code=planner.PlanFromRest(request.goal,request.yaw,true);
  budget(deadline);
  if(code!=super_utils::SUCCESS) throw NativeFailure("native SUPER PlanFromRest rejected");
  const auto committed=planner.getCommittedPositionTrajectory();
  const auto native=planner.getExplorationPositionTrajectory();
  const auto raw_path=std::filesystem::current_path()/("native-trajectories-attempt-"+std::to_string(attempt)+".json");
  std::ofstream diagnostic(raw_path);
  diagnostic<<std::setprecision(17)<<"{\"scope\":\"raw native output; admission requires independent continuous certificate\",\"committed\":[";
  auto raw=[&diagnostic](const geometry_utils::Trajectory& trajectory) {
    bool first=true;
    for(const auto& piece:trajectory) {
      if(!first) diagnostic<<',';first=false;
      diagnostic<<"{\"duration\":"<<piece.getDuration()<<",\"coefficients\":[";
      for(int axis=0;axis<3;++axis) for(int j=0;j<piece.getCoeffMat().cols();++j) {
        if(axis||j) diagnostic<<',';diagnostic<<piece.getCoeffMat()(axis,j);
      }
      diagnostic<<"]}";
    }
  };
  raw(committed);diagnostic<<"],\"exploration\":[";raw(native);diagnostic<<"]}\n";

  Plan output;
  output.backup_start=request.execute_committed ? interface->backup_start : -1.;
  const auto& selected=request.execute_committed ? committed : native;
  output.partial=(selected.getPos(selected.getTotalDuration())-request.goal).norm()>=1e-5;
  for(const auto& piece:selected) {
    if(piece.getDegree()!=7) throw std::runtime_error("native SUPER polynomial degree");
    output.trajectory.emplace_back(piece.getDuration(),piece.getCoeffMat());
  }
  output.route=interface->route;
  // This field is the independently observable duration, not a fabricated
  // value of the private optimizer objective.
  output.cost=output.trajectory.getTotalDuration();
  validate(request,output,request.grid,deadline);
  std::cout<<"native SUPER PlanFromRest: code="<<code
           <<" detail="<<planner.getLatestReplanLog().getRetCode()
           <<" solve_seconds="<<std::chrono::duration<double>(Steady::now()-before).count()
           <<" ROG_cells="<<request.grid.data.size()<<" degree=7\n";
  diagnostic.close();
  if(output.maximum_acceleration<=kTrackingAcceleration && output.maximum_jerk<=kTrackingJerk) {
    std::filesystem::copy_file(settings,"super-settings.yaml");
    std::filesystem::copy_file(raw_path,"native-trajectories.json");
  }
  return output;
}

Plan optimize(const Request& request, Steady::time_point deadline) {
  // Re-run the native optimizer with tighter timing targets. Scaling an existing
  // polynomial would also change the measured initial velocity, so do not do it.
  // Every attempt retains the full native backup and independent certificates,
  // and all attempts share the original wall/CPU/memory budget.
  std::shared_ptr<SnapshotMap> map;
  double velocity_target=request.max_speed*(request.execute_committed && request.max_speed > .8 ? .85 : .5);
  for(int attempt=0;attempt<8;++attempt) {
    budget(deadline);
    Plan output;
    try {output=optimizeAttempt(request,deadline,velocity_target,attempt,map);}
    catch (const NativeFailure& error) {
      if (!request.execute_committed) {throw;}
      std::cout << "SUPER native retry: attempt=" << attempt << " reason=" << error.what() << '\n';
      velocity_target*=.75;
      continue;
    }
    std::cout<<"SUPER tracking candidate: attempt="<<attempt
             <<" acceleration="<<output.maximum_acceleration
             <<" jerk="<<output.maximum_jerk<<'\n';
    if(output.maximum_acceleration<=kTrackingAcceleration && output.maximum_jerk<=kTrackingJerk)
      return output;
    // Derivative scaling estimates only choose the next native solve target;
    // no existing coefficients or measured initial derivatives are rescaled.
    // Leave margin and certify the regenerated polynomial independently.
    double reduction=.75;
    if(output.maximum_acceleration>kTrackingAcceleration)
      reduction=std::min(reduction,.9*std::sqrt(kTrackingAcceleration/output.maximum_acceleration));
    if(output.maximum_jerk>kTrackingJerk)
      reduction=std::min(reduction,.9*std::cbrt(kTrackingJerk/output.maximum_jerk));
    velocity_target*=reduction;
  }
  throw std::runtime_error("tracking_dynamics_infeasible");
}
}
