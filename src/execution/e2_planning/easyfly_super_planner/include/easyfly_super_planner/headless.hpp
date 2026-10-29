// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <ros_interface/ros_interface.hpp>
#include <chrono>
#include <iostream>
namespace easyfly::superapi {
using namespace geometry_utils;
using namespace super_utils;
class Headless final : public ros_interface::RosInterface {
 public:
  // This worker solves a fixed measured snapshot; the vehicle is held at rest
  // until independent validation/admission. CPU time must not advance its
  // trajectory state. Real worker deadlines still use Steady::now().
  const double snapshot_time = std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
  std::vector<Eigen::Vector3d> route;
  double backup_start{-1.};
  void debug(const std::string& msg) override {}
  void info(const std::string& msg) override {}
  void warn(const std::string& msg) override {std::cerr << msg << "\n";}
  void error(const std::string& msg) override {std::cerr << msg << "\n";}
  void fatal(const std::string& msg) override {std::cerr << msg << "\n";}
  void setSimTime(const double &sim_time) override {throw std::logic_error("headless interface never publishes synthetic time");}
  double getSimTime() override {return snapshot_time;}
  void getSimTime(int32_t &sec, uint32_t &nsec) override {const auto t=getSimTime();sec=static_cast<int32_t>(t);nsec=static_cast<uint32_t>((t-sec)*1e9);}
  void vizExpTraj(const Trajectory &traj, const std::string & ns) override {}
  void vizBackupTraj(const Trajectory & traj) override {}
  void vizFrontendPath(const vec_Vec3f & path) override {route.assign(path.begin(),path.end());}
  void vizExpSfc(const PolytopeVec & sfc) override {}
  void vizBackupSfc(const Polytope & sfc) override {}
  void vizGoalPath(const vec_Vec3f & path) override {}
  void vizCommittedTraj(const Trajectory &committed_traj, const double & backup_traj_start_TT) override {backup_start = backup_traj_start_TT;}
  void vizYawTraj(const Trajectory & pos_traj, const Trajectory & yaw_traj) override {}
  void vizAstarBoundingBox(const Vec3f & bbox_min, const Vec3f & bbox_max) override {}
  void vizAstarPoints(const Vec3f & position, const Color & c, const std::string & ns, const double & size, const int & id) override {}
  void vizReplanLog(const Trajectory & exp_traj, const Trajectory & backup_traj, const Trajectory & exp_yaw_traj, const Trajectory & backup_yaw_traj, const PolytopeVec & exp_sfc, const Polytope & backup_sfc, const vec_Vec3f & pc_for_sfc, const int & ret_code) override {}
  void vizCiriSeedLine(const Vec3f & a, const Vec3f & b, const double & robot_r) override {}
  void vizCiriEllipsoid(const Ellipsoid & ellipsoid) override {}
  void vizCiriInfeasiblePoint(const Vec3f p) override {}
  void vizCiriPolytope(const Polytope & polytope, const std::string & ns) override {}
  void vizCiriPointCloud(const vec_Vec3f & points) override {}
};
}
