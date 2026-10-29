// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_super_planner/plan.hpp"
namespace es=easyfly::superapi;
static es::Request request() {
  es::Request r;
  r.grid.size={30,30,18};r.grid.origin={-2.,-3.,0.};r.grid.resolution=.2;
  r.grid.data.assign(r.grid.size.prod(),0);
  r.start={0.,0.,1.2};r.goal={1.,0.,1.2};r.velocity.setZero();
  return r;
}
static es::Plan analytic() {
  es::Plan p;Eigen::Matrix<double,3,8> c=Eigen::Matrix<double,3,8>::Zero();
  // Exact septic smooth step. This fixture tests validation, not an optimizer.
  c(0,0)=-20./1e7;c(0,1)=70./1e6;c(0,2)=-84./1e5;c(0,3)=35./1e4;
  c(2,7)=1.2;p.trajectory.emplace_back(10.,c);p.cost=10.;return p;
}
TEST(Septic, ExactEndpointsAndDerivativeExtrema) {
  auto r=request();auto p=analytic();
  EXPECT_NO_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)));
  EXPECT_NEAR(p.maximum_speed,.21875,1e-7);
  EXPECT_LE(p.maximum_acceleration,2.);EXPECT_LE(p.maximum_jerk,8.);
}
TEST(Septic, UnknownBetweenFreeEndpointsIsRejectedContinuously) {
  auto r=request();auto p=analytic();
  r.grid.data[r.grid.index(r.grid.cell(es::Vec{.5,0.,1.2}))]=-1;
  EXPECT_TRUE(r.grid.free(p.trajectory.getPos(0.)));
  EXPECT_TRUE(r.grid.free(p.trajectory.getPos(10.)));
  EXPECT_FALSE(es::collisionFree(r.grid,p.trajectory));
  EXPECT_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)),std::runtime_error);
}
TEST(Septic, OverSpeedAndChangedMeasuredStartAreRejected) {
  auto r=request();auto p=analytic();r.max_speed=.1;
  EXPECT_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)),std::runtime_error);
  r=request();r.start.x()+=.02;
  EXPECT_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)),std::runtime_error);
}
TEST(Septic, NativePartialStopNeedsExplicitModeAndProgress) {
  auto r=request();auto p=analytic();r.goal={2.,0.,1.2};
  p.partial=true;p.backup_start=5.;
  EXPECT_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)),std::runtime_error);
  r.execute_committed=true;
  EXPECT_NO_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)));
  p.partial=false;
  EXPECT_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)),std::runtime_error);
  p.partial=true;p.backup_start=11.;
  EXPECT_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)),std::runtime_error);
  p.backup_start=5.;r.goal={-2.,0.,1.2};
  EXPECT_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)),std::runtime_error);
  r.goal={2.,0.,1.2};r.grid.data[r.grid.index(r.grid.cell(es::Vec{.5,0.,1.2}))]=-1;
  EXPECT_THROW(es::validate(r,p,r.grid,es::Steady::now()+std::chrono::seconds(1)),std::runtime_error);
}
TEST(Septic, UnknownEnvelopeIncludesWholeCellsAndNeverClearsOccupancy) {
  auto r=request();auto original=r.grid;
  const auto unknown=r.grid.cell(es::Vec{1.5,0.,1.2});
  original.data[original.index(unknown)]=-1;
  const auto occupied=r.grid.cell(es::Vec{1.,0.,1.2});
  original.data[original.index(occupied)]=100;
  auto grid=original;es::reserveUnknown(grid,.65);
  EXPECT_EQ(grid.data[grid.index(occupied)],100);
  for (size_t i=0;i<grid.data.size();++i) {
    const auto cell=grid.cell(static_cast<int>(i));
    const bool reserved=(cell-unknown).cwiseAbs().maxCoeff()<=5;
    EXPECT_EQ(grid.data[i],original.data[i]==100 ? 100 : reserved ? -1 : 0);
  }
}
