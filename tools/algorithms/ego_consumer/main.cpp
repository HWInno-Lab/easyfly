// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iomanip>
#include <iostream>
#include <bspline_opt/uniform_bspline.h>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <traj_utils/msg/bspline.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

int main()
{
  // Take an out-of-line search symbol without constructing an uninitialized map.
  auto volatile search_symbol = &AStar::AstarSearch;
  if (search_symbol == nullptr) {return 1;}
  Eigen::MatrixXd line(3, 6);
  for (int i = 0; i < 6; ++i) {line.col(i) = Eigen::Vector3d(i, 2*i, 3.);}
  ego_planner::UniformBspline linear(line, 3, 1.);
  if ((linear.evaluateDeBoorT(0.)-Eigen::Vector3d(1., 2., 3.)).norm() > 1e-12 ||
      (linear.evaluateDeBoorT(3.)-Eigen::Vector3d(4., 8., 3.)).norm() > 1e-12 ||
      (linear.getDerivative().evaluateDeBoorT(1.)-Eigen::Vector3d(1., 2., 0.)).norm() > 1e-12) {return 2;}
  traj_utils::msg::Bspline message;
  message.navigation_session_id = "installed-consumer";
  message.order = 3; message.traj_id = 42;
  message.knots = {0., 1., 2., 3., 4., 5., 6., 7.};
  rclcpp::Serialization<traj_utils::msg::Bspline> codec;
  rclcpp::SerializedMessage bytes;
  codec.serialize_message(&message, &bytes);
  traj_utils::msg::Bspline roundtrip;
  codec.deserialize_message(&bytes, &roundtrip);
  if (message != roundtrip) {return 3;}
  quadrotor_msgs::msg::PositionCommand command;
  command.navigation_session_id = message.navigation_session_id;
  rclcpp::Serialization<quadrotor_msgs::msg::PositionCommand> position_codec;
  position_codec.serialize_message(&command, &bytes);
  quadrotor_msgs::msg::PositionCommand restored;
  position_codec.deserialize_message(&bytes, &restored);
  if (restored != command) {return 4;}
  std::cout << std::setprecision(17);
  for (int frame = 0; frame < 200; ++frame) {
    const int count = 6+frame%5;
    Eigen::MatrixXd control(3, count);
    for (int i = 0; i < count; ++i) {
      control.col(i) = Eigen::Vector3d(std::sin(.13*(i+frame)), std::cos(.07*(i-frame)), .2*i);
    }
    ego_planner::UniformBspline spline(control, 3, .1+.05*(frame%7));
    spline.setPhysicalLimits(2., 3., .05);
    auto derivative = spline.getDerivative();
    std::cout << "F " << frame << ' ' << spline.getTimeSum() << ' ' << spline.getMaxVelocityBound() << '\n';
    for (int i = 0; i <= 8; ++i) {
      const double t = spline.getTimeSum()*i/8.;
      const auto p = spline.evaluateDeBoorT(t);
      const auto v = derivative.evaluateDeBoorT(t);
      std::cout << p.transpose() << ' ' << v.transpose() << '\n';
    }
    spline.lengthenTime(1.3);
    std::cout << "L " << spline.getTimeSum() << ' ' << spline.getMaxVelocityBound() << '\n';
  }
  return 0;
}
