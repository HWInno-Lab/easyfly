// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
// Explicit simulation actuator. Encoder output is the measured physical joint,
// never the requested sine trajectory. The moving pad has physical collision
// and velocity integration; no world-pose teleportation or detector injection.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace easyfly_gazebo
{
class LandingFixture : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    node_ = gazebo_ros::Node::Get(sdf);
    model_ = model;
    mode_ = sdf->Get<std::string>("mode");
    if (mode_ == "gimbal") {
      joint_ = model_->GetJoint("landing_gimbal_pitch");
      if (!joint_) {throw std::runtime_error("missing physical landing gimbal joint");}
      publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/landing/gimbal/joint_states", rclcpp::SensorDataQoS());
    } else if (mode_ == "pad") {
      pad_ = model_->GetLink("pad");
      if (!pad_ || model_->IsStatic() || !pad_->GetKinematic()) {
        throw std::runtime_error("moving pad requires an explicit dynamic kinematic link");
      }
      start_x_ = pad_->WorldPose().Pos().X();
      service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/landing_fixture/start_motion", [this](
          std_srvs::srv::Trigger::Request::SharedPtr,
          std_srvs::srv::Trigger::Response::SharedPtr response) {
          // One-way explicit start; observers cannot reset physical history.
          response->success = !moving_.exchange(true);
          response->message = response->success ? "started bounded 0.02 m/s motion" : "already started";
        });
    } else {throw std::runtime_error("unknown landing fixture mode");}
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo & info) {update(info);});
  }

private:
  void update(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    if (mode_ == "pad") {
      const double travel = pad_->WorldPose().Pos().X() - start_x_;
      const bool enabled = moving_.load() && travel < 2.0;
      // Low-speed kinematic bodies can otherwise auto-sleep despite a command.
      // Wake the physical body; motion still comes only from velocity integration.
      pad_->SetEnabled(true);
      pad_->SetLinearVel(ignition::math::Vector3d(enabled ? 0.02 : 0, 0, 0));
      pad_->SetAngularVel(ignition::math::Vector3d::Zero);
      return;
    }
    const double angle = joint_->Position(0), velocity = joint_->GetVelocity(0);
    const double command = 0.14 * std::sin(0.8 * now);
    joint_->SetForce(0, std::clamp(0.05 * (command - angle) - 0.004 * velocity, -0.02, 0.02));
    if (now - published_ < 0.009999 || now < published_) {return;}
    published_ = now;
    sensor_msgs::msg::JointState out;
    out.header.stamp.sec = info.simTime.sec;
    out.header.stamp.nanosec = info.simTime.nsec;
    out.header.frame_id = "base_link";
    out.name = {"landing_gimbal_pitch"};
    out.position = {angle};
    out.velocity = {velocity};
    publisher_->publish(out);
  }

  gazebo_ros::Node::SharedPtr node_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::LinkPtr pad_;
  gazebo::event::ConnectionPtr update_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::string mode_;
  std::atomic<bool> moving_{false};
  double published_{0}, start_x_{0};
};
GZ_REGISTER_MODEL_PLUGIN(LandingFixture)
}  // namespace easyfly_gazebo
