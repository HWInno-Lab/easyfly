// EasyFly derivative of gazebo_ros_pkgs 3.9.0; see NOTICE.
// Copyright 2018 Open Source Robotics Foundation, Inc.
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


#include <builtin_interfaces/msg/time.hpp>
#include "easyfly_gazebo_plugins/safe_imu.hpp"
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <sensor_msgs/msg/imu.hpp>

#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <time.h>
#include <unistd.h>

namespace gazebo_plugins
{

namespace
{
int64_t MonotonicNs()
{
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
}

struct ImuTimingSample
{
  int64_t source_ns = 0;
  int64_t entry_ns = 0;
  int64_t snapshot_end_ns = 0;
  int64_t publish_begin_ns = 0;
  int64_t publish_end_ns = 0;
  int result = 0;  // 0 = uninitialized, 1 = identical snapshot, 2 = published.
};
}  // namespace

class EasyFlyGazeboRosImuSensorPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for imu message
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  /// IMU message modified each update
  sensor_msgs::msg::Imu::SharedPtr msg_;
  /// IMU sensor this plugin is attached to
  gazebo::sensors::ImuSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  std::string last_snapshot_;
  std::string timing_path_;
  std::vector<ImuTimingSample> timings_;
  std::mutex timing_mutex_;
  size_t timings_discarded_ = 0;
  void RecordTiming(const ImuTimingSample & sample);
  void WriteTimings();
  /// Publish latest imu data to ROS
  void OnUpdate();
};

EasyFlyGazeboRosImuSensor::EasyFlyGazeboRosImuSensor()
: impl_(std::make_unique<EasyFlyGazeboRosImuSensorPrivate>())
{
}

EasyFlyGazeboRosImuSensor::~EasyFlyGazeboRosImuSensor()
{
  impl_->sensor_update_event_.reset();
  impl_->WriteTimings();
}

void EasyFlyGazeboRosImuSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
  if (!impl_->ros_node_) {
    RCLCPP_ERROR(rclcpp::get_logger("easyfly_gazebo_imu"), "IMU ROS node creation failed");
    return;
  }

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not an imu sensor. Exiting.");
    return;
  }

  bool initial_orientation_as_reference = false;
  if (!_sdf->HasElement("initial_orientation_as_reference")) {
    RCLCPP_INFO_STREAM(
      impl_->ros_node_->get_logger(),
      "<initial_orientation_as_reference> is unset, using default value of false "
      "to comply with REP 145 (world as orientation reference)");
  } else {
    initial_orientation_as_reference = _sdf->Get<bool>("initial_orientation_as_reference");
  }

  if (initial_orientation_as_reference) {
    RCLCPP_WARN_STREAM(
      impl_->ros_node_->get_logger(),
      "<initial_orientation_as_reference> set to true, this behavior is deprecated "
      "as it does not comply with REP 145.");
  } else {
    // This complies with REP 145
    impl_->sensor_->SetWorldToReferenceOrientation(ignition::math::Quaterniond::Identity);
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
    "~/out", qos.get_publisher_qos("~/out", rclcpp::QoS(1000).reliable()));
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "IMU measurement snapshots: actual publisher history depth=%zu",
    impl_->pub_->get_actual_qos().depth());

  // Create message to be reused
  auto msg = std::make_shared<sensor_msgs::msg::Imu>();

  // Get frame for message
  msg->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Fill covariances
  // TODO(anyone): covariance for IMU's orientation once this is added to gazebo
  using SNT = gazebo::sensors::SensorNoiseType;
  msg->angular_velocity_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_X_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S));
  msg->linear_acceleration_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));

  impl_->msg_ = msg;

  // Explicit diagnostic opt-in only. Buffer a bounded trace; no per-sample file
  // I/O or extra ROS traffic in the sensor callback. Never change sample stamps.
  const auto trace_prefix = std::getenv("EASYFLY_IMU_TIMING_TRACE");
  if (trace_prefix && trace_prefix[0] == '/') {
    impl_->timing_path_ = std::string(trace_prefix) + "." + std::to_string(getpid()) +
      "." + impl_->ros_node_->get_name() + ".csv";
    impl_->timings_.reserve(60000);
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "IMU timing trace: %s",
      impl_->timing_path_.c_str());
  }

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&EasyFlyGazeboRosImuSensorPrivate::OnUpdate, impl_.get()));
}

void EasyFlyGazeboRosImuSensorPrivate::OnUpdate()
{
  ImuTimingSample timing;
  if (!timing_path_.empty()) {
    timing.entry_ns = MonotonicNs();
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("EasyFlyGazeboRosImuSensorPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  // One mutex-protected measurement snapshot. LastUpdateTime is the scheduler
  // time, and does not advance in Gazebo strict-rate mode; it is not sample time.
  const auto snapshot = sensor_->ImuMessage();
  if (timing.entry_ns) {
    timing.snapshot_end_ns = MonotonicNs();
    timing.source_ns = static_cast<int64_t>(snapshot.stamp().sec()) * 1000000000LL +
      snapshot.stamp().nsec();
  }
  if (!snapshot.has_stamp() || (snapshot.stamp().sec() == 0 && snapshot.stamp().nsec() == 0)) {
    RecordTiming(timing);
    return;  // Sensor has not produced its first measurement.
  }
  const auto serialized = snapshot.SerializeAsString();
  if (serialized == last_snapshot_) {
    timing.result = 1;
    RecordTiming(timing);
    return;  // No new physical measurement; do not fabricate another sample.
  }
  last_snapshot_ = serialized;
  msg_->header.stamp.sec = snapshot.stamp().sec();
  msg_->header.stamp.nanosec = snapshot.stamp().nsec();
  msg_->orientation.x = snapshot.orientation().x();
  msg_->orientation.y = snapshot.orientation().y();
  msg_->orientation.z = snapshot.orientation().z();
  msg_->orientation.w = snapshot.orientation().w();
  msg_->angular_velocity.x = snapshot.angular_velocity().x();
  msg_->angular_velocity.y = snapshot.angular_velocity().y();
  msg_->angular_velocity.z = snapshot.angular_velocity().z();
  msg_->linear_acceleration.x = snapshot.linear_acceleration().x();
  msg_->linear_acceleration.y = snapshot.linear_acceleration().y();
  msg_->linear_acceleration.z = snapshot.linear_acceleration().z();
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("Publish");
#endif
  // Publish message
  if (timing.entry_ns) {
    timing.publish_begin_ns = MonotonicNs();
  }
  pub_->publish(*msg_);
  if (timing.entry_ns) {
    timing.publish_end_ns = MonotonicNs();
    timing.result = 2;
    RecordTiming(timing);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

void EasyFlyGazeboRosImuSensorPrivate::RecordTiming(const ImuTimingSample & sample)
{
  if (!sample.entry_ns) {
    return;
  }
  std::lock_guard<std::mutex> lock(timing_mutex_);
  if (timings_.size() < 60000) {
    timings_.push_back(sample);
  } else {
    ++timings_discarded_;
  }
}

void EasyFlyGazeboRosImuSensorPrivate::WriteTimings()
{
  if (timing_path_.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(timing_mutex_);
  std::ofstream stream(timing_path_);
  stream << "# CLOCK_MONOTONIC capacity=60000 discarded=" << timings_discarded_ << '\n';
  stream << "source_ns,entry_ns,snapshot_end_ns,publish_begin_ns,publish_end_ns,result\n";
  for (const auto & sample : timings_) {
    stream << sample.source_ns << ',' << sample.entry_ns << ',' << sample.snapshot_end_ns << ',' <<
      sample.publish_begin_ns << ',' << sample.publish_end_ns << ',' << sample.result << '\n';
  }
  stream.close();
  if (!stream) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "Cannot write IMU timing trace: %s",
      timing_path_.c_str());
  }
}

GZ_REGISTER_SENSOR_PLUGIN(EasyFlyGazeboRosImuSensor)

}  // namespace gazebo_plugins
