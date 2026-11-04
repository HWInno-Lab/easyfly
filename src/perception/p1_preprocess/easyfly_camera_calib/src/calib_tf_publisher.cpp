// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_camera_calib/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

// Configured extrinsics, not an online calibration solver. Validate ALL before TF publication.
#include <tf2_ros/static_transform_broadcaster.h>
#include "easyfly_camera_calib/calibration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace easyfly_camera_calib
{
class CalibTfPublisher : public rclcpp::Node
{
public:
  explicit CalibTfPublisher(const rclcpp::NodeOptions & options)
  : Node("calib_tf_publisher", options)
  {
    rcl_interfaces::msg::ParameterDescriptor frozen; frozen.read_only = true;
    auto file = declare_parameter<std::string>("calib_file", "", frozen);
    if (file.empty()) {
      file = ament_index_cpp::get_package_share_directory("easyfly_camera_calib") +
        "/config/sensors.yaml";
    }
    const auto calibration = loadCalibration(file);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (const auto & m : calibration.mounts) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now(); tf.header.frame_id = m.parent; tf.child_frame_id = m.child;
      tf.transform.translation.x = m.translation[0]; tf.transform.translation.y = m.translation[1];
      tf.transform.translation.z = m.translation[2];
      const double cr = std::cos(m.rotation_rpy[0] * .5), sr = std::sin(m.rotation_rpy[0] * .5);
      const double cp = std::cos(m.rotation_rpy[1] * .5), sp = std::sin(m.rotation_rpy[1] * .5);
      const double cy = std::cos(m.rotation_rpy[2] * .5), sy = std::sin(m.rotation_rpy[2] * .5);
      tf.transform.rotation.x = sr * cp * cy - cr * sp * sy;
      tf.transform.rotation.y = cr * sp * cy + sr * cp * sy;
      tf.transform.rotation.z = cr * cp * sy - sr * sp * cy;
      tf.transform.rotation.w = cr * cp * cy + sr * sp * sy;
      transforms.push_back(tf);
    }
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(get_logger(), "Validated %zu configured static transforms", transforms.size());
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};
}  // namespace easyfly_camera_calib
RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_camera_calib::CalibTfPublisher)
