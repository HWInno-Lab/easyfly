# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("easyfly_fast_livo2_integration"))
    topics = [
        "pose",
        "status",
        "base_pose",
        "aligned_pose",
        "alignment_status",
        "navigation_state",
        "navigation_status",
    ]
    remaps = [("/localization/openvins/" + t, "/localization/fast_livo2/" + t) for t in topics]
    remaps += [("/openvins/raw/visual_tracking", "/fastlivo/raw/visual_tracking")]
    params = {"use_sim_time": True, "source_backend": "fast_livo2"}
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("lidar_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("enable_atlas", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument(
                "enable_navigation", default_value="false", choices=["true", "false"]
            ),
            Node(
                package="easyfly_fast_livo2_integration",
                executable="fastlivo_guard",
                parameters=[
                    {
                        "use_sim_time": True,
                        **{
                            k + "_topic": LaunchConfiguration(k + "_topic")
                            for k in ["camera", "imu", "lidar"]
                        },
                    }
                ],
                output="screen",
            ),
            Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                namespace="fastlivo/raw",
                parameters=[str(share / "config/gazebo.yaml"), {"use_sim_time": True,
                    "publish.atlas_keyframes": ParameterValue(LaunchConfiguration("enable_atlas"), value_type=bool)}],
                output="screen",
                remappings=[
                    ("/aft_mapped_to_init", "/fastlivo/raw/odometry"),
                    ("/cloud_registered", "/fastlivo/raw/cloud"),
                    ("/mavros/vision_pose/pose", "/fastlivo/raw/unused_vision_pose"),
                    *[
                        (topic, "/fastlivo/raw" + topic)
                        for topic in [
                            "/tf",
                            "/tf_static",
                            "/path",
                            "/visualization_marker",
                            "/cloud_visual_sub_map_before",
                            "/cloud_effected",
                            "/Laser_map",
                            "/planner_normal",
                            "/voxels",
                            "/dyn_obj",
                            "/dyn_obj_removed",
                            "/dyn_obj_dbg_hist",
                            "/LIVO2/imu_propagate",
                            "/planes",
                        ]
                    ],
                ],
            ),
            Node(
                package="easyfly_openvins_integration",
                executable="openvins_alignment",
                name="fastlivo_alignment",
                parameters=[
                    dict(
                        params,
                        source_pose_topic="/localization/fast_livo2/pose",
                        source_status_topic="/localization/fast_livo2/status",
                        source_frame="fastlivo_odom",
                    )
                ],
                remappings=remaps,
                output="screen",
            ),
            Node(
                package="easyfly_openvins_integration",
                executable="openvins_navigation",
                name="fastlivo_navigation",
                parameters=[params],
                remappings=remaps,
                condition=IfCondition(LaunchConfiguration("enable_navigation")),
                output="screen",
            ),
        ]
    )
