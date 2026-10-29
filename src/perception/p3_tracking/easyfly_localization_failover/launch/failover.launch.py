# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Start the independent backup and opt-in manager beside a FAST-LIO SITL session."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    if LaunchConfiguration('enable_external_vision').perform(context) != 'true':
        raise ValueError('explicit enable_external_vision:=true required')
    share = Path(get_package_share_directory('easyfly_openvins_integration'))
    # The strict native queue admission and deferred initial camera subscription
    # are retained without starting reference alignment, TF or a navigation owner.
    return [
        Node(package='easyfly_localization_failover', executable='sitl_clock_witness',
             output='screen', parameters=[{'use_sim_time':True,
                 'px4_pid':int(LaunchConfiguration('px4_pid').perform(context)),
                 'px4_binary':LaunchConfiguration('px4_binary').perform(context),
                 'px4_rootfs':LaunchConfiguration('px4_rootfs').perform(context)}]),
        Node(package='easyfly_openvins_integration', executable='openvins_guard',
             namespace='openvins', name='openvins_guard', output='screen',
             parameters=[str(share/'config/guard.yaml'), {'use_sim_time':True,
                          'defer_camera_until_calibration':True}]),
        Node(package='ov_msckf', executable='run_subscribe_msckf', namespace='openvins/raw',
             arguments=[str(share/'config/gazebo_mono/estimator_config_navigation.yaml')], output='screen',
             parameters=[{'use_sim_time':True, 'strict_input_timing':True,
                          'publish_global_to_imu_tf':False, 'publish_calibration_tf':False}]),
        Node(package='easyfly_localization_failover', executable='localization_failover',
             output='screen', parameters=[{'use_sim_time':True, 'enable_external_vision':True}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('px4_pid'), DeclareLaunchArgument('px4_binary'),
                              DeclareLaunchArgument('px4_rootfs'),
                              DeclareLaunchArgument('enable_external_vision', default_value='false',
                              choices=['false', 'true']), OpaqueFunction(function=start)])
