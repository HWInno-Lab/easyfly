# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('easyfly_orbslam3_integration'))
    core = Path(get_package_share_directory('easyfly_orbslam3_core'))
    topics = ['pose', 'status', 'base_pose', 'aligned_pose', 'alignment_status',
              'navigation_state', 'navigation_status']
    remaps = [('/localization/openvins/'+t, '/localization/orbslam3/'+t) for t in topics]
    remaps.append(('/openvins/raw/visual_tracking', '/orbslam3/raw/visual_tracking'))
    common = dict(use_sim_time=True, source_backend='orbslam3')
    return LaunchDescription([
        DeclareLaunchArgument('runtime_directory'),
        DeclareLaunchArgument('sensor_mode', default_value='rgbd', choices=['rgbd', 'rgbd_imu']),
        DeclareLaunchArgument('settings_template', default_value=PythonExpression(["'"+str(share)+"/config/gazebo_' + '", LaunchConfiguration('sensor_mode'), "' + '.yaml'"])),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('save_atlas', default_value=''),
        DeclareLaunchArgument('load_atlas', default_value=''),
        DeclareLaunchArgument('enable_navigation', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/aligned_depth_to_color/depth/image_raw'),
        Node(package='easyfly_orbslam3_integration', executable='orbslam3_rgbd',
             parameters=[dict(use_sim_time=True,
                              require_navigation_admission=ParameterValue(PythonExpression([
                                  "'", LaunchConfiguration('sensor_mode'), "' == 'rgbd_imu' and '",
                                  LaunchConfiguration('enable_navigation'), "' == 'true'"]), value_type=bool),
                              settings_template=ParameterValue(LaunchConfiguration('settings_template'), value_type=str),
                              vocabulary=str(core/'vocabulary/ORBvoc.txt'),
                              **{k:ParameterValue(LaunchConfiguration(k), value_type=str) for k in ['runtime_directory', 'save_atlas', 'load_atlas', 'rgb_topic', 'depth_topic', 'sensor_mode', 'imu_topic']})],
             output='screen'),
        Node(package='easyfly_openvins_integration', executable='openvins_alignment',
             name='orbslam3_alignment', remappings=remaps, output='screen',
             parameters=[dict(common, require_source_covariance=ParameterValue(
                              PythonExpression(["'", LaunchConfiguration('sensor_mode'), "' == 'rgbd_imu'"]), value_type=bool),
                              source_pose_topic='/localization/orbslam3/pose',
                              source_status_topic='/localization/orbslam3/status',
                              source_frame='orbslam3_map', source_body='base_link',
                              body_to_base_translation=[0.,0.,0.],
                              body_to_base_orientation=[0.,0.,0.,1.])]),
        Node(package='easyfly_openvins_integration', executable='openvins_navigation',
             name='orbslam3_navigation', remappings=remaps, output='screen',
             parameters=[dict(common, minimum_visual_tracks=30)],
             condition=IfCondition(LaunchConfiguration('enable_navigation'))),
    ])
