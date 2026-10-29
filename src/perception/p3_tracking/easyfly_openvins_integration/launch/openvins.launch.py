# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Explicit, coupled guard + OpenVINS process lifetime. No default flight wiring."""

from pathlib import Path
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import SignalProcess
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def session_exit_handler(processes=None):
    # Both children may exit in the same executor cycle. Emit at most one
    # shutdown, and never emit another while an enclosing launch is stopping.
    sent = False

    def exited(event, context):
        nonlocal sent
        if sent or context.is_shutdown:
            return []
        sent = True
        if processes is not None:
            # A failed localization session must not kill PX4/its control bridge
            # in flight. Stop its own workers; stale admission drains navigation.
            return [EmitEvent(event=SignalProcess(signal_number=signal.SIGINT,
                               process_matcher=matches_action(process)))
                    for process in processes if process is not event.action]
        return [EmitEvent(event=Shutdown(
            reason='localization process exited; restart the whole session'))]

    return exited


def session_nodes(context):
    if LaunchConfiguration('sensor_profile').perform(context) != 'gazebo_mono':
        raise ValueError('only the calibrated gazebo_mono profile is available')
    share = Path(get_package_share_directory('easyfly_openvins_integration'))
    enable_navigation = LaunchConfiguration('enable_navigation', default='false').perform(context)
    if enable_navigation not in ('true', 'false'):
        raise ValueError('enable_navigation must be true or false')
    estimator_name = ('estimator_config_navigation.yaml' if enable_navigation == 'true'
                      else 'estimator_config.yaml')
    estimator_config = share / 'config/gazebo_mono' / estimator_name
    guard_config = share / 'config/guard.yaml'
    for filename in [estimator_config, guard_config]:
        if not filename.is_file():
            raise ValueError('missing installed configuration: ' + str(filename))
    guard = Node(package='easyfly_openvins_integration', executable='openvins_guard',
                 namespace='openvins', name='openvins_guard',
                 parameters=[str(guard_config), {'use_sim_time': True,
                     'defer_camera_until_calibration': enable_navigation == 'true'}],
                 output='screen')
    estimator = Node(package='ov_msckf', executable='run_subscribe_msckf',
                     namespace='openvins/raw', arguments=[str(estimator_config)],
                     parameters=[{'use_sim_time': True, 'publish_global_to_imu_tf': False,
                                  'strict_input_timing': enable_navigation == 'true',
                                  'publish_calibration_tf': False}], output='screen')
    processes = [guard, estimator]
    start_alignment = LaunchConfiguration('start_alignment', default='false').perform(context)
    if start_alignment not in ('true', 'false'):
        raise ValueError('start_alignment must be true or false')
    if enable_navigation == 'true' and start_alignment != 'true':
        raise ValueError('navigation requires source-time alignment')
    if start_alignment == 'true':
        alignment_config = share / 'config/alignment_gazebo.yaml'
        if not alignment_config.is_file():
            raise ValueError('missing installed alignment configuration')
        processes.append(Node(
            package='easyfly_openvins_integration', executable='openvins_alignment',
            namespace='openvins', name='openvins_alignment',
            parameters=[str(alignment_config), {'use_sim_time': True}], output='screen'))
    if enable_navigation == 'true':
        processes.append(Node(
            package='easyfly_openvins_integration', executable='openvins_navigation',
            namespace='openvins', name='openvins_navigation',
            parameters=[{'use_sim_time': True}], output='screen'))
    exited = session_exit_handler(processes if enable_navigation == 'true' else None)
    handlers = [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=exited))
                for process in processes]
    return [*handlers, *processes]


def startup_exit_handler(session_actions):
    handled = False

    def exited(event, context):
        nonlocal handled
        if handled or context.is_shutdown:
            return []
        handled = True
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='OpenVINS startup readiness failed'))]
        return session_actions

    return exited


def nodes(context):
    wait = LaunchConfiguration('wait_for_inputs', default='false').perform(context)
    if wait == 'false':
        return session_nodes(context)
    if wait != 'true':
        raise ValueError('wait_for_inputs must be true or false')
    if LaunchConfiguration('sensor_profile').perform(context) != 'gazebo_mono':
        raise ValueError('only the calibrated gazebo_mono profile is available')
    share = Path(get_package_share_directory('easyfly_openvins_integration'))
    # Resolve inside the include's configuration scope. Process-exit events run
    # after that scope has been popped from the enclosing launch context.
    session_actions = session_nodes(context)
    observer = Node(package='easyfly_openvins_integration', executable='openvins_startup',
                    namespace='openvins', name='openvins_startup',
                    parameters=[str(share / 'config/guard.yaml'), {'use_sim_time': True}],
                    output='screen')
    return [RegisterEventHandler(OnProcessExit(
        target_action=observer, on_exit=startup_exit_handler(session_actions))), observer]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_profile', default_value='gazebo_mono',
                              choices=['gazebo_mono'],
                              description='Installed simulation profile; ROS simulation clock'),
        DeclareLaunchArgument('start_alignment', default_value='false', choices=['true', 'false'],
                              description='Source-stamped PX4 reference; observation only'),
        DeclareLaunchArgument('wait_for_inputs', default_value='false', choices=['true', 'false'],
                              description='Bounded managed Gazebo startup readiness'),
        DeclareLaunchArgument('enable_navigation', default_value='false', choices=['true', 'false'],
                              description='Simulation-only session-pinned navigation adapter'),
        OpaqueFunction(function=nodes),
    ])
