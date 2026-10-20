"""An observed OpenVINS pose cannot authorize navigation before frame/map integration."""
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy
from easyfly_msgs.action import NavigateTo, FollowTrajectory
from easyfly_msgs.msg import LocalizationStatus
from easyfly_msgs.srv import SetNavigationSession


@pytest.mark.parametrize('backend,strategy', [
    ('openvins', 'astar'), ('openvins', 'ego'), ('openvins', 'combined'),
    ('disabled', 'astar')])
def test_unaligned_backend_rejects_navigation_even_with_healthy_looking_status(backend, strategy):
    rclpy.init()
    node = rclpy.create_node('localization_admission_mock')
    children = []

    def admit(request):
        children.append(request)
        return GoalResponse.REJECT

    servers = [ActionServer(node, NavigateTo, '/navigate_astar',
                            execute_callback=lambda _: None, goal_callback=admit),
               ActionServer(node, FollowTrajectory, '/follow_trajectory',
                            execute_callback=lambda _: None, goal_callback=admit)]
    service = node.create_service(SetNavigationSession, '/planning/navigation_session',
                                  lambda _, response: response)
    publisher = node.create_publisher(
        LocalizationStatus, '/localization/openvins/status',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    process = subprocess.Popen([
        'ros2', 'launch', 'easyfly_global_planner', 'navigate.launch.py',
        'use_sim_time:=false', 'localization_backend:=' + backend, 'strategy:=' + strategy],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def wait(future):
        deadline = time.monotonic() + 6
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert future.done()
        return future.result()

    try:
        client = ActionClient(node, NavigateTo, '/navigate')
        assert client.wait_for_server(timeout_sec=8)
        status = LocalizationStatus(backend_id='openvins', session_id='test-session',
                                    state=LocalizationStatus.ESTIMATING, pose_valid=True,
                                    covariance_available=True, tracked_frame='livox_imu_frame')
        status.header.frame_id = 'openvins_odom'
        status.header.stamp = node.get_clock().now().to_msg()
        status.pose_stamp = status.header.stamp
        publisher.publish(status)
        goal = NavigateTo.Goal(target_x=1., target_y=0., target_z=1., max_speed=1.)
        assert not wait(client.send_goal_async(goal)).accepted
        parameters = node.create_client(SetParameters, '/navigation_coordinator/set_parameters')
        assert parameters.wait_for_service(timeout_sec=5)
        change = Parameter('localization_backend', Parameter.Type.STRING, 'fast_lio')
        result = wait(parameters.call_async(SetParameters.Request(
            parameters=[change.to_parameter_msg()])))
        assert not result.results[0].successful
        assert not wait(client.send_goal_async(goal)).accepted
        assert not children
        names = {name for name, _ in node.get_node_names_and_namespaces()}
        assert 'easyfly_global_planner' not in names
        assert 'obstacle_supervisor' not in names
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
            pytest.fail('navigation launch did not stop on SIGINT')
        for server in servers:
            server.destroy()
        node.destroy_service(service)
        node.destroy_node()
        rclpy.shutdown()
