# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_gimbal_control/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""Exercise real process/services, target freshness and graceful exits."""
import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import GimbalState, Target3D, Target3DArray, UAVState
from easyfly_msgs.srv import SetGimbalAngle, SetGimbalTracking
import pytest
import rclpy
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool


@pytest.mark.parametrize("exit_signal", [signal.SIGINT, signal.SIGTERM])
def test_live_gimbal_and_shutdown(exit_signal):
    """Use actual ROS endpoints and positive frozen ROS time."""
    executable = os.path.join(
        get_package_prefix("easyfly_gimbal_control"),
        "lib/easyfly_gimbal_control/gimbal_control_node",
    )
    process = subprocess.Popen(
        [
            executable,
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "observation_timeout_s:=0.25",
            "-p",
            "connection_timeout_s:=0.15",
        ]
    )
    rclpy.init()
    node = rclpy.create_node("gimbal_test_observer")
    states = []
    node.create_subscription(GimbalState, "/gimbal/state", states.append, 10)
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    uav_pub = node.create_publisher(UAVState, "/uav_state", 10)
    target_pub = node.create_publisher(Target3DArray, "/target_positions", 10)
    angle = node.create_client(SetGimbalAngle, "/gimbal/set_angle")
    tracking = node.create_client(SetGimbalTracking, "/gimbal/set_tracking")
    connection = node.create_client(SetBool, "/gimbal/simulate_connection")
    source = True
    advancing = True
    ros_ns = 10_000_000_000
    observed_id = 7
    observed_frame = "odom"
    stamp_offset = -20_000_000
    pose_valid = True

    def pump(duration):
        nonlocal ros_ns
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if advancing:
                ros_ns += 10_000_000
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(
                ros_ns, 1_000_000_000
            )
            clock_pub.publish(clock)
            if source:
                uav = UAVState()
                uav.header.stamp.sec, uav.header.stamp.nanosec = divmod(
                    ros_ns + stamp_offset, 1_000_000_000
                )
                uav.header.frame_id = observed_frame
                uav.connected = uav.pose_valid = uav.attitude_valid = True
                uav.pose_valid = pose_valid
                uav_pub.publish(uav)
                batch = Target3DArray()
                batch.header = uav.header
                target = Target3D()
                target.header = uav.header
                target.track_id = observed_id
                target.position_valid = True
                target.position_x = target.position_y = 10.0
                target.confidence = 1.0
                batch.targets = [target]
                target_pub.publish(batch)
            rclpy.spin_once(node, timeout_sec=0.01)

    def call(client, request):
        future = client.call_async(request)
        end = time.monotonic() + 2
        while not future.done() and time.monotonic() < end:
            pump(0.02)
        assert future.done()
        return future.result()

    try:
        for client in [angle, tracking, connection]:
            assert client.wait_for_service(timeout_sec=5)
        pump(0.4)
        assert states[-1].is_connected
        request = SetGimbalAngle.Request()
        request.yaw_angle = 181.0
        assert not call(angle, request).success
        request.yaw_angle = 30.0
        assert call(angle, request).success
        pump(0.1)
        assert 0 < states[-1].yaw_angle < 30
        follow = SetGimbalTracking.Request()
        follow.enabled = True
        follow.target_id = -1
        assert not call(tracking, follow).success
        follow.target_id = 7
        for bad_frame, bad_offset, bad_pose in [
            ("map", -20_000_000, True),
            ("odom", -1_000_000_000, True),
            ("odom", 1_000_000_000, True),
            ("odom", -20_000_000, False),
        ]:
            observed_frame, stamp_offset, pose_valid = (
                bad_frame,
                bad_offset,
                bad_pose,
            )
            pump(0.05)
            assert not call(tracking, follow).success
        observed_frame, stamp_offset, pose_valid = "odom", -20_000_000, True
        pump(0.05)
        assert call(tracking, follow).success
        pump(0.05)
        assert states[-1].is_tracking
        assert states[-1].tracking_target_id == 7
        observed_id = 8
        pump(0.08)
        assert not states[-1].is_tracking
        assert states[-1].tracking_target_id == -1
        observed_id = 7
        pump(0.05)
        assert not states[-1].is_tracking
        assert call(tracking, follow).success
        assert call(angle, request).success
        pump(0.03)
        assert not states[-1].is_tracking
        follow.enabled = False
        assert call(tracking, follow).success
        pump(0.05)
        held = states[-1].yaw_angle
        pump(0.05)
        assert states[-1].yaw_angle == held
        follow.enabled = True
        assert call(tracking, follow).success
        source = False
        advancing = False
        pump(0.4)
        assert not states[-1].is_tracking
        stopped = states[-1].yaw_angle
        pump(0.1)
        assert states[-1].yaw_angle == stopped
        source = advancing = True
        pump(0.1)
        assert not states[-1].is_tracking
        assert states[-1].yaw_angle == stopped
        fault = SetBool.Request()
        fault.data = False
        assert call(connection, fault).success
        pump(0.25)
        assert not states[-1].is_connected
        assert not call(angle, request).success
        fault.data = True
        assert call(connection, fault).success
        pump(0.05)
        assert states[-1].is_connected
        assert states[-1].yaw_angle == stopped
        process.send_signal(exit_signal)
        assert process.wait(timeout=3) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
