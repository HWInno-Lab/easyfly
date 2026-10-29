# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Synthetic analytical encoder/optical inputs through the installed ROS node."""

import math
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import LandingGuidanceTarget, LandingTarget, UAVState
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def test_dynamic_mount_original_time_geometry_expiry_and_identity(tmp_path):
    """The actual converter must undo the observed moving camera rotation."""
    rclpy.init()
    node = rclpy.create_node('dynamic_mount_fixture')
    clocks = node.create_publisher(Clock, '/clock', 10)
    states = node.create_publisher(UAVState, '/uav_state', 10)
    targets = node.create_publisher(LandingTarget, '/landing_target', qos_profile_sensor_data)
    mounts = node.create_publisher(TransformStamped, '/landing/camera_mount',
                                   qos_profile_sensor_data)
    seen = []
    node.create_subscription(LandingGuidanceTarget, '/landing/guidance_target', seen.append, 10)
    executable = Path(get_package_prefix('easyfly_precision_landing')) / \
        'lib/easyfly_precision_landing/landing_transform_node'
    args = [str(executable), '--ros-args', '-p', 'use_sim_time:=true',
            '-p', 'configured:=true', '-p', 'dynamic_camera:=true',
            '-p', 'selected_tag_id:=7', '-p', 'tag_size_m:=0.2',
            '-p', 'landing_plane_z_m:=0.0']
    log = (tmp_path / 'dynamic.log').open('w')
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
    tick = 0
    original = set()

    def drive(with_mount=True):
        nonlocal tick
        tick += 1
        ns = 10000000000 + tick * 10000000
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(ns, 1000000000)
        clocks.publish(clock)
        mount = TransformStamped()
        mount.header.frame_id = 'base_link'
        mount.child_frame_id = 'camera_down_optical_frame'
        mount.header.stamp = clock.clock
        mount.transform.translation.z = -.08
        # R_y(theta) times the calibrated down-camera orientation.
        theta = min(.2, tick * .004)
        root = math.sqrt(.5)
        q = (root * math.cos(theta / 2), -root * math.cos(theta / 2),
             -root * math.sin(theta / 2), root * math.sin(theta / 2))
        (mount.transform.rotation.x, mount.transform.rotation.y,
         mount.transform.rotation.z, mount.transform.rotation.w) = q
        if with_mount:
            mounts.publish(mount)
        source = ns - 5000000
        state = UAVState()
        state.header.frame_id = 'odom'
        state.header.stamp.sec, state.header.stamp.nanosec = divmod(source, 1000000000)
        state.connected = state.pose_valid = state.attitude_valid = True
        state.position_z = 1.
        states.publish(state)
        angle = min(.2, (tick - .5) * .004)
        target = LandingTarget()
        target.header = state.header
        target.header.frame_id = mount.child_frame_id
        target.tag_id, target.tag_size_m, target.valid = 7, .2, True
        target.pose.position.y = -.92 * math.sin(angle)
        target.pose.position.z = target.optical_depth_m = .92 * math.cos(angle)
        c, s = math.cos(angle / 2), math.sin(angle / 2)
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = (-root*c, root*c,
                                                                  root*s, root*s)
        target.reprojection_error_px = .1
        targets.publish(target)
        original.add(source)
        rclpy.spin_once(node, timeout_sec=.005)
        time.sleep(.01)

    def until(predicate, budget=4., with_mount=True):
        deadline = time.monotonic() + budget
        while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'dynamic.log').read_text()
            drive(with_mount)
            if predicate():
                return
        raise AssertionError((tmp_path / 'dynamic.log').read_text())

    try:
        until(lambda: len([v for v in seen if v.valid]) >= 8)
        good = [v for v in seen if v.valid]
        for value in good:
            assert value.reason == 'dynamic_camera_original_time_paired_observation'
            assert math.hypot(value.position.x, value.position.y, value.position.z) < 1e-6
            assert abs(value.plane_normal.z - 1.) < 1e-6
            assert value.header.stamp.sec * 1000000000 + value.header.stamp.nanosec in original
        seen.clear()
        until(lambda: tick > 100, budget=3., with_mount=False)
        seen.clear()
        for _ in range(35):
            drive(False)
        assert seen and not any(v.valid for v in seen)
        until(lambda: any(v.valid for v in seen))
        node.destroy_publisher(mounts)
        mounts = node.create_publisher(TransformStamped, '/landing/camera_mount',
                                       qos_profile_sensor_data)
        seen.clear()
        until(lambda: any('source_changed' in v.reason for v in seen))
        seen.clear()
        for _ in range(20):
            drive()
        assert seen and not any(v.valid for v in seen)
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=8)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    assert process.returncode == 0
