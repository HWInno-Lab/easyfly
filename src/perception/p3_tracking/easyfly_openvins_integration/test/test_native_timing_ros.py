# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual optional estimator boundary, synthetic timing fixtures; no VIO accuracy claim."""
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import pytest
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, Image


@pytest.mark.parametrize('fault', ['gap', 'duplicate', 'receipt', 'camera_age', 'startup_stale'])
def test_native_rejects_timing_fault_before_filter_and_exits_normally(tmp_path, fault, monkeypatch):
    try:
        prefix = Path(get_package_prefix('ov_msckf'))
    except PackageNotFoundError:
        pytest.skip('optional actual OpenVINS estimator not installed')
    executable = prefix/'lib/ov_msckf/run_subscribe_msckf'
    if not executable.is_file():
        pytest.skip('optional actual OpenVINS estimator not built')
    config = Path(get_package_share_directory('easyfly_openvins_integration'))/(
        'config/gazebo_mono/estimator_config_navigation.yaml')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('native_timing_contract_'+fault)
    clock = node.create_publisher(Clock, '/clock', 10)
    imu = node.create_publisher(Imu, '/openvins/input/imu', 1000)
    camera = node.create_publisher(Image, '/openvins/input/camera', 32)
    log = (tmp_path/'estimator.log').open('w')
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
    process = subprocess.Popen([
        str(executable), str(config), '--ros-args', '-p', 'use_sim_time:=true',
        '-p', 'strict_input_timing:=true', '-p', 'publish_global_to_imu_tf:=false',
        '-p', 'publish_calibration_tf:=false', '-r', '__ns:=/native_timing_fixture'],
        env=env, stdout=log, stderr=subprocess.STDOUT)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    def stamp(value):
        return rclpy.time.Time(nanoseconds=value).to_msg()

    def measurement(value):
        clock.publish(Clock(clock=stamp(value)))
        spin(.025)
        value_msg = Imu()
        value_msg.header.stamp, value_msg.header.frame_id = stamp(value), 'livox_imu_frame'
        value_msg.linear_acceleration.z = 9.8
        imu.publish(value_msg)
        spin(.025)

    try:
        deadline = time.monotonic()+8
        while imu.get_subscription_count() != 1 and time.monotonic() < deadline:
            clock.publish(Clock(clock=stamp(10_000_000_000)))
            spin(.05)
        assert imu.get_subscription_count() == 1
        # Establish fresh valid callback history before the isolated fault.
        for index in range(10):
            value = 10_000_000_000+index*5_000_000
            measurement(value)
            assert process.poll() is None
        if fault in ['camera_age', 'startup_stale']:
            image = Image(height=480, width=640, encoding='mono8', step=640,
                          data=bytes(640*480))
            image.header.frame_id = 'camera_color_optical_frame'
            if fault == 'startup_stale':
                image.header.stamp = stamp(value-600_000_000)
                camera.publish(image)
                spin(.05)
                assert process.poll() is None  # rejected history never seeds the estimator
            image.header.stamp = stamp(value-40_000_000)
            camera.publish(image)
            spin(.05)
            assert process.poll() is None
        if fault in ['gap', 'startup_stale']:
            measurement(value+30_000_000)
        elif fault == 'duplicate':
            measurement(value)
        elif fault == 'receipt':
            spin(.6)  # frozen source clock must not conceal a native callback stall
            measurement(value+5_000_000)
        else:
            clock.publish(Clock(clock=stamp(value+650_000_000)))
            spin(.025)
            image.header.stamp = stamp(value)  # increasing, but now 650ms old
            camera.publish(image)
        process.wait(timeout=5)
        log.flush()
        output = (tmp_path/'estimator.log').read_text()
        assert process.returncode == 0, output[-4000:]
        assert 'strict input timing failure' in output
        assert 'Assertion' not in output
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
                pytest.fail('native timing boundary required forced shutdown')
        log.close()
        node.destroy_node()
        rclpy.shutdown()
