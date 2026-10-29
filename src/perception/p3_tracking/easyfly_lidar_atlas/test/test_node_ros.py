# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""ROS contract fixture; synthetic geometry is not native flight acceptance."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from test_geometry import room


@pytest.mark.parametrize('simulation_clock', [False, True])
def test_real_ros_identity_persistence_and_stale_latch(tmp_path, simulation_clock):
    rclpy.init()
    source = rclpy.create_node('laser_mapping')
    clock_pub = source.create_publisher(Clock, '/clock', 5) if simulation_clock else None
    clock_ns = 10_000_000_000
    for name, value in [('mapping.extrinsic_T', [.011,.02329,.00588]),
                        ('mapping.extrinsic_R', [1.,0.,0.,0.,1.,0.,0.,0.,1.]),
                        ('preprocess.lidar_type', 1), ('mapping.acc_cov', .1),
                        ('publish.scan_publish_en', True), ('publish.scan_bodyframe_pub_en', True),
                        ('mapping.gyr_cov', .1)]:
        source.declare_parameter(name, value)
    observer = rclpy.create_node('atlas_contract_observer')
    cloud_pub = source.create_publisher(PointCloud2, '/cloud_registered_body', 5)
    odom_pub = source.create_publisher(Odometry, '/Odometry', 5)
    states, poses = [], []
    observer.create_subscription(DiagnosticArray, '/localization/lidar_atlas/status',
        lambda message: states.append(json.loads(message.status[0].values[0].value)), 5)
    observer.create_subscription(PoseStamped, '/localization/lidar_atlas/pose',
        lambda message: poses.append(message), 5)
    client = observer.create_client(Trigger, '/localization/lidar_atlas/save')
    executable = Path(get_package_prefix('easyfly_lidar_atlas'))/'lib/easyfly_lidar_atlas/lidar_atlas'
    log = (tmp_path/'node.log').open('x')
    process = subprocess.Popen([str(executable), '--ros-args', '-p', 'use_sim_time:='+str(simulation_clock).lower(),
        '-p', 'save_atlas:='+str(tmp_path/'atlas')], stdout=log, stderr=subprocess.STDOUT)
    cloud = PointCloud2()
    cloud.header.frame_id = 'body'
    values = room(73).astype('<f4')
    cloud.height, cloud.width = 1, len(values)
    cloud.point_step, cloud.row_step = 12, len(values)*12
    cloud.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                    for i, name in enumerate(('x','y','z'))]
    cloud.data = values.tobytes()
    position = 0.
    last = 0.
    def spin_until(predicate, seconds=15, publish=True):
        nonlocal position, last, clock_ns
        deadline = time.monotonic()+seconds
        while not predicate():
            assert process.poll() is None, (tmp_path/'node.log').read_text()
            assert time.monotonic()<deadline, states[-1:] or (tmp_path/'node.log').read_text()
            if publish and time.monotonic()-last >= .05:
                stamp = source.get_clock().now().to_msg()
                if clock_pub is not None:
                    clock_ns += 50_000_000
                    clock_pub.publish(Clock(clock=Time(sec=clock_ns//10**9, nanosec=clock_ns%10**9)))
                    measured = clock_ns-100_000_000
                    stamp = Time(sec=measured//10**9, nanosec=measured%10**9)
                odometry = Odometry()
                odometry.header.frame_id, odometry.child_frame_id = 'camera_init', 'body'
                odometry.header.stamp, cloud.header.stamp = stamp, stamp
                position = min(.9, position+.015)
                cloud.data = (values-np.array([position,0.,0.],dtype='<f4')).tobytes()
                odometry.pose.pose.position.x = position
                odometry.pose.pose.orientation.w = 1.
                odom_pub.publish(odometry)
                cloud_pub.publish(cloud)
                last = time.monotonic()
            rclpy.spin_once(source, timeout_sec=.005)
            rclpy.spin_once(observer, timeout_sec=.005)
    try:
        spin_until(lambda: states and states[-1]['valid'] and states[-1].get('keyframes',0)>=2 and poses)
        topics = dict(observer.get_publisher_names_and_types_by_node('lidar_atlas','/'))
        assert set(topics) <= {'/parameter_events','/rosout',
            '/localization/lidar_atlas/pose','/localization/lidar_atlas/status',
            '/localization/lidar_atlas/local_to_atlas'}
        spin_until(client.service_is_ready)
        saved = False
        for _ in range(8):
            future = client.call_async(Trigger.Request())
            spin_until(future.done)
            if future.result().success:
                saved = True
                break
            started = time.monotonic()
            spin_until(lambda: time.monotonic()-started>.1)
        assert saved
        assert (tmp_path/'atlas/manifest.json').is_file()
        spin_until(lambda: states[-1]['latched'], seconds=5, publish=False)
        drained = time.monotonic()
        spin_until(lambda: time.monotonic()-drained>.2, publish=False)
        count = len(poses)
        resumed = time.monotonic()
        spin_until(lambda: time.monotonic()-resumed>1.)
        assert len(poses)==count and states[-1]['latched'] and not states[-1]['valid']
        future = client.call_async(Trigger.Request())
        spin_until(future.done)
        assert not future.result().success
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        process.wait(timeout=12)
        log.close()
        source.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
    assert process.returncode==0, (tmp_path/'node.log').read_text()
