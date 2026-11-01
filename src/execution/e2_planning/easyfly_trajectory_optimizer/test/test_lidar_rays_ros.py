# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0.
"""Real adapter+mapper in an isolated ROS domain; no hardware or flight clients."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import VoxelGrid
from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster


def test_actual_livox_packet_preserves_per_point_origin_and_unknown():
    os.environ["ROS_DOMAIN_ID"] = "216"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    node = rclpy.create_node("physical_lidar_fixture")
    prefix = (
        Path(get_package_prefix("easyfly_trajectory_optimizer"))
        / "lib/easyfly_trajectory_optimizer"
    )
    processes, logs = [], []
    try:
        for executable, parameters in [
            (
                "lidar_rays_node",
                [
                    "input_topic:=/physical_livox",
                    "source_frame:=real_sensor",
                    "range_min:=0.2",
                    "range_max:=10.0",
                ],
            ),
            (
                "gcopter_voxel_map",
                [
                    "source_frame:=real_sensor",
                    "physical_livox:=true",
                    "uninflated_topic:=/physical_raw",
                ],
            ),
        ]:
            log = tempfile.TemporaryFile(mode="w+")
            logs.append(log)
            args = [str(prefix / executable), "--ros-args"]
            for parameter in parameters:
                args.extend(["-p", parameter])
            processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))
        sensor = node.create_publisher(CustomMsg, "/physical_livox", 10)
        tf = TransformBroadcaster(node)
        maps, clouds = [], []
        subscriptions = [
            node.create_subscription(
                VoxelGrid,
                "/physical_raw",
                maps.append,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
            node.create_subscription(PointCloud2, "/livox/lidar/rays", clouds.append, 10),
        ]

        def spin(seconds):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.01)
                for process, log in zip(processes, logs):
                    if process.poll() is not None:
                        log.seek(0)
                        raise AssertionError(log.read())

        def transform(stamp, x):
            value = TransformStamped()
            value.header.frame_id, value.child_frame_id = "odom", "real_sensor"
            value.header.stamp = Time(nanoseconds=stamp).to_msg()
            value.transform.translation.x, value.transform.translation.z = x, 1.3
            value.transform.rotation.w = 1.0
            tf.sendTransform(value)

        spin(1.0)
        base = node.get_clock().now().nanoseconds - 250000000
        transform(base - 10000000, -0.1)
        transform(base + 110000000, 1.1)
        spin(0.1)
        packet = CustomMsg()
        packet.header.frame_id = "real_sensor"
        packet.header.stamp = Time(nanoseconds=base).to_msg()
        packet.timebase = base
        packet.points = [
            CustomPoint(x=2.0, offset_time=0),
            CustomPoint(x=2.0, offset_time=100000000),
            CustomPoint(x=0.0, offset_time=100000001),
            CustomPoint(x=float("nan"), offset_time=100000002),
        ]
        packet.point_num = len(packet.points)
        sensor.publish(packet)
        spin(0.4)
        assert len(clouds) == 1 and clouds[0].point_step == 28 and clouds[0].width == 2
        valid = [value for value in maps if value.valid]
        assert valid, "physical map never became valid"
        grid = valid[-1]
        assert grid.header.stamp == Time(nanoseconds=base + 100000000).to_msg()

        def cell(x, y, z):
            i = int((x - grid.origin.x) // grid.resolution)
            j = int((y - grid.origin.y) // grid.resolution)
            k = int((z - grid.origin.z) // grid.resolution)
            return grid.data[(k * grid.size_y + j) * grid.size_x + i]

        assert cell(2.05, 0.05, 1.35) == 100
        assert cell(3.05, 0.05, 1.35) == 100, "second return must use its acquisition-time origin"
        assert cell(1.0, 2.0, 1.35) == -1, "unobserved space cannot become free"
        revision = grid.revision
        # Zero returns, bad frame, repeated timebase and absent exact TF produce no new valid map.
        packet.header.frame_id = "wrong"
        sensor.publish(packet)
        packet.header.frame_id = "real_sensor"
        sensor.publish(packet)
        spin(0.15)
        assert all(value.revision == revision for value in maps if value.valid)
        spin(0.8)
        assert not maps[-1].valid and all(value == -1 for value in maps[-1].data)
        cloud_count = len(clouds)
        packet.timebase = node.get_clock().now().nanoseconds - 100000000
        packet.header.stamp = Time(nanoseconds=packet.timebase).to_msg()
        packet.points = [CustomPoint(x=0.0)]
        packet.point_num = 1
        sensor.publish(packet)
        spin(0.1)
        assert len(clouds) == cloud_count, "absent/zero return cannot become a miss ray"
        packet.points = [CustomPoint(x=2.0)]
        second_source = node.create_publisher(CustomMsg, "/physical_livox", 10)
        spin(0.2)
        sensor.publish(packet)
        spin(0.1)
        assert len(clouds) == cloud_count, "ambiguous real source must be rejected"
        node.destroy_publisher(second_source)
        spin(0.2)
        packet.timebase = node.get_clock().now().nanoseconds - 100000000
        packet.header.stamp = Time(nanoseconds=packet.timebase).to_msg()
        sensor.publish(packet)  # No TF brackets this new physical acquisition.
        spin(0.2)
        assert len(clouds) == cloud_count + 1
        assert not maps[-1].valid
        assert not any(value.valid and value.revision > revision for value in maps)
        spin(1.1)  # Expire the pending hit before supplying newer TF.
        # Physical mapper itself rejects synthetic misses, even if its adapter is replaced.
        adapter = processes.pop(0)
        adapter.send_signal(signal.SIGINT)
        adapter.wait(timeout=5)
        logs.pop(0).close()
        direct = node.create_publisher(PointCloud2, "/livox/lidar/rays", 2)
        spin(0.2)
        stamp = node.get_clock().now().nanoseconds - 100000000
        transform(stamp - 10000000, 0.0)
        transform(stamp + 10000000, 0.0)
        spin(0.1)
        forged = PointCloud2()
        forged.header.frame_id = "real_sensor"
        forged.header.stamp = Time(nanoseconds=stamp).to_msg()
        forged.fields = clouds[0].fields
        forged.height = forged.width = 1
        forged.point_step = forged.row_step = 28
        forged.data = struct.pack("<fffB3xffI", 10.0, 0.0, 0.0, 0, 0.2, 10.0, 0)
        direct.publish(forged)
        spin(0.2)
        assert not maps[-1].valid
        assert not any(value.valid and value.revision > revision for value in maps)
        assert subscriptions
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
