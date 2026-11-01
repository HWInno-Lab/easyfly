# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0.
"""A real idle FUEL server remains observable without sending any task or flight goal."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from easyfly_msgs.msg import VoxelGrid
import rclpy
from rclpy.action import get_action_names_and_types
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger


def test_idle_service_is_available_before_policy_map_and_no_automatic_execution():
    os.environ["ROS_DOMAIN_ID"] = "217"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    node = rclpy.create_node("fuel_idle_fixture")
    process = None
    try:
        with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(mode="w+") as log:
            environment = dict(os.environ)
            source = str(Path(__file__).resolve().parents[1])
            environment["PYTHONPATH"] = source + os.pathsep + environment.get("PYTHONPATH", "")
            process = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    "from easyfly_fuel_exploration.node import main; main()",
                    "--ros-args",
                    "-p",
                    "use_sim_time:=false",
                    "-p",
                    "extended_profile:=false",
                    "-p",
                    "evidence_directory:=" + directory,
                ],
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            client = node.create_client(Trigger, "/exploration/management_idle")
            maps = []
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            subscription = node.create_subscription(
                VoxelGrid, "/planning/voxel_grid", maps.append, qos
            )
            raw = node.create_publisher(VoxelGrid, "/planning/fuel_raw_grid", qos)
            observed = node.create_publisher(VoxelGrid, "/planning/fuel_observed_grid", qos)

            def spin(seconds):
                until = time.monotonic() + seconds
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if process.poll() is not None:
                        log.seek(0)
                        raise AssertionError(log.read())

            spin(1.0)
            assert client.service_is_ready()
            assert (
                "/explore",
                ["easyfly_msgs/action/Explore"],
            ) in get_action_names_and_types(node)
            future = client.call_async(Trigger.Request())
            spin(0.2)
            assert future.done() and future.result().success
            assert future.result().message == "idle" and maps == []
            grid = VoxelGrid()
            grid.header.frame_id = "odom"
            grid.header.stamp = node.get_clock().now().to_msg()
            grid.session_id, grid.generation, grid.revision = "sensor_epoch", 0, 1
            grid.valid, grid.resolution = True, 0.2
            grid.size_x = grid.size_y = grid.size_z = 2
            grid.data = [0] * 8
            raw.publish(grid)
            grid.inflation_radius = 0.65
            observed.publish(grid)
            spin(0.3)
            future = client.call_async(Trigger.Request())
            spin(0.2)
            assert future.done() and future.result().success
            assert maps == [], "input grids must not start a policy epoch or submit a task"
            assert list(Path(directory).iterdir()) == [], "idle server must not launch native work"
            assert subscription
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=5) == 0
    finally:
        if process and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
