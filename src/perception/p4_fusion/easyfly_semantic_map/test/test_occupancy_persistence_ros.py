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

#
#

"""Actual-node occupancy miss count and invalidation reset in both footprint modes."""
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import UAVState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize("mode", ["cluster_boxes", "observed_cells"])
def test_occupancy_hold_release_and_invalid_reset(tmp_path, mode):
    rclpy.init()
    peer = rclpy.create_node("alg73_occupancy_persistence")
    clouds = peer.create_publisher(PointCloud2, "/pointcloud_filtered", 10)
    states = peer.create_publisher(UAVState, "/uav_state", 10)
    broadcaster = TransformBroadcaster(peer)
    frames, invalid, phases = [], [], {}
    state = dict(obstacle=True, valid=True, phase="initial")

    def stamp_key(stamp):
        return stamp.sec * 1000000000 + stamp.nanosec

    def receive(grid):
        if stamp_key(grid.header.stamp) == 0:
            invalid.append(grid)
        else:
            frames.append((phases.get(stamp_key(grid.header.stamp)), grid))

    peer.create_subscription(
        OccupancyGrid,
        "/occupancy_grid",
        receive,
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(("x", "y", "z", "intensity"))
    ]

    def publish():
        header = Header(frame_id="odom", stamp=peer.get_clock().now().to_msg())
        phases[stamp_key(header.stamp)] = state["phase"]
        vehicle = TransformStamped(header=header, child_frame_id="base_link")
        vehicle.transform.translation.z = 1.0
        vehicle.transform.rotation.w = 1.0
        broadcaster.sendTransform(vehicle)
        states.publish(
            UAVState(
                header=header,
                connected=True,
                pose_valid=state["valid"],
                armed=False,
                position_z=1.0,
            )
        )
        points = [(x * 0.1, y * 0.1, 0.0, 1.0) for x in range(-20, 21) for y in range(-20, 21)]
        if state["obstacle"]:
            points += [
                (2.5 + 0.1 * x, 0.1 * y, 0.5 + 0.1 * z, 2.0)
                for x in range(5)
                for y in range(5)
                for z in range(5)
            ]
        clouds.publish(create_cloud(header, fields, points))

    timer = peer.create_timer(0.1, publish)
    executable = (
        Path(get_package_prefix("easyfly_semantic_map"))
        / "lib/easyfly_semantic_map/semantic_map_node"
    )
    process = None
    log_path = tmp_path / "occupancy.log"

    def spin():
        assert process.poll() is None, log_path.read_text()
        rclpy.spin_once(peer, timeout_sec=0.01)

    def until(predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, log_path.read_text()
            spin()

    def selected(phase):
        return [grid for p, grid in frames if p == phase]

    def cell(grid):
        col = int((2.7 - grid.info.origin.position.x) / grid.info.resolution)
        row = int((0.2 - grid.info.origin.position.y) / grid.info.resolution)
        return grid.data[row * grid.info.width + col]

    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(
                [
                    str(executable),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=false",
                    "-p",
                    "grid_footprint_mode:=" + mode,
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            until(lambda: len(selected("initial")) >= 4)
            assert all(cell(g) == 100 for g in selected("initial"))
            state.update(obstacle=False, phase="empty")
            until(lambda: len(selected("empty")) >= 22)
            empty = selected("empty")
            assert [cell(g) for g in empty[:20]] == [100] * 20
            assert all(cell(g) == 0 for g in empty[20:])
            state.update(obstacle=True, phase="repopulate")
            until(lambda: len(selected("repopulate")) >= 3)
            assert all(cell(g) == 100 for g in selected("repopulate"))
            before = len(invalid)
            state.update(valid=False, phase="invalid")
            until(lambda: len(invalid) > before)
            assert all(v == -1 for v in invalid[-1].data)
            state.update(obstacle=False, valid=True, phase="recovery")
            until(lambda: len(selected("recovery")) >= 4)
            assert all(cell(g) == 0 for g in selected("recovery"))
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=4.0) == 0, log_path.read_text()
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=2.0)
        peer.destroy_timer(timer)
        peer.destroy_node()
        rclpy.shutdown()
