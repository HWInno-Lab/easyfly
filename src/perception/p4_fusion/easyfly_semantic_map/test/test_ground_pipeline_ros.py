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

"""Actual ground lifecycle, without simulator or flight actions."""
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
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize("publish_period", [0.1, 0.11])
def test_ground_init_armed_freeze_basis_jump_recovery_and_stale_tf(tmp_path, publish_period):
    rclpy.init()
    peer = rclpy.create_node("alg71_ground_pipeline")
    clouds = peer.create_publisher(PointCloud2, "/pointcloud_filtered", 10)
    states = peer.create_publisher(UAVState, "/uav_state", 10)
    broadcaster = TransformBroadcaster(peer)
    grids, nonground = [], []
    peer.create_subscription(
        OccupancyGrid,
        "/occupancy_grid",
        grids.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    peer.create_subscription(PointCloud2, "/pointcloud_nonground", nonground.append, 10)
    fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(("x", "y", "z", "intensity"))
    ]
    state = dict(armed=True, floor=0.0, basis=0.0, publish_basis=True)
    sent = [0]

    def publish():
        header = Header(frame_id="odom", stamp=peer.get_clock().now().to_msg())
        vehicle = TransformStamped(header=header, child_frame_id="base_link")
        vehicle.transform.translation.z = 1.0
        vehicle.transform.rotation.w = 1.0
        transforms = [vehicle]
        if state["publish_basis"]:
            basis = TransformStamped(
                header=Header(frame_id="map", stamp=header.stamp), child_frame_id="odom"
            )
            basis.transform.translation.z = state["basis"]
            basis.transform.rotation.w = 1.0
            transforms.append(basis)
        broadcaster.sendTransform(transforms)
        states.publish(
            UAVState(
                header=header,
                connected=True,
                pose_valid=True,
                armed=state["armed"],
                position_z=1.0,
            )
        )
        points = [
            (x * 0.1, y * 0.1, state["floor"], 1.0) for x in range(-20, 21) for y in range(-20, 21)
        ]
        points += [(x * 0.05 + 0.5, y * 0.05 + 0.5, 0.2, 2.0) for x in range(5) for y in range(5)]
        clouds.publish(create_cloud(header, fields, points))
        sent[0] += 1

    timer = peer.create_timer(publish_period, publish)
    executable = (
        Path(get_package_prefix("easyfly_semantic_map"))
        / "lib/easyfly_semantic_map/semantic_map_node"
    )
    process = None
    log_path = tmp_path / "ground.log"

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(peer, timeout_sec=0.01)

    def until(predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, log_path.read_text()
            spin(0.03)

    def ready():
        return len([g for g in grids if g.header.stamp.sec > 0]) >= 4

    def low_obstacle_present():
        observed = [cloud for cloud in nonground if cloud.width > 0]
        return bool(observed) and any(
            abs(float(p[0]) - 0.2) < 0.015
            for p in read_points(observed[-1], field_names=("z",), skip_nans=True)
        )

    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(
                [
                    str(executable),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=false",
                    "-p",
                    "ground_model_frame:=map",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            until(
                lambda: clouds.get_subscription_count() > 0 and states.get_subscription_count() > 0
            )
            spin(1.0)
            assert sent[0] >= 8 and grids and not any(g.header.stamp.sec > 0 for g in grids)
            state["armed"] = False
            grids.clear()
            until(ready)
            assert low_obstacle_present()
            # A fresh but false .3m candidate must not move the trusted plane while armed.
            state.update(armed=True, floor=0.3)
            grids.clear()
            nonground.clear()
            spin(1.2)
            assert ready() and low_obstacle_present(), log_path.read_text()
            # A >.25m basis jump resets the model. An armed vehicle cannot reinitialize it.
            state["basis"] = 0.3
            sent_before_jump = sent[0]
            grids.clear()
            until(lambda: any(g.header.stamp.sec == 0 for g in grids))
            jump_invalid = next(i for i, g in enumerate(grids) if g.header.stamp.sec == 0)
            spin(0.8)
            # Repeated invalid maps coalesce; fresh traffic must not produce a ready map.
            assert sent[0] >= sent_before_jump + 7
            assert all(g.header.stamp.sec == 0 for g in grids[jump_invalid:])
            state.update(armed=False, floor=0.0)
            grids.clear()
            nonground.clear()
            until(ready)
            assert low_obstacle_present()
            # Keep cloud/state/vehicle TF fresh while only the ground basis ages out.
            stale_log_start = log_path.stat().st_size
            state["publish_basis"] = False
            grids.clear()
            until(lambda: any(g.header.stamp.sec == 0 for g in grids), timeout=4.0)
            first_invalid = next(i for i, g in enumerate(grids) if g.header.stamp.sec == 0)
            sent_before_stale = sent[0]
            spin(0.4)
            assert sent[0] >= sent_before_stale + 3
            assert all(g.header.stamp.sec == 0 for g in grids[first_invalid:])
            # Exact-time TF admission holds clouds until a transform arrives.
            # The 1 s source-age and wait budgets can expire in either order;
            # a 0.11 s publisher also exercises expiry before the queue fills.
            # All paths must keep the map invalid despite ongoing fresh input.
            with log_path.open("rb") as observed:
                observed.seek(stale_log_start)
                stale_log = observed.read().decode()
            assert any(
                reason in stale_log
                for reason in (
                    "地面模型坐标变换过期",
                    "对应源时间的 TF 等待超时",
                    "点云 TF 待定队列已满",
                    "点云时间戳为零/过旧/未来或不可转换",
                )
            )
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=4.0) == 0, log_path.read_text()
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=2.0)
        peer.destroy_timer(timer)
        peer.destroy_node()
        rclpy.shutdown()
