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

"""Current moving geometry, sweeps and floor-only reset through the actual node."""
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import ObstacleInfoArray, UAVState
from geometry_msgs.msg import TransformStamped
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def test_current_motion_sweep_and_empty_frame(tmp_path):
    rclpy.init()
    peer = rclpy.create_node("alg72_dynamic_geometry")
    clouds = peer.create_publisher(PointCloud2, "/pointcloud_filtered", 10)
    states = peer.create_publisher(UAVState, "/uav_state", 10)
    broadcaster = TransformBroadcaster(peer)
    obstacles, nonground = [], []
    peer.create_subscription(ObstacleInfoArray, "/obstacles", obstacles.append, 10)
    peer.create_subscription(PointCloud2, "/pointcloud_nonground", nonground.append, 10)
    fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(("x", "y", "z", "intensity"))
    ]
    state = dict(moving=False, empty=False, x=2.5)

    def publish():
        header = Header(frame_id="odom", stamp=peer.get_clock().now().to_msg())
        vehicle = TransformStamped(header=header, child_frame_id="base_link")
        vehicle.transform.translation.z = 1.0
        vehicle.transform.rotation.w = 1.0
        broadcaster.sendTransform(vehicle)
        states.publish(
            UAVState(header=header, connected=True, pose_valid=True, armed=False, position_z=1.0)
        )
        points = [(x * 0.1, y * 0.1, 0.0, 1.0) for x in range(-20, 21) for y in range(-20, 21)]
        if state["moving"]:
            state["x"] += 0.04
        if not state["empty"]:
            points += [
                (state["x"] + 0.1 * x, 0.1 * y, 0.5 + 0.1 * z, 2.0)
                for x in range(5)
                for y in range(5)
                for z in range(5)
            ]
        clouds.publish(create_cloud(header, fields, points))

    timer = peer.create_timer(0.05, publish)
    executable = (
        Path(get_package_prefix("easyfly_semantic_map"))
        / "lib/easyfly_semantic_map/semantic_map_node"
    )
    process = None
    log_path = tmp_path / "dynamic.log"

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

    def dynamic_messages():
        return [o for a in obstacles for o in a.obstacles if o.is_dynamic]

    def predicted_points():
        if not nonground:
            return []
        return [
            p
            for p in read_points(
                nonground[-1], field_names=("x", "y", "z", "intensity"), skip_nans=True
            )
            if float(p[3]) == 0.0
        ]

    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(
                [str(executable), "--ros-args", "-p", "use_sim_time:=false"],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            until(
                lambda: clouds.get_subscription_count() > 0 and states.get_subscription_count() > 0
            )
            until(lambda: len([a for a in obstacles if a.obstacles]) >= 5)
            assert not dynamic_messages()
            obstacles.clear()
            state["moving"] = True
            until(lambda: len(dynamic_messages()) >= 3 and bool(predicted_points()))
            moving = dynamic_messages()
            assert len({o.obstacle_id for o in moving}) == 1
            assert all(0.2 < o.velocity_x < 2.0 and abs(o.velocity_y) < 0.1 for o in moving)
            prediction = predicted_points()
            assert len(prediction) % 9 == 0 and len(prediction) >= 9
            assert max(float(p[0]) for p in prediction) > moving[-1].center_x + 0.2
            # Floor-only frames contain no historical obstacle or swept occupancy.
            state.update(empty=True, moving=False)
            obstacles.clear()
            nonground.clear()
            until(lambda: len(obstacles) >= 4 and len(nonground) >= 4)
            assert all(not a.obstacles for a in obstacles[-3:])
            assert all(c.width == 0 for c in nonground[-3:])
            # Reappearance at rest must establish fresh motion evidence.
            state["empty"] = False
            obstacles.clear()
            until(lambda: len([a for a in obstacles if a.obstacles]) >= 5)
            assert not dynamic_messages()
            assert not predicted_points()
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=4.0) == 0, log_path.read_text()
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=2.0)
        peer.destroy_timer(timer)
        peer.destroy_node()
        rclpy.shutdown()
