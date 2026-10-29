# Copyright 2026 EasyFly Project
#
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

"""Actual mapper: original scan-time TF, native ray semantics and invalidation."""
import math
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
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


class Mapper:
    def __init__(self, simulated_time=False, dynamic=False, rolling=False):
        os.environ["ROS_DOMAIN_ID"] = "198"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()
        self.node = rclpy.create_node("voxel_source_time_fixture")
        self.node.set_parameters([Parameter("use_sim_time", value=simulated_time)])
        self.clock = self.node.create_publisher(Clock, "/clock", 10)
        self.log = tempfile.TemporaryFile(mode="w+")
        executable = (
            Path(get_package_prefix("easyfly_trajectory_optimizer"))
            / "lib/easyfly_trajectory_optimizer/gcopter_voxel_map"
        )
        self.origin = [0., 0., 1.3]
        self.process = subprocess.Popen(
            [str(executable), "--ros-args", "-p", "use_sim_time:=" + str(simulated_time).lower(),
             "-p", "dynamic_clearing:=" + str(dynamic).lower(),
             "-p", "rolling_xy:=" + str(rolling).lower()],
            stdout=self.log,
            stderr=subprocess.STDOUT,
        )
        self.rays = self.node.create_publisher(PointCloud2, "/livox/lidar/rays", 2)
        self.tf = TransformBroadcaster(self.node)
        self.maps = []
        self.sub = self.node.create_subscription(
            VoxelGrid,
            "/planning/voxel_grid",
            self.maps.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.spin(0.8)

    def spin(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            assert self.process.poll() is None

    def wait(self, predicate, seconds=2.0):
        until = time.monotonic() + seconds
        while not predicate() and time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            assert self.process.poll() is None
        assert predicate()

    def transform(self, stamp):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id, tf.header.stamp = "odom", "livox_frame", stamp
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = self.origin
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)

    def scan(self, kind=0, stamp=None):
        p = PointCloud2()
        p.header.frame_id = "livox_frame"
        p.header.stamp = stamp or self.node.get_clock().now().to_msg()
        p.height = p.width = 1
        p.point_step = p.row_step = 24
        p.is_dense = True
        p.fields = [
            PointField(name=name, offset=offset, datatype=kind, count=1)
            for name, offset, kind in [
                ("x", 0, 7),
                ("y", 4, 7),
                ("z", 8, 7),
                ("return_type", 12, 2),
                ("range_min", 16, 7),
                ("range_max", 20, 7),
            ]
        ]
        p.data = struct.pack("<fffB3xff", 5.0 if kind == 0 else 1.0, 0.0, 0.0, kind, 0.2, 5.0)
        self.rays.publish(p)
        return p

    def observed(self, kind=0):
        stamp = self.node.get_clock().now().to_msg()
        self.transform(stamp)
        p = self.scan(kind, stamp)
        self.wait(lambda: any(m.valid and m.header.stamp == p.header.stamp for m in self.maps))
        return self.maps[-1]

    @staticmethod
    def value(grid, p):
        c = [
            math.floor((p[i] - v) / grid.resolution)
            for i, v in enumerate([grid.origin.x, grid.origin.y, grid.origin.z])
        ]
        return grid.data[c[0] + grid.size_x * (c[1] + grid.size_y * c[2])]

    def close(self, signum=signal.SIGINT):
        self.process.send_signal(signum)
        try:
            assert self.process.wait(timeout=5) == 0
        finally:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait(timeout=3)
            self.log.seek(0)
            print(self.log.read(), flush=True)
            self.log.close()
            self.node.destroy_node()
            rclpy.shutdown()


@pytest.fixture
def h():
    value = Mapper()
    try:
        yield value
    finally:
        value.close()


def test_miss_clears_only_observed_ray_and_hit_is_inflated(h):
    free = h.observed()
    assert h.value(free, [2.1, 0.1, 1.3]) == 0
    assert h.value(free, [5.0, 0.1, 1.3]) == 0
    assert h.value(free, [0.1, 2.1, 1.3]) == -1
    hit = h.observed(1)
    assert h.value(hit, [1.1, 0.1, 1.3]) == 100
    assert h.value(hit, [1.1, 0.1, 1.9]) == 100
    assert h.value(h.observed(), [1.1, 0.1, 1.3]) == 100


def test_future_tf_cannot_substitute_scan_time_then_exact_tf_unblocks(h):
    source = h.node.get_clock().now()
    h.transform((source + rclpy.duration.Duration(seconds=0.2)).to_msg())
    p = h.scan(stamp=source.to_msg())
    h.spin(0.2)
    assert not any(m.valid for m in h.maps)
    h.transform(source.to_msg())
    h.wait(lambda: any(m.valid and m.header.stamp == p.header.stamp for m in h.maps))


def test_stopped_input_publishes_zero_stamp_unknown_map(h):
    good = h.observed()
    h.wait(lambda: h.maps and not h.maps[-1].valid, 2.0)
    invalid = h.maps[-1]
    assert invalid.header.stamp.sec == invalid.header.stamp.nanosec == 0
    assert (
        invalid.size_x == good.size_x
        and invalid.size_y == good.size_y
        and invalid.size_z == good.size_z
    )
    assert len(invalid.data) == len(good.data) and set(invalid.data) == {-1}


def test_invalid_return_never_creates_free_space(h):
    stamp = h.node.get_clock().now().to_msg()
    h.transform(stamp)
    h.scan(2, stamp)
    h.spin(0.2)
    assert not any(m.valid for m in h.maps)


def test_clock_rollback_discards_map_and_advances_generation():
    mapper = Mapper(simulated_time=True)
    try:
        mapper.wait(lambda: mapper.clock.get_subscription_count() >= 2)
        clock = Clock()
        clock.clock.sec = 10
        mapper.clock.publish(clock)
        mapper.spin(0.1)
        before = mapper.observed(1)
        assert mapper.value(before, [1.1, 0.1, 1.3]) == 100
        clock.clock.sec = 5
        mapper.clock.publish(clock)
        mapper.wait(lambda: mapper.maps[-1].generation > before.generation)
        reset = mapper.maps[-1]
        assert reset.session_id == before.session_id and not reset.valid
        assert reset.header.stamp.sec == reset.header.stamp.nanosec == 0
        assert set(reset.data) == {-1}
        after = mapper.observed(0)
        assert after.generation == before.generation + 1
        assert mapper.value(after, [1.1, 0.1, 1.3]) == 0
    finally:
        mapper.close()


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_shutdown_during_native_mapper_callbacks(signum):
    # Each run must exit normally; this is a fixed stress set for the observed
    # context-invalid graph-query race, not a retry-until-success fixture.
    for _ in range(4):
        mapper = Mapper()
        try:
            mapper.observed()
            for _ in range(4):
                source = mapper.node.get_clock().now().to_msg()
                mapper.transform(source)
                mapper.scan(stamp=source)
        finally:
            mapper.close(signum)


def test_removed_hit_requires_three_new_source_observations():
    mapper = Mapper(dynamic=True)
    try:
        mapper.observed(1)
        first = mapper.observed(0)
        assert mapper.value(first, [1.1, .1, 1.3]) == 100
        for _ in range(5):
            mapper.scan(0, first.header.stamp)
        mapper.spin(.15)
        assert mapper.maps[-1].revision == first.revision
        assert mapper.value(mapper.observed(0), [1.1, .1, 1.3]) == 100
        assert mapper.value(mapper.observed(0), [1.1, .1, 1.3]) == 0
        assert mapper.value(mapper.maps[-1], [1.1, 2.1, 1.3]) == -1
    finally:
        mapper.close()


def test_rolling_map_preserves_world_hit_and_marks_new_side_unknown():
    mapper = Mapper(dynamic=True, rolling=True)
    try:
        original = mapper.observed(1)
        for step in range(1, 10):
            mapper.origin[0] = step*.5
            current = mapper.observed(0)
        assert current.origin.x > original.origin.x
        assert current.session_id == original.session_id
        assert current.generation == original.generation
        assert current.revision > original.revision
        assert mapper.value(current, [1.1, .1, 1.3]) == 100
        assert mapper.value(current, [9.1, 2.1, 1.3]) == -1
        # A pose jump cannot shift stored world evidence into a new coordinate epoch.
        mapper.origin[0] = 40.
        stamp = mapper.node.get_clock().now().to_msg()
        mapper.transform(stamp)
        mapper.scan(0, stamp)
        mapper.wait(lambda: not mapper.maps[-1].valid)
        mapper.origin[0] = 4.5
        stamp = mapper.node.get_clock().now().to_msg()
        mapper.transform(stamp)
        mapper.scan(0, stamp)
        mapper.spin(.15)
        assert not mapper.maps[-1].valid
    finally:
        mapper.close()
