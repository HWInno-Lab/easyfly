# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bounded actual semantic node shutdown with valid cloud, TF, clock and service traffic."""

import os
import importlib.util
from pathlib import Path
import signal
import subprocess
import sys
import time
from unittest.mock import Mock

from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from easyfly_msgs.msg import ObstacleInfoArray, UAVState
from easyfly_msgs.srv import GetObstacles
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray


@pytest.mark.parametrize('mismatch', ['executable', 'start_ticks'])
def test_capture_refuses_changed_identity(monkeypatch, mismatch):
    """Exercise the routing guard without sending any real shutdown signal."""
    source = Path(__file__).parents[1] / 'scripts/semantic_shutdown_capture.py'
    spec = importlib.util.spec_from_file_location('semantic_capture_guard', source)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    executable = Path(sys.executable).resolve(strict=True)
    owned = helper.identity(os.getpid(), executable)
    assert owned is not None
    if mismatch == 'start_ticks':
        owned = (owned[0], str(int(owned[1]) + 1))
    else:
        executable = source.resolve(strict=True)
    sender = Mock()
    monkeypatch.setattr(helper.signal, 'pidfd_send_signal', sender)
    with pytest.raises(RuntimeError, match='refuse signaling'):
        helper.signal_owned(owned, executable, signal.SIGINT)
    sender.assert_not_called()


@pytest.mark.parametrize('clock_mode', ['system', 'running', 'frozen'])
@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('capture', [False, True])
def test_active_mapping_shutdown(tmp_path, clock_mode, stop_signal, capture):
    """Require actual ready maps before signaling; forced cleanup is never a pass."""
    run_active_mapping_shutdown(tmp_path, clock_mode, stop_signal, capture)


def run_active_mapping_shutdown(tmp_path, clock_mode, stop_signal, capture, workload='steady'):
    """Exercise bounded legitimate traffic, optionally with moving obstacles and readers."""
    os.environ['ROS_DOMAIN_ID'] = '186'
    rclpy.init()
    peer = rclpy.create_node('semantic_shutdown_peer')
    clouds = peer.create_publisher(PointCloud2, '/pointcloud_filtered', 10)
    states = peer.create_publisher(UAVState, '/uav_state', 10)
    clock_pub = peer.create_publisher(Clock, '/clock', 10)
    broadcaster = TransformBroadcaster(peer)
    maps = []
    peer.create_subscription(
        ObstacleInfoArray, '/obstacles', maps.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    outputs = {'grid': 0, 'nonground': 0, 'markers': 0}

    def observed(name, message):
        if (name == 'grid' and message.header.stamp.sec > 0) or \
                (name == 'nonground' and message.width > 0) or name == 'markers':
            outputs[name] += 1

    for kind, topic, name, qos in [
            (OccupancyGrid, '/occupancy_grid', 'grid',
             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            (PointCloud2, '/pointcloud_nonground', 'nonground', 10),
            (MarkerArray, '/obstacle_markers', 'markers', 10)]:
        peer.create_subscription(kind, topic, lambda m, n=name: observed(n, m), qos)
    client = peer.create_client(GetObstacles, '/get_obstacles')
    # Valid XYZI only: no malformed layouts, NaNs, resource overflow or crashes.
    extent = 20 if workload == 'steady' else 40
    ground = [(x * .1, y * .1, 0., 1.)
              for x in range(-extent, extent + 1) for y in range(-extent, extent + 1)]
    obstacle = [(2.5 + x * .1, y * .1, .5 + z * .1, 1.)
                for x in range(5) for y in range(5) for z in range(5)]
    fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(['x', 'y', 'z', 'intensity'])]
    tick = [0]
    frozen = [False]
    moving = [False]
    clock_offset = [0]

    def publish():
        if not frozen[0]:
            tick[0] += 1
        stamp = (peer.get_clock().now().to_msg() if clock_mode == 'system' else
                 Time(sec=10 + clock_offset[0] + tick[0] // 20,
                      nanosec=(tick[0] % 20) * 50000000))
        if clock_mode != 'system':
            clock_pub.publish(Clock(clock=stamp))
        header = Header(frame_id='odom', stamp=stamp)
        transform = TransformStamped(header=header, child_frame_id='base_link')
        transform.transform.translation.z = 1.0
        transform.transform.rotation.w = 1.0
        ground_basis = TransformStamped(header=Header(frame_id='map', stamp=stamp), child_frame_id='odom')
        ground_basis.transform.rotation.w = 1.0
        broadcaster.sendTransform([ground_basis, transform])
        states.publish(UAVState(header=header, connected=True, pose_valid=True,
                                armed=False, position_z=1.0))
        if not frozen[0]:
            shift = .04 * (tick[0] - moving[0]) if moving[0] else 0.
            points = ground + [(x + shift, y, z, intensity)
                               for x, y, z, intensity in obstacle]
            clouds.publish(create_cloud(header, fields, points))

    timer = peer.create_timer(.05, publish)
    executable = Path(get_package_prefix('easyfly_semantic_map')) / (
        'lib/easyfly_semantic_map/semantic_map_node')
    process = None
    forced = False
    readers = []

    def until(predicate, seconds):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            assert process.poll() is None, 'node exited before the planned shutdown signal'
            rclpy.spin_once(peer, timeout_sec=.01)
        assert predicate(), 'ready mapping/service evidence deadline expired'

    try:
        with (tmp_path / 'semantic.log').open('w') as output:
            command = [str(executable), '--ros-args', '-p',
                       'use_sim_time:=' + ('false' if clock_mode == 'system' else 'true'),
                       '-p', 'ground_model_frame:=map']
            if capture:
                command = [sys.executable, str(Path(__file__).parents[1] /
                           'scripts/semantic_shutdown_capture.py'), *command]
            process = subprocess.Popen(
                command,
                stdout=output, stderr=subprocess.STDOUT)
            until(lambda: len([m for m in maps if m.obstacles and m.header.stamp.sec > 0]) >= 3
                  and all(v >= 3 for v in outputs.values()) and client.service_is_ready(), 12.)
            reply = client.call_async(GetObstacles.Request(range_limit=5.0))
            until(reply.done, 3.)
            assert reply.result().obstacles.obstacles, 'TF query returned no ready obstacles'
            if workload != 'steady':
                moving[0] = tick[0]
                before = len(maps)
                began = time.monotonic()
                next_reader = began
                jumped = False
                while time.monotonic() - began < 4.:
                    assert process.poll() is None, 'node exited during legitimate active traffic'
                    rclpy.spin_once(peer, timeout_sec=.01)
                    now = time.monotonic()
                    if now >= next_reader:
                        for reader in readers:
                            peer.destroy_subscription(reader)
                        readers.clear()
                        readers.append(peer.create_subscription(
                            ObstacleInfoArray, '/obstacles', lambda m: None,
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
                        next_reader = now + .25
                    if workload == 'clock-reset' and not jumped and now - began >= 2.:
                        clock_offset[0] = -2
                        jumped = True
                assert len(maps) - before >= 20, 'insufficient processed workload evidence'
                assert any(o.is_dynamic for m in maps[before:] for o in m.obstacles), \
                    'moving obstacle never acquired real dynamic evidence'
                if workload == 'clock-reset':
                    after_jump = len(maps)
                    until(lambda: len([m for m in maps[after_jump:] if m.obstacles]) >= 3, 3.)
                print(f'workload={workload}: ready_outputs={len(maps) - before} '
                      f'clock_jump={jumped} points_per_frame={len(ground) + len(obstacle)}',
                      flush=True)
            if clock_mode == 'frozen':
                frozen[0] = True
            pending = client.call_async(GetObstacles.Request(range_limit=5.0))
            start = time.monotonic()
            process.send_signal(stop_signal)
            deadline = start + 4.0
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(peer, timeout_sec=.01)
            assert process.poll() == 0, f'nonclean semantic shutdown: {process.poll()}'
            print(f'{clock_mode}/{stop_signal.name}/capture={capture}: ready_maps={len(maps)} '
                  f'exit_seconds={time.monotonic() - start:.3f} '
                  f'outputs={outputs} pending_query_done={pending.done()}', flush=True)
        if capture:
            log = (tmp_path / 'semantic.log').read_text()
            assert 'semantic capture forwarded:' in log
            assert 'exited normally' in log
            assert 'received signal SIGSEGV' not in log and 'forced cleanup' not in log
    finally:
        if process is not None and process.poll() is None:
            forced = True
            process.kill()
            process.wait(timeout=2.)
        peer.destroy_timer(timer)
        peer.destroy_node()
        rclpy.shutdown()
    assert not forced, 'forced cleanup is not acceptance'
