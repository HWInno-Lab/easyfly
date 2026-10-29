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

"""Inspect local history and replay selected sensor types in an isolated network."""

import argparse
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import time

from .storage import inspect_session


def bag_inventory(directory, record):
    """Require contained SQLite files and a sensor-only, finite rosbag timeline."""
    import rosbag2_py
    import yaml
    root = Path(directory)
    metadata = root / 'bag/metadata.yaml'
    if 'bag/metadata.yaml' not in record['files']:
        raise ValueError('this recording has no raw rosbag2 data')
    info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'sqlite3':
        raise ValueError('only the recorded SQLite3 backend is supported')
    paths = info['relative_file_paths']
    if not paths or len(paths) > 16:
        raise ValueError('invalid bag file inventory')
    for name in paths:
        if (Path(name).name != name or not name.endswith('.db3') or
                'bag/' + name not in record['files']):
            raise ValueError('bag metadata escapes the verified payload')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    allowed = {'sensor_msgs/msg/Image': '/history/image',
               'nav_msgs/msg/Odometry': '/history/odom'}
    topics = reader.get_all_topics_and_types()
    if (not topics or len(topics) > 2 or len({x.type for x in topics}) != len(topics) or
            any(x.type not in allowed or x.serialization_format != 'cdr' for x in topics)):
        raise ValueError('only unique Image and Odometry sources may be replayed')
    first = int(info['starting_time']['nanoseconds_since_epoch'])
    duration = int(info['duration']['nanoseconds'])
    if first <= 0 or duration < 0 or duration > 24 * 3600 * 10**9:
        raise ValueError('invalid replay timeline')
    return reader, {x.name: (x.type, allowed[x.type]) for x in topics}, first, first + duration


def isolated_command(executable, directory, rate=1., auto_play=False):
    """Build a new network/IPC/PID namespace; never fall back to the live graph."""
    root = Path(directory).resolve(strict=True)
    record = inspect_session(root)
    bag_inventory(root, record)
    if not math.isfinite(rate) or not .1 <= rate <= 4.:
        raise ValueError('playback rate must be between 0.1 and 4')
    bwrap = shutil.which('bwrap')
    if not bwrap:
        raise RuntimeError('bubblewrap is required for isolated replay')
    parent_net = os.stat('/proc/self/ns/net').st_ino
    command = [bwrap, '--die-with-parent', '--new-session', '--unshare-net',
               '--unshare-ipc', '--unshare-pid', '--ro-bind', '/', '/',
               '--proc', '/proc', '--dev', '/dev', '--tmpfs', '/dev/shm',
               '--tmpfs', '/tmp', '--dir', '/tmp/history-home',
               '--setenv', 'HOME', '/tmp/history-home',
               '--setenv', 'ROS_HOME', '/tmp/history-home',
               '--setenv', 'ROS_LOG_DIR', '/tmp/history-home/log',
               '--setenv', 'ROS_DOMAIN_ID', '219', '--setenv', 'ROS_LOCALHOST_ONLY', '1',
               '--setenv', 'PYTHONNOUSERSITE', '1']
    for key in ['FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'CYCLONEDDS_URI', 'RMW_IMPLEMENTATION', 'ROS_DISCOVERY_SERVER']:
        command.extend(['--unsetenv', key])
    command.extend([str(executable), '--inside-replay', str(root),
                    '--parent-net', str(parent_net), '--rate', str(rate)])
    if auto_play:
        command.append('--auto-play')
    return command


def replay_inside(directory, parent_net, rate, auto_play):
    """Publish actual CDR in the sandbox with pause/resume/seek via bounded stdin."""
    if not parent_net or os.stat('/proc/self/ns/net').st_ino == parent_net:
        raise RuntimeError('refusing replay outside an isolated network namespace')
    import rclpy
    from builtin_interfaces.msg import Time
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from rosidl_runtime_py.utilities import get_message
    record = inspect_session(directory)
    reader, topics, first, last = bag_inventory(directory, record)
    rclpy.init()
    node = rclpy.create_node('isolated_history_player')
    publishers = {name: (get_message(typ), node.create_publisher(get_message(typ), dest, 10))
                  for name, (typ, dest) in topics.items()}
    clock = node.create_publisher(Clock, '/clock', 10)
    paused = not auto_play
    source = first
    anchor_source = first
    anchor_wall = time.monotonic() + .5
    pending = None
    published = 0
    reports = 0.
    stopped = False
    buffer = bytearray()
    control_open = True

    def stop(signum, frame):
        nonlocal stopped
        del signum, frame
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    os.set_blocking(sys.stdin.fileno(), False)
    try:
        while not stopped and rclpy.ok():
            if control_open and select.select([sys.stdin], [], [], 0)[0]:
                chunk = os.read(sys.stdin.fileno(), 2048)
                if not chunk:
                    control_open = False
                    stopped = not auto_play
                    continue
                buffer.extend(chunk)
                if len(buffer) > 4096:
                    raise ValueError('replay command exceeds input limit')
                while b'\n' in buffer:
                    line, _, rest = buffer.partition(b'\n')
                    buffer[:] = rest
                    command = json.loads(line)
                    action = command.get('action')
                    if action == 'pause':
                        paused = True
                    elif action == 'resume':
                        paused = False
                        anchor_source, anchor_wall = source, time.monotonic()
                    elif action == 'seek':
                        target = command.get('source_ns')
                        if not isinstance(target, int) or not first <= target <= last:
                            raise ValueError('seek outside recorded source timeline')
                        reader.seek(target)
                        source = anchor_source = target
                        anchor_wall = time.monotonic()
                        pending = None
                    elif action == 'stop':
                        stopped = True
                    else:
                        raise ValueError('unknown replay command')
            if not paused:
                if pending is None and reader.has_next():
                    pending = reader.read_next()
                if pending is not None:
                    name, data, timestamp = pending
                    deadline = anchor_wall + (timestamp - anchor_source) / (rate * 10**9)
                    if time.monotonic() >= deadline:
                        source = timestamp
                        clock.publish(Clock(clock=Time(
                            sec=timestamp // 10**9, nanosec=timestamp % 10**9)))
                        typ, publisher = publishers[name]
                        publisher.publish(deserialize_message(data, typ))
                        published += 1
                        pending = None
                elif not reader.has_next():
                    paused = True
                    if auto_play:
                        stopped = True
            rclpy.spin_once(node, timeout_sec=.005)
            if time.monotonic() - reports >= .1 or stopped:
                print(json.dumps({'phase': 'finished' if stopped else
                                  ('paused' if paused else 'playing'),
                                  'source_ns': source, 'first_ns': first, 'last_ns': last,
                                  'published': published, 'rate': rate,
                                  'isolated_network': True,
                                  'net_namespace': os.stat('/proc/self/ns/net').st_ino,
                                  'topics': [v[1] for v in topics.values()]}), flush=True)
                reports = time.monotonic()
        # Let the middleware finish the last delivery before orderly teardown.
        deadline = time.monotonic() + .2
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    """Open the history window or a validated isolated replay process."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inspect', type=Path)
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--inside-replay', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--parent-net', type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--auto-play', action='store_true')
    args = parser.parse_args()
    if args.inspect:
        print(json.dumps(inspect_session(args.inspect), indent=2))
    elif args.inside_replay:
        replay_inside(args.inside_replay, args.parent_net, args.rate, args.auto_play)
    elif args.replay:
        command = isolated_command(Path(sys.argv[0]).resolve(), args.replay,
                                   args.rate, args.auto_play)
        raise SystemExit(subprocess.call(command))
    else:
        from .history_window import run
        run()
