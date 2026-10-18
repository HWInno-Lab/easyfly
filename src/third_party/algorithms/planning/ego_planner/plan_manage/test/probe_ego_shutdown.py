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

"""Bounded real EGO cloud/odom shutdown probe, without navigation goals."""

import ast
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
import yaml


def main():
    """Capture native exit and debugger status without silently forced success."""
    root = next(p for p in Path(__file__).resolve().parents if (p / "src/algorithms").is_dir())
    output = Path(tempfile.mkdtemp(
        prefix='ego-cloud-shutdown-', dir=root / 'artifacts/shutdown-20261007'))
    launch = root / ('src/execution/e2_planning/easyfly_local_planner/'
                     'launch/local_planner.launch.py')
    tree = ast.parse(launch.read_text())
    assignment = next(item for item in ast.walk(tree) if isinstance(item, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == 'ego_params'
                              for t in item.targets))
    context = {'use_sim_time': False, 'flight_type': 1, 'managed': True,
               'virtual_ceil_height': 3.2,
               'ParameterValue': lambda value, value_type: value}
    parameters = eval(compile(ast.Expression(assignment.value), str(launch), 'eval'),
                      {'__builtins__': {}}, context)
    params = output / 'parameters.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    os.environ['ROS_DOMAIN_ID'] = '209'
    rclpy.init()
    node = rclpy.create_node('ego_shutdown_probe')
    odom = node.create_publisher(Odometry, '/odom', 10)
    cloud = node.create_publisher(PointCloud2, '/pointcloud_nonground', 10)
    executable = Path(get_package_prefix('ego_planner')) / 'lib/ego_planner/ego_planner_node'
    results = []
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            log = output / (sig.name + '.log')
            with log.open('w') as stream:
                process = subprocess.Popen([
                    'gdb', '-q', '-batch', '--return-child-result', '-x',
                    str(root / 'scripts/shutdown_capture.gdb'), '--args', str(executable),
                    '--ros-args', '--params-file', str(params),
                    '-r', 'grid_map/cloud:=/pointcloud_nonground',
                    '-r', 'grid_map/odom:=/odom', '-r', 'odom_world:=/odom'],
                    stdout=stream, stderr=stream)
                inferior = None
                try:
                    deadline = time.monotonic() + 12
                    accepted = False
                    while time.monotonic() < deadline:
                        assert process.poll() is None, log.read_text()
                        children = Path(f'/proc/{process.pid}/task/{process.pid}/children')
                        for pid in children.read_text().split():
                            try:
                                if Path(f'/proc/{pid}/exe').resolve() == executable.resolve():
                                    inferior = int(pid)
                            except FileNotFoundError:
                                # GDB briefly forks helpers before creating its inferior.
                                continue
                        stamp = node.get_clock().now().to_msg()
                        state = Odometry(header=Header(stamp=stamp, frame_id='odom'))
                        state.pose.pose.position.z = 1.5
                        state.pose.pose.orientation.w = 1.0
                        odom.publish(state)
                        cloud.publish(create_cloud_xyz32(
                            Header(stamp=stamp, frame_id='odom'),
                            [(3.0, 0.0, 1.5), (3.0, 0.1, 1.6)]))
                        rclpy.spin_once(node, timeout_sec=0.03)
                        accepted = 'trusted cloud accepted:' in log.read_text()
                        if accepted and inferior is not None:
                            break
                    assert accepted and inferior is not None, log.read_text()
                    start = time.monotonic()
                    os.kill(inferior, sig)
                    code = process.wait(timeout=5)
                    results.append({'signal': sig.name, 'inferior_pid': inferior,
                                    'debugger_return': code,
                                    'exit_seconds': time.monotonic() - start,
                                    'trusted_cloud_accepted': accepted})
                    assert code == 0, log.read_text()
                    assert 'exited normally' in log.read_text(), log.read_text()
                finally:
                    if process.poll() is None:
                        if inferior is not None:
                            os.kill(inferior, signal.SIGKILL)
                        process.kill()
                        process.wait(timeout=3)
    finally:
        (output / 'result.json').write_text(json.dumps(results, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
