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

"""Read-only details against the real service node, never a flight backend."""
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from easyfly_msgs.action import ExecuteMission
from easyfly_msgs.srv import GetMissionDetails


class MissionDetailsTest(unittest.TestCase):
    def setUp(self):
        self.process = self.log = self.directory = self.node = self.backend = None
        self.context = self.executor = None
        self.goals = 0
        self.addCleanup(self.cleanup)
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('mission_details_test', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.directory = tempfile.TemporaryDirectory(prefix='easyfly-ui-details-')
        self.root = Path(self.directory.name)
        self.goals = 0
        self.backend = ActionServer(
            self.node, ExecuteMission, '/execute_mission',
            lambda handle: ExecuteMission.Result(), goal_callback=self.reject_goal)
        self.log = tempfile.TemporaryFile(mode='w+')
        binary = Path(get_package_prefix('easyfly_mission_srvs')) / (
            'lib/easyfly_mission_srvs/mission_services')
        self.process = subprocess.Popen(
            [str(binary), '--ros-args', '-p', f'mission_directory:={self.root}'],
            stdout=self.log, stderr=self.log, start_new_session=True)
        self.client = self.node.create_client(GetMissionDetails, '/platform/mission/details')
        self.assertTrue(self.client.wait_for_service(timeout_sec=10))

    def reject_goal(self, _goal):
        self.goals += 1
        return GoalResponse.REJECT

    def cleanup(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
        if self.executor is not None:
            self.executor.shutdown()
        if self.backend is not None:
            self.backend.destroy()
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None and self.context.ok():
            self.context.shutdown()
        if self.log is not None:
            self.log.close()
        if self.directory is not None:
            self.directory.cleanup()
        self.assertEqual(self.goals, 0)

    def details(self, name):
        future = self.client.call_async(GetMissionDetails.Request(mission_name=name))
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            self.assertIsNone(self.process.poll())
        self.assertTrue(future.done())
        response = future.result()
        self.assertEqual(response.mission_name, name)
        return response

    def test_exact_bytes_digest_and_catalog_mutation(self):
        first = b'# comment\n---\ntype=hover\nwait=2\n'
        path = self.root / 'demo.mission'
        path.write_bytes(first)
        response = self.details(path.name)
        self.assertTrue(response.success)
        self.assertEqual(response.status, 'valid_snapshot')
        self.assertEqual(response.content.encode(), first)
        self.assertEqual(response.content_sha256, hashlib.sha256(first).hexdigest())
        path.write_text('type=land\n')
        changed = self.details(path.name)
        self.assertTrue(changed.success)
        self.assertNotEqual(changed.content_sha256, response.content_sha256)
        self.assertEqual(response.content.encode(), first)
        self.assertEqual({p.name for p in self.root.iterdir()}, {'demo.mission'})

    def test_yaml_and_static_defaults_are_not_rewritten(self):
        content = ('schema_version: 1\nframe_id: odom\n'
                   'units: {position: m, angle: rad, time: s}\n'
                   'steps:\n- {type: takeoff, altitude: 1.5}\n- {type: land}\n')
        (self.root / 'flight.yaml').write_text(content)
        response = self.details('flight.yaml')
        self.assertTrue(response.success)
        self.assertEqual(response.content, content)
        self.assertEqual(response.content_sha256, hashlib.sha256(content.encode()).hexdigest())

    def test_reject_invalid_paths_symlinks_and_non_regular_files(self):
        (self.root / 'escape.yaml').symlink_to('/etc/passwd')
        (self.root / 'directory.yaml').mkdir()
        (self.root / 'hidden.yaml').write_text('type=land\n')
        (self.root / '.hidden.yaml').write_text('type=land\n')
        (self.root / 'wrong.txt').write_text('type=land\n')
        os.mkfifo(self.root / 'fifo.yaml')
        for name in ('../outside.yaml', '/etc/passwd', 'escape.yaml', 'directory.yaml',
                     '.hidden.yaml', 'wrong.txt', 'fifo.yaml', 'missing.yaml'):
            response = self.details(name)
            self.assertFalse(response.success, name)
            self.assertEqual(response.content, '')
            self.assertEqual(response.content_sha256, '')

    def test_reject_whole_invalid_or_over_budget_mission(self):
        contents = (
            'type=land\n---\ntype=unknown\n',
            'schema_version: 2\nsteps: [{type: land}]\n',
            'type=land\n\x00',
            '#' * 65537,
        )
        for content in contents:
            (self.root / 'invalid.yaml').write_text(content)
            response = self.details('invalid.yaml')
            self.assertFalse(response.success)
            self.assertEqual(response.content, '')
            self.assertEqual(response.content_sha256, '')
        (self.root / 'invalid.yaml').write_bytes(b'#\xff\ntype=land\n')
        response = self.details('invalid.yaml')
        self.assertFalse(response.success)
        self.assertEqual(response.content, '')


if __name__ == '__main__':
    unittest.main()
