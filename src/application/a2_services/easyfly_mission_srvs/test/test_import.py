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

"""Create-only catalog imports against the real node, with zero action goals."""
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
from easyfly_msgs.srv import GetMissionDetails, ImportMission, ManageMission

BODY = ('# exact reviewed bytes\r\n'
        'schema_version: 1\r\nframe_id: odom\r\n'
        'units: {position: m, angle: rad, time: s}\r\n'
        'steps:\r\n- type: hover\r\n  duration: 2\r\n')


class ImportTest(unittest.TestCase):
    def setUp(self):
        self.process = self.log = self.directory = self.node = self.backend = None
        self.context = self.executor = None
        self.goals = 0
        self.addCleanup(self.cleanup)
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '127')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        self.directory = tempfile.TemporaryDirectory(prefix='easyfly-import-test-')
        self.root = Path(self.directory.name)
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('catalog_import_test', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.backend = ActionServer(
            self.node, ExecuteMission, '/execute_mission',
            lambda handle: ExecuteMission.Result(), goal_callback=self.reject_goal)
        self.api = self.node.create_client(ImportMission, '/platform/mission/import')
        self.details = self.node.create_client(GetMissionDetails, '/platform/mission/details')
        self.manage = self.node.create_client(ManageMission, '/platform/mission/manage')
        self.log = tempfile.TemporaryFile(mode='w+')
        self.start()

    def reject_goal(self, _goal):
        self.goals += 1
        return GoalResponse.REJECT

    def start(self, enabled=True, capacity=8):
        binary = Path(get_package_prefix('easyfly_mission_srvs')) / (
            'lib/easyfly_mission_srvs/mission_services')
        self.process = subprocess.Popen(
            [str(binary), '--ros-args', '-p', f'mission_directory:={self.root}',
             '-p', f'enable_mission_import:={str(enabled).lower()}',
             '-p', f'import_request_capacity:={capacity}'],
            stdout=self.log, stderr=self.log, start_new_session=True)
        self.assertTrue(self.api.wait_for_service(timeout_sec=8))
        self.session = self.call(ImportMission.Request(operation='session'))

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=4)
        end = time.monotonic() + 5
        while self.api.service_is_ready() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.05)
        self.assertFalse(self.api.service_is_ready())

    def cleanup(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=4)
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

    def call(self, request, client=None):
        client = client or self.api
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        future = client.call_async(request)
        end = time.monotonic() + 5
        while not future.done() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)
            self.assertIsNone(self.process.poll())
        self.assertTrue(future.done())
        return future.result()

    def request(self, name='new.yaml', body=BODY, identity='a' * 32):
        return ImportMission.Request(
            operation='create', server_instance_id=self.session.server_instance_id,
            catalog_id=self.session.catalog_id, client_request_id=identity,
            mission_name=name, content=body,
            expected_content_sha256=hashlib.sha256(body.encode()).hexdigest())

    def lookup(self, identity='a' * 32):
        return self.call(ImportMission.Request(
            operation='lookup', server_instance_id=self.session.server_instance_id,
            catalog_id=self.session.catalog_id, client_request_id=identity))

    def test_exact_bytes_readback_catalog_and_duplicate(self):
        self.assertTrue(self.session.enabled)
        self.assertEqual(self.session.catalog_directory, str(self.root))
        request = self.request()
        result = self.call(request)
        self.assertTrue(result.success and result.resolved and result.published)
        path = self.root / 'new.yaml'
        self.assertEqual(path.read_bytes(), BODY.encode())
        self.assertEqual(path.stat().st_mode & 0o777, 0o600)
        inode = path.stat().st_ino
        details = self.call(GetMissionDetails.Request(mission_name='new.yaml'), self.details)
        self.assertTrue(details.success)
        self.assertEqual(details.content, BODY)
        self.assertEqual(details.content_sha256, request.expected_content_sha256)
        catalog = self.call(ManageMission.Request(operation='list'), self.manage)
        self.assertIn('new.yaml', catalog.missions)
        self.assertTrue(self.call(request).success)
        self.assertEqual(path.stat().st_ino, inode)
        self.assertTrue(self.lookup().success)
        path.write_text('externally changed')
        self.assertTrue(self.call(request).published)  # Historical decision, not readback.
        self.assertEqual(path.read_text(), 'externally changed')
        self.assertEqual(self.call(self.request(body=BODY + '# changed\n')).status,
                         'request_conflict')
        self.assertEqual(self.lookup('b' * 32).status, 'not_found')

    def test_existing_files_links_and_fifo_are_never_overwritten(self):
        target = self.root / 'existing.yaml'
        target.write_text('original bytes')
        (self.root / 'link.yaml').symlink_to(target)
        os.link(target, self.root / 'hard.yaml')
        os.mkfifo(self.root / 'pipe.yaml')
        for i, name in enumerate(('existing.yaml', 'link.yaml', 'hard.yaml', 'pipe.yaml')):
            with self.subTest(name=name):
                result = self.call(self.request(name, identity=f'{i:032x}'))
                self.assertEqual(result.status, 'exists')
                self.assertTrue(result.resolved)
                self.assertFalse(result.published)
        self.assertEqual(target.read_text(), 'original bytes')
        self.assertTrue((self.root / 'link.yaml').is_symlink())
        self.assertFalse(list(self.root.glob('.easyfly-import-*.tmp')))

    def test_invalid_names_hash_format_and_identity_do_not_publish(self):
        for name in ('../outside.yaml', '/tmp/outside.yaml', '.hidden.yaml', 'x.mission'):
            self.assertEqual(self.call(self.request(name)).status, 'invalid')
        request = self.request()
        request.expected_content_sha256 = '0' * 64
        self.assertEqual(self.call(request).status, 'invalid')
        request = self.request()
        request.server_instance_id = '0' * 32
        self.assertEqual(self.call(request).status, 'instance_mismatch')
        for i, body in enumerate(('type=hover\nwait=2\n', 'schema_version: 2\n',
                                  BODY + 'unknown: 4\n')):
            result = self.call(self.request(body=body, identity=f'{i:032x}'))
            self.assertEqual(result.status, 'rejected')
            self.assertTrue(result.resolved)
        self.assertFalse((self.root / 'new.yaml').exists())

    def test_capacity_no_eviction_and_restart_never_replays(self):
        request = self.request()
        self.assertTrue(self.call(request).success)
        self.stop()
        self.start(capacity=1)
        self.assertNotEqual(request.server_instance_id, self.session.server_instance_id)
        self.assertEqual(self.call(request).status, 'instance_mismatch')
        occupied = self.request('new.yaml', identity='b' * 32)
        self.assertEqual(self.call(occupied).status, 'exists')
        self.assertEqual(self.call(self.request('next.yaml', identity='c' * 32)).status,
                         'request_capacity')
        self.assertEqual(self.lookup('b' * 32).status, 'exists')
        self.assertEqual((self.root / 'new.yaml').read_bytes(), BODY.encode())
        self.assertFalse((self.root / 'next.yaml').exists())

    def test_root_replacement_disables_writes(self):
        # Move the catalog and replace its original path, all within test-owned storage.
        parent = self.root.parent
        saved = parent / (self.root.name + '-moved')
        self.root.rename(saved)
        self.root.mkdir(mode=0o700)
        try:
            result = self.call(self.request())
            self.assertEqual(result.status, 'disabled')
            self.assertFalse(result.enabled or result.published)
            self.assertFalse((saved / 'new.yaml').exists())
            self.assertFalse((self.root / 'new.yaml').exists())
        finally:
            self.root.rmdir()
            saved.rename(self.root)

    def test_disabled_endpoint_has_no_write_fallback(self):
        self.stop()
        self.start(enabled=False)
        self.assertFalse(self.session.enabled)
        self.assertEqual(self.call(self.request()).status, 'disabled')
        self.assertFalse((self.root / 'new.yaml').exists())

    def test_invalid_startup_catalogs_are_not_adopted(self):
        self.stop()
        binary = Path(get_package_prefix('easyfly_mission_srvs')) / (
            'lib/easyfly_mission_srvs/mission_services')

        def refused(path=None):
            arguments = [str(binary), '--ros-args', '-p', 'enable_mission_import:=true']
            if path is not None:
                arguments += ['-p', f'mission_directory:={path}']
            result = subprocess.run(arguments, capture_output=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)

        refused()  # Installed source/package catalog must stay untouched.
        ordinary = self.root / 'ordinary'
        ordinary.mkdir(mode=0o700)
        (ordinary / 'user.yaml').write_text('original')
        refused(ordinary)
        self.assertEqual((ordinary / 'user.yaml').read_text(), 'original')
        self.assertFalse((ordinary / '.easyfly-mission-catalog-v1').exists())
        link = self.root / 'link'
        link.symlink_to(self.root, target_is_directory=True)
        refused(link)
        os.chmod(self.root, 0o755)
        try:
            refused(self.root)
        finally:
            os.chmod(self.root, 0o700)


if __name__ == '__main__':
    unittest.main()
