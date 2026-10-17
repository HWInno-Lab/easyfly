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

"""Real reviewed facade, isolated mock Actions; no flight stack or UI writes."""
import hashlib
import os
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import uuid

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from easyfly_msgs.action import ExecuteMission
from easyfly_msgs.srv import GetActionOwnership, GetCapabilities, GetMissionDetails, ManageMission
from easyfly_msgs.srv import ManageReviewedMission


class ReviewedMissionTest(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '111')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        self.process = self.log = self.directory = self.node = self.backend = None
        self.context = self.executor = self.thread = None
        self.goals, self.cancellations, self.snapshots = [], [], []
        self.terminal = set()
        self.delay = 0
        self.reject = self.complete = False
        self.release = True
        self.result_success = True
        self.addCleanup(self.cleanup)
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('reviewed_backend', context=self.context)
        self.directory = tempfile.TemporaryDirectory(prefix='easyfly-ui-reviewed-')
        self.root = Path(self.directory.name)
        self.content = b'# reviewed\ntype=hover\nwait=0.1\n'
        (self.root / 'demo.mission').write_bytes(self.content)
        self.backend = ActionServer(
            self.node, ExecuteMission, '/execute_mission', self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())
        self.owner = self.node.create_service(
            GetActionOwnership, '/mission/ownership', self.ownership)
        self.client = self.node.create_client(
            ManageReviewedMission, '/platform/mission/reviewed')
        self.details = self.node.create_client(GetMissionDetails, '/platform/mission/details')
        self.legacy = self.node.create_client(ManageMission, '/platform/mission/manage')
        self.capabilities = self.node.create_client(
            GetCapabilities, '/platform/mission/capabilities')
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.log = tempfile.TemporaryFile(mode='w+')
        self.launch()
        self.instance = self.call('session').server_instance_id
        self.assertRegex(self.instance, r'^[0-9a-f]{32}$')
        self.assertEqual(self.call('session').request_capacity, 256)
        time.sleep(0.2)  # Allow mock Action and ownership discovery to settle.

    def launch(self, capacity=256):
        binary = Path(get_package_prefix('easyfly_mission_srvs')) / (
            'lib/easyfly_mission_srvs/mission_services')
        self.process = subprocess.Popen(
            [str(binary), '--ros-args', '-p', f'mission_directory:={self.root}',
             '-p', 'response_timeout:=0.15', '-p', f'reviewed_request_capacity:={capacity}'],
            stdout=self.log, stderr=self.log, start_new_session=True)
        self.assertTrue(self.client.wait_for_service(timeout_sec=10))

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)

    def cleanup(self):
        self.stop()
        self.complete = True
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=5)
        if self.thread is not None:
            self.thread.join(timeout=5)
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

    def accept(self, goal):
        self.goals.append(goal.mission_file)
        time.sleep(self.delay)
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def cancel(self, handle):
        self.cancellations.append(bytes(handle.goal_id.uuid).hex())
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        identity = bytes(handle.goal_id.uuid).hex()
        self.snapshots.append((Path(handle.request.mission_file),
                               Path(handle.request.mission_file).read_bytes()))
        finished = rclpy.task.Future()

        def poll():
            if not finished.done() and (self.complete or handle.is_cancel_requested):
                finished.set_result(True)

        timer = self.node.create_timer(0.01, poll)
        await finished
        self.node.destroy_timer(timer)
        self.terminal.add(identity)
        if handle.is_cancel_requested:
            handle.canceled()
        elif self.result_success:
            handle.succeed()
        else:
            handle.abort()
        return ExecuteMission.Result(success=self.result_success, message='mock completed')

    def ownership(self, request, response):
        response.goal_id = request.goal_id
        response.state = 2 if self.release and request.goal_id in self.terminal else 1
        return response

    def wait(self, future):
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            self.assertIsNone(self.process.poll())
            time.sleep(0.005)
        self.assertTrue(future.done())
        return future.result()

    def request(self, operation, request_id='', name='demo.mission', digest=None, instance=None,
                execution=''):
        return ManageReviewedMission.Request(
            operation=operation, server_instance_id=instance or getattr(self, 'instance', ''),
            client_request_id=request_id, mission_name=name,
            expected_content_sha256=digest or hashlib.sha256(self.content).hexdigest(),
            execution_id=execution)

    def call(self, operation, request_id='', **kwargs):
        return self.wait(self.client.call_async(self.request(operation, request_id, **kwargs)))

    def until(self, predicate, request_id):
        deadline = time.monotonic() + 5
        current = self.call('lookup', request_id)
        while not predicate(current) and time.monotonic() < deadline:
            time.sleep(0.02)
            current = self.call('lookup', request_id)
        self.assertTrue(predicate(current), current)
        return current

    def test_reviewed_bytes_bound_to_private_snapshot(self):
        preview = self.wait(self.details.call_async(
            GetMissionDetails.Request(mission_name='demo.mission')))
        self.assertEqual(preview.content.encode(), self.content)
        request_id = uuid.uuid4().hex
        started = self.call('start', request_id, digest=preview.content_sha256)
        self.assertTrue(started.success)
        self.assertEqual(started.admission_status, 'accepted')
        self.assertTrue(started.execution_pending)
        self.assertRegex(started.action_goal_id, r'^[0-9a-f]{32}$')
        (self.root / 'demo.mission').write_text('type=land\n')
        duplicate = self.call('start', request_id, digest=preview.content_sha256)
        self.assertEqual(duplicate.execution_id, started.execution_id)
        self.complete = True
        terminal = self.until(lambda r: not r.execution_pending, request_id)
        self.assertEqual(terminal.status, 'succeeded')
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(self.snapshots[0][1], self.content)
        self.assertFalse(self.snapshots[0][0].exists())
        self.assertEqual(self.call('start', request_id).execution_id, started.execution_id)
        self.assertEqual(len(self.goals), 1)

    def test_audit_distinguishes_request_replay_and_sender_without_poll_noise(self):
        request_id = uuid.uuid4().hex
        started = self.call('start', request_id)
        self.assertTrue(started.success)
        self.call('start', request_id)
        other = self.node.create_client(ManageReviewedMission, '/platform/mission/reviewed')
        self.assertTrue(other.wait_for_service(timeout_sec=5))
        self.wait(other.call_async(self.request('start', request_id)))
        self.node.destroy_client(other)
        self.call('lookup', request_id)
        self.call('session')
        text = os.pread(self.log.fileno(), 1024 * 1024, 0).decode()
        lines = [line.split('managed_audit ', 1)[1] for line in text.splitlines()
                 if 'managed_audit ' in line]
        records = [dict(field.split('=', 1) for field in line.split()) for line in lines]
        received = [r for r in records if r['event'] == 'write_received']
        self.assertEqual(len(received), 3)
        self.assertEqual({r['request'] for r in received}, {request_id})
        self.assertEqual({r['instance'] for r in received}, {self.instance})
        self.assertEqual({r['domain'] for r in received}, {'111'})
        self.assertEqual(len({r['writer_guid'] for r in received}), 2)
        self.assertEqual(len({(r['writer_guid'], r['sequence']) for r in received}), 3)
        for record in received:
            self.assertIsNotNone(re.fullmatch('[0-9a-f]{32}', record['writer_guid']))
        self.assertEqual(sum(r['event'] == 'start_new_request' for r in records), 1)
        self.assertEqual(sum(r['event'] == 'start_existing_request' for r in records), 2)
        response = next(r for r in records if r['event'] == 'start_response')
        self.assertEqual(response['execution'], started.execution_id)
        self.assertEqual(response['goal'], started.action_goal_id)
        self.assertNotIn('demo.mission', text)
        self.assertNotIn('# reviewed', text)
        self.assertEqual(len(self.goals), 1)

    def test_changed_content_rejected_and_not_retried(self):
        request_id = uuid.uuid4().hex
        (self.root / 'demo.mission').write_text('type=land\n')
        response = self.call('start', request_id)
        self.assertEqual(response.status, 'content_changed')
        self.assertTrue(response.admission_resolved)
        self.assertFalse(response.execution_id)
        (self.root / 'demo.mission').write_bytes(self.content)
        self.assertEqual(self.call('start', request_id).status, 'content_changed')
        conflict = self.call('start', request_id, digest='a' * 64)
        self.assertEqual(conflict.status, 'request_conflict')
        self.assertEqual(self.call('start', request_id, name='other.mission').status,
                         'request_conflict')
        self.assertEqual(self.call('lookup', request_id).status, 'content_changed')
        self.assertFalse(self.goals)

    def test_pending_duplicate_and_lost_reply_recovery(self):
        self.delay = 0.4
        request_id = uuid.uuid4().hex
        request = self.request('start', request_id)
        future = self.client.call_async(request)
        pending = self.until(lambda r: bool(r.execution_id), request_id)
        self.assertFalse(pending.admission_resolved)
        self.assertTrue(pending.execution_pending)
        duplicate = self.call('start', request_id)
        self.assertEqual(duplicate.execution_id, pending.execution_id)
        # Simulate losing the caller's reply: remove local pending tracking.
        self.client.remove_pending_request(future)
        recovered = self.until(lambda r: r.admission_resolved, request_id)
        self.assertEqual(recovered.admission_status, 'pending_timeout')
        terminal = self.until(lambda r: not r.execution_pending, request_id)
        self.assertEqual(terminal.status, 'canceled')
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(self.cancellations, [terminal.action_goal_id])
        self.assertEqual(self.call('start', request_id).execution_id, terminal.execution_id)

    def test_public_terminal_does_not_release_lock(self):
        self.release = False
        first_id, second_id = uuid.uuid4().hex, uuid.uuid4().hex
        self.assertTrue(self.call('start', first_id).execution_id)
        self.complete = True
        current = self.until(lambda r: r.status == 'succeeded', first_id)
        self.assertTrue(current.execution_pending)
        busy = self.call('start', second_id)
        self.assertEqual(busy.status, 'busy')
        self.assertFalse(busy.execution_id)  # Never binds to the first caller's execution.
        self.assertFalse(self.call('lookup', second_id).execution_id)
        self.release = True
        self.until(lambda r: not r.execution_pending, first_id)
        self.assertEqual(self.call('start', second_id).status, 'busy')
        third = self.call('start', uuid.uuid4().hex)
        self.assertTrue(third.execution_id)
        self.assertEqual(len(self.goals), 2)

    def test_rejection_is_retained_without_second_goal(self):
        self.reject = True
        request_id = uuid.uuid4().hex
        rejected = self.call('start', request_id)
        self.assertEqual(rejected.status, 'rejected')
        self.assertFalse(rejected.execution_pending)
        self.assertTrue(rejected.admission_resolved)
        self.assertFalse(rejected.admission_success)
        self.reject = False
        self.assertEqual(self.call('start', request_id).execution_id, rejected.execution_id)
        self.assertEqual(len(self.goals), 1)

    def test_capacity_retains_old_records_and_restart_refuses_old_instance(self):
        old_request_id = uuid.uuid4().hex
        self.assertEqual(self.call('start', old_request_id, digest='0' * 64).status,
                         'content_changed')
        self.stop()
        old_instance = self.instance
        self.launch(capacity=2)
        self.instance = self.call('session').server_instance_id
        self.assertNotEqual(self.instance, old_instance)
        self.assertEqual(self.call('lookup', old_request_id).status, 'not_found')
        self.assertEqual(self.call('lookup', old_request_id, instance=old_instance).status,
                         'instance_mismatch')
        self.assertEqual(self.call('start', old_request_id, instance=old_instance).status,
                         'instance_mismatch')
        ids = [uuid.uuid4().hex for _ in range(3)]
        for request_id in ids[:2]:
            self.assertEqual(self.call('start', request_id, digest='0' * 64).status,
                             'content_changed')
        self.assertEqual(self.call('start', ids[2]).status, 'request_capacity')
        self.assertEqual(self.call('lookup', ids[0]).status, 'content_changed')
        self.assertEqual(self.call('start', ids[0], digest='0' * 64).status, 'content_changed')
        self.assertEqual(self.call('session').retained_requests, 2)
        self.assertEqual(self.call('lookup', ids[0], instance=old_instance).status,
                         'instance_mismatch')
        self.assertEqual(self.call('start', ids[0], instance=old_instance).status,
                         'instance_mismatch')
        self.assertFalse(self.goals)

    def test_failed_result_is_retained_and_cannot_start_again(self):
        self.result_success = False
        request_id = uuid.uuid4().hex
        admitted = self.call('start', request_id)
        self.complete = True
        failed = self.until(lambda r: not r.execution_pending, request_id)
        self.assertEqual(failed.status, 'failed')
        duplicate = self.call('start', request_id)
        self.assertEqual(duplicate.execution_id, admitted.execution_id)
        self.assertEqual(duplicate.status, 'failed')
        self.assertTrue(duplicate.admission_success)
        self.assertEqual(len(self.goals), 1)

    def test_request_lookup_survives_legacy_history_pruning(self):
        self.complete = True
        first_id, first_execution = '', ''
        for index in range(35):
            request_id = uuid.uuid4().hex
            admitted = self.call('start', request_id)
            self.assertTrue(admitted.execution_id)
            self.until(lambda r: not r.execution_pending, request_id)
            if index == 0:
                first_id, first_execution = request_id, admitted.execution_id
        legacy = self.wait(self.legacy.call_async(ManageMission.Request(
            operation='query', execution_id=first_execution)))
        self.assertEqual(legacy.status, 'not_found')
        retained = self.call('lookup', first_id)
        self.assertEqual(retained.execution_id, first_execution)
        self.assertEqual(retained.status, 'succeeded')
        self.assertFalse(retained.execution_pending)
        self.assertEqual(self.call('start', first_id).execution_id, first_execution)
        self.assertEqual(len(self.goals), 35)

    def test_unavailable_record_does_not_retry_after_backend_recovery(self):
        def readiness(expected):
            deadline = time.monotonic() + 5
            ready = not expected
            while ready != expected and time.monotonic() < deadline:
                ready = self.wait(self.capabilities.call_async(
                    GetCapabilities.Request())).ready[0]
                time.sleep(0.01)
            self.assertEqual(ready, expected)

        self.node.destroy_service(self.owner)
        self.owner = None
        readiness(False)
        request_id = uuid.uuid4().hex
        self.assertEqual(self.call('start', request_id).status, 'unsupported')
        self.assertFalse(self.goals)
        self.owner = self.node.create_service(
            GetActionOwnership, '/mission/ownership', self.ownership)
        readiness(True)
        self.assertEqual(self.call('start', request_id).status, 'unsupported')
        self.assertFalse(self.goals)
        new_id = uuid.uuid4().hex
        self.assertTrue(self.call('start', new_id).execution_id)
        self.complete = True
        self.until(lambda r: not r.execution_pending, new_id)
        self.assertEqual(len(self.goals), 1)

    def test_invalid_identity_path_schema_and_unknown_lookup(self):
        self.assertEqual(self.call('lookup', uuid.uuid4().hex).status, 'not_found')
        self.assertEqual(self.call('start', 'not-a-request-id').status, 'invalid')
        self.assertEqual(self.call('start', uuid.uuid4().hex, digest='A' * 64).status, 'invalid')
        self.assertEqual(self.call('start', uuid.uuid4().hex, name='../demo.mission').status,
                         'invalid')
        (self.root / 'broken.yaml').write_text('schema_version: 2\nsteps: []\n')
        self.assertEqual(self.call('start', uuid.uuid4().hex, name='broken.yaml').status,
                         'invalid')
        self.assertEqual(self.call('unknown', uuid.uuid4().hex).status, 'unsupported')
        self.assertFalse(self.goals)

    def test_managed_cancel_identity_ack_and_real_ownership_drain(self):
        self.release = False
        request_id = uuid.uuid4().hex
        admitted = self.call('start', request_id)
        execution = admitted.execution_id
        wrong = self.call('cancel', request_id, execution='f' * 32)
        self.assertEqual(wrong.status, 'execution_mismatch')
        self.assertEqual(
            self.call('cancel', request_id, execution=execution, name='other.mission').status,
            'request_conflict')
        self.assertEqual(self.call('cancel', uuid.uuid4().hex, execution=execution).status,
                         'not_found')
        self.assertEqual(
            self.call('cancel', request_id, execution=execution, instance='f' * 32).status,
            'instance_mismatch')
        self.assertFalse(self.cancellations)
        canceled = self.call('cancel', request_id, execution=execution)
        self.assertEqual(canceled.status, 'cancel_requested')
        self.assertTrue(canceled.execution_pending)
        self.assertEqual(canceled.admission_status, 'accepted')
        self.assertTrue(canceled.admission_success)
        current = self.until(lambda r: r.status == 'canceled', request_id)
        self.assertTrue(current.execution_pending)
        self.assertEqual(self.cancellations, [admitted.action_goal_id])
        self.release = True
        self.until(lambda r: not r.execution_pending, request_id)
        terminal = self.call('cancel', request_id, execution=execution)
        self.assertFalse(terminal.success)
        self.assertEqual(terminal.status, 'canceled')
        self.assertFalse(terminal.execution_pending)
        self.assertEqual(terminal.admission_status, 'accepted')

    def test_managed_cancel_before_late_admission_never_rewrites_admission(self):
        self.delay = 0.4
        request_id = uuid.uuid4().hex
        future = self.client.call_async(self.request('start', request_id))
        pending = self.until(lambda r: bool(r.execution_id), request_id)
        canceled = self.call('cancel', request_id, execution=pending.execution_id)
        self.assertIn(canceled.status, ('cancel_timeout', 'cancel_requested'))
        self.wait(future)
        terminal = self.until(lambda r: not r.execution_pending, request_id)
        self.assertEqual(terminal.status, 'canceled')
        self.assertEqual(terminal.admission_status, 'pending_timeout')
        self.assertFalse(terminal.admission_success)
        self.assertEqual(self.cancellations, [terminal.action_goal_id])


if __name__ == '__main__':
    unittest.main()
