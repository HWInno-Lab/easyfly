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

"""Real ROS action mocks for the SAME C++ SDK through its Python binding."""
import gc
import os
import subprocess
import sys
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from easyfly_msgs.action import Takeoff, Land, NavigateTo, ExecuteMission, PrecisionLand, Explore
from easyfly_msgs.msg import UAVState
import easyfly_pybind as sdk


os.environ["ROS_DOMAIN_ID"] = "92"


class PythonSdkTest(unittest.TestCase):
    def test_explore_typed_goal_feedback_and_reason(self):
        client = sdk.Client()
        self.addCleanup(lambda: client.close(3))
        self.until(lambda: client.server_ready("explore"))
        operation = client.explore([-2., -2., .5], [2., 2., 2.5],
                                   [[[1., 1., .5], [1.5, 1.5, 2.]]], False, 30.)
        self.assertTrue(operation.wait(3))
        state = operation.snapshot()
        self.assertTrue(state.success)
        self.assertEqual(state.message, "done")
        self.assertEqual(state.feedback["phase"], "exploring")
        self.assertEqual(state.feedback["coverage"], .4)
        self.assertEqual(state.feedback["completed_viewpoints"], 2)
        self.assertFalse(self.explore_goal.return_home)
        self.assertEqual(self.explore_goal.timeout_sec, 30.)
        self.assertEqual(len(self.explore_goal.forbidden_boxes), 2)
        self.assertEqual(self.explore_goal.min_bound.z, .5)
        self.assertTrue(client.close(3))

    def test_precision_landing_uses_same_owned_cpp_client(self):
        client = sdk.Client()
        self.until(lambda: client.server_ready("precision_land"))
        operation = client.precision_land(7, timeout=3)
        self.assertTrue(operation.wait(3))
        self.assertTrue(operation.snapshot().success)
        self.assertTrue(client.close(3))

    def test_late_acceptance_subprocess_drain_exits_after_real_cancel_result(self):
        body = """
import time
import sys
from easyfly_pybind import Client, Status
client = Client()
deadline = time.monotonic() + 4
while not client.server_ready('navigate') and time.monotonic() < deadline:
    time.sleep(.01)
assert client.server_ready('navigate'), 'mock navigate server unavailable'
operation = client.navigate(99, 0, 1)
# Ensure submission reaches the server before closing; its goal callback delays
# acceptance by .25s, so ownership must survive the facade being destroyed.
time.sleep(.08)
assert not client.close(0)
del client
assert operation.wait(4), 'owned late goal did not reach a real terminal result'
snapshot = operation.snapshot()
assert snapshot.status == Status.CANCELED, snapshot.message
assert not snapshot.remote_execution_pending
assert not snapshot.success
print('real_cancel_terminal_received', file=sys.stderr, flush=True)
# No additional sleep or facade close: exercise final drain cleanup at exit.
"""
        before = self.cancel_count
        result = subprocess.run(
            [sys.executable, "-c", body], capture_output=True, text=True, timeout=10
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertGreater(self.cancel_count, before)

    def test_idle_subprocess_context_and_implicit_destructor_exit_cleanly(self):
        for body in (
            "with Client() as client: client.state()",
            "client = Client(); client.state(); del client",
            "client = Client(); client.state()",
        ):
            result = subprocess.run(
                [sys.executable, "-c", "from easyfly_pybind import Client\n" + body],
                capture_output=True,
                text=True,
                timeout=8,
            )
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("python_sdk_mock")
        cls.group = ReentrantCallbackGroup()
        cls.cancel_count = 0
        cls.accept_count = 0
        cls.state_x = 12.3
        cls.source_mode = "fresh"
        cls.publisher = cls.node.create_publisher(UAVState, "/uav_state", 10)

        def publish():
            state = UAVState()
            state.header.frame_id = "odom"
            state.header.stamp = cls.node.get_clock().now().to_msg()
            if cls.source_mode == "stale":
                state.header.stamp.sec -= 10
            elif cls.source_mode == "zero":
                state.header.stamp.sec = state.header.stamp.nanosec = 0
            elif cls.source_mode == "future":
                state.header.stamp.sec += 10
            elif cls.source_mode == "wrong_frame":
                state.header.frame_id = "map"
            state.connected = state.pose_valid = True
            state.position_x = cls.state_x
            cls.publisher.publish(state)

        cls.timer = cls.node.create_timer(0.02, publish)
        cls.servers = []
        for action, name in [
            (Takeoff, "/takeoff"),
            (Land, "/land"),
            (NavigateTo, "/navigate"),
            (ExecuteMission, "/execute_mission"),
            (PrecisionLand, "/precision_land"),
            (Explore, "/explore"),
        ]:

            def goal(request):
                if getattr(request, "target_altitude", 0) < 0:
                    return GoalResponse.REJECT
                if getattr(request, "target_x", 0) == 99:
                    time.sleep(0.25)
                cls.accept_count += 1
                return GoalResponse.ACCEPT

            def cancel(handle):
                cls.cancel_count += 1
                return CancelResponse.ACCEPT

            def execute(handle, action=action):
                feedback = action.Feedback()
                if action is Takeoff:
                    feedback.current_altitude = 1.2
                    feedback.progress = 0.5
                elif action is Explore:
                    feedback.phase = "exploring"
                    feedback.coverage = .4
                    feedback.completed_viewpoints = 2
                    cls.explore_goal = handle.request
                elif action is NavigateTo:
                    feedback.distance_remaining = 4.5
                handle.publish_feedback(feedback)
                request = handle.request
                if getattr(request, "target_x", 0) in (10, 99):
                    end = time.monotonic() + 10
                    while not handle.is_cancel_requested and time.monotonic() < end:
                        time.sleep(0.01)
                    if handle.is_cancel_requested:
                        time.sleep(0.15)  # cancel ACK must not masquerade as stopped
                        result = action.Result()
                        result.success, result.message = False, "mock_canceled"
                        handle.canceled()
                        return result
                # Feedback is volatile: repeat while executing, not just one
                # packet raced against a first-goal subscription discovery.
                for _ in range(10):
                    handle.publish_feedback(feedback)
                    time.sleep(0.02)
                result = action.Result()
                if getattr(request, "target_altitude", 0) == 2:
                    result.success, result.message = False, "false_success"
                    handle.succeed()
                elif getattr(request, "target_altitude", 0) == 3:
                    result.success, result.message = False, "mock_abort"
                    handle.abort()
                elif getattr(request, "mission_file", "") == "unsupported":
                    result.success, result.message = False, "unsupported:mission_engine"
                    handle.abort()
                else:
                    result.success = True
                    setattr(result, "reason" if action is Explore else "message", "done")
                    handle.succeed()
                return result

            cls.servers.append(
                ActionServer(
                    cls.node,
                    action,
                    name,
                    execute,
                    goal_callback=goal,
                    cancel_callback=cancel,
                    callback_group=cls.group,
                )
            )
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=5)
        cls.thread.join(timeout=5)
        for server in cls.servers:
            server.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def until(self, predicate, timeout=4):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            time.sleep(0.01)
        self.assertTrue(predicate())

    def setUp(self):
        type(self).source_mode = "fresh"

    def client(self, **kwargs):
        client = sdk.Client(**kwargs)
        self.until(
            lambda: all(
                client.server_ready(n) for n in ("takeoff", "land", "navigate", "execute_mission")
            )
        )
        self.addCleanup(lambda: client.close(3))
        return client

    def test_success_feedback_false_success_reject_failure_and_unsupported(self):
        client = self.client()
        before = self.cancel_count
        operation = client.takeoff(1)
        self.assertFalse(operation.wait(0.001))
        self.assertTrue(operation.wait(3))
        self.assertTrue(operation.snapshot().success)
        self.assertEqual(operation.snapshot().feedback["current_altitude"], 1.2)
        self.assertEqual(self.cancel_count, before)  # polling wait never cancels
        for altitude, expected in [
            (-1, sdk.Status.REJECTED),
            (2, sdk.Status.FAILED),
            (3, sdk.Status.FAILED),
        ]:
            operation = client.takeoff(altitude)
            self.assertTrue(operation.wait(3))
            self.assertEqual(operation.snapshot().status, expected)
            self.assertFalse(operation.snapshot().success)
        operation = client.execute_mission("unsupported")
        self.assertTrue(operation.wait(3))
        self.assertEqual(operation.snapshot().status, sdk.Status.UNSUPPORTED)
        self.assertFalse(operation.snapshot().success)
        self.assertEqual(
            client.unsupported("landing_tags").snapshot().status, sdk.Status.UNSUPPORTED
        )
        operation = client.land()
        self.assertTrue(operation.wait(3))
        self.assertTrue(operation.snapshot().success)
        operation = client.execute_mission("validated.yaml")
        self.assertTrue(operation.wait(3))
        self.assertTrue(operation.snapshot().success)

    def test_cancel_ack_not_terminal_and_capacity(self):
        client = self.client(max_pending_operations=1)
        operation = client.navigate(10, 0, 1)
        rejected = client.navigate(0, 0, 1)
        self.assertEqual(rejected.snapshot().status, sdk.Status.CAPACITY_EXCEEDED)
        self.until(lambda: operation.snapshot().admission == sdk.Admission.ACCEPTED)
        self.assertTrue(operation.cancel())
        self.until(lambda: operation.snapshot().cancel_status == sdk.CancelStatus.ACCEPTED)
        self.assertTrue(operation.snapshot().remote_execution_pending)
        self.assertFalse(operation.snapshot().success)
        self.assertTrue(operation.wait(3))
        self.assertEqual(operation.snapshot().status, sdk.Status.CANCELED)

    def test_close_late_acceptance_survives_facade_destruction(self):
        client = sdk.Client()
        self.until(lambda target=client: target.server_ready("navigate"))
        operation = client.navigate(99, 0, 1)
        time.sleep(0.06)
        self.assertFalse(client.close(0.001))
        self.assertEqual(operation.snapshot().local_status, sdk.LocalStatus.CLOSED)
        self.assertTrue(operation.snapshot().remote_execution_pending)
        del client
        gc.collect()
        self.assertTrue(operation.wait(3))
        self.assertEqual(operation.snapshot().status, sdk.Status.CANCELED)

    def test_admission_timeout_state_freeze_context_manager(self):
        client = self.client(admission_timeout_seconds=0.03, cancel_on_admission_timeout=True)
        self.until(lambda: client.state()["received"])
        state = client.state()
        self.assertTrue(state["fresh"])
        self.assertEqual(state["position"][0], 12.3)
        operation = client.navigate(99, 0, 1)
        time.sleep(0.08)
        self.assertEqual(operation.snapshot().local_status, sdk.LocalStatus.ADMISSION_TIMED_OUT)
        self.assertTrue(operation.snapshot().remote_execution_pending)
        self.assertTrue(operation.wait(3))
        self.assertEqual(operation.snapshot().status, sdk.Status.CANCELED)
        self.assertTrue(client.close(3))
        stamp = client.state()["stamp_nanosec"]
        time.sleep(0.08)
        self.assertEqual(client.state()["stamp_nanosec"], stamp)
        self.assertFalse(client.state()["fresh"])
        self.assertEqual(client.takeoff(1).snapshot().status, sdk.Status.NOT_SENT)
        with sdk.Client() as other:
            self.assertTrue(other.running())
        self.assertFalse(other.running())

    def test_receipt_freshness_does_not_hide_stale_or_invalid_source_time(self):
        client = self.client()
        self.until(lambda: client.state()["fresh"])
        for mode in ("stale", "zero", "future", "wrong_frame"):
            type(self).source_mode = mode
            time.sleep(0.08)
            state = client.state()
            self.assertTrue(state["receipt_fresh"])
            self.assertFalse(state["fresh"], mode)
        type(self).source_mode = "fresh"
        with sdk.Client(use_sim_time=True) as sim_client:
            self.until(lambda: sim_client.state()["received"])
            self.assertTrue(sim_client.state()["receipt_fresh"])
            self.assertFalse(sim_client.state()["source_time_valid"])
            self.assertFalse(sim_client.state()["fresh"])  # No /clock in this mock graph.


if __name__ == "__main__":
    unittest.main()
