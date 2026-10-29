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

"""A canceled backend's late success cannot erase the public time budget."""
import os
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from easyfly_msgs.action import NavigateTo


class AstarDeadlineTest(unittest.TestCase):
    def test_late_success_after_budget(self):
        rclpy.init()
        node = rclpy.create_node("astar_deadline_mock")

        async def execute(handle):
            ready = rclpy.task.Future()
            timer = node.create_timer(
                0.35, lambda: ready.set_result(True) if not ready.done() else None
            )
            await ready
            node.destroy_timer(timer)
            handle.succeed()  # A deliberately misbehaving/late backend.
            return NavigateTo.Result(success=True, message="late arrival")

        backend = ActionServer(
            node,
            NavigateTo,
            "/navigate_astar",
            execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "easyfly_global_planner",
                "navigation_coordinator_node",
                "--ros-args",
                "-p",
                "goal_timeout:=0.2",
            ],
            start_new_session=True,
        )
        client = ActionClient(node, NavigateTo, "/navigate")

        def wait(future):
            until = time.monotonic() + 5
            while not future.done() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertTrue(future.done())
            return future.result()

        try:
            self.assertTrue(client.wait_for_server(timeout_sec=10))
            handle = wait(client.send_goal_async(NavigateTo.Goal(target_x=1.0, target_z=1.0)))
            self.assertTrue(handle.accepted)
            result = wait(handle.get_result_async())
            self.assertEqual(result.status, 6)
            self.assertFalse(result.result.success)
            self.assertEqual(result.result.message, "navigation time budget exhausted")
        finally:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
            backend.destroy()
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
