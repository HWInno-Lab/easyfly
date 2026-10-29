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

"""Real-process signal regression; a forced kill or nonzero exit is a failure."""
import os
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
import rclpy
from rosgraph_msgs.msg import Clock


class ShutdownTest(unittest.TestCase):
    def test_signal_exit(self):
        os.environ["ROS_DOMAIN_ID"] = "193"
        rclpy.init()
        node = rclpy.create_node("shutdown_regression")
        clocks = node.create_publisher(Clock, "/clock", 10)
        executable = os.path.join(
            get_package_prefix("ego_planner"),
            "lib",
            "ego_planner",
            "traj_server",
        )
        try:
            for mode in ("wall", "paused", "active"):
                for sig in (signal.SIGINT, signal.SIGTERM):
                    with self.subTest(mode=mode, signal=sig.name):
                        name = f"shutdown_{mode}_{sig.name.lower()}"
                        with tempfile.TemporaryFile(mode="w+t") as output:
                            process = subprocess.Popen(
                                [
                                    executable,
                                    "--ros-args",
                                    "-r",
                                    f"__node:={name}",
                                    "-p",
                                    f'use_sim_time:={str(mode != "wall").lower()}',
                                ],
                                stdout=output,
                                stderr=output,
                            )
                            try:
                                ready = False
                                deadline = time.monotonic() + 8.0
                                tick = 0
                                while time.monotonic() < deadline:
                                    if mode != "wall":
                                        clocks.publish(
                                            Clock(
                                                clock=Time(
                                                    sec=10,
                                                    nanosec=tick * 1000000,
                                                )
                                            )
                                        )
                                        if mode == "active":
                                            tick = (tick + 1) % 1000
                                    rclpy.spin_once(node, timeout_sec=0.02)
                                    output.seek(0)
                                    logs = output.read()
                                    self.assertIsNone(process.poll(), logs)
                                    ready = "[Traj server]: ready." in logs
                                    if ready:
                                        break
                                self.assertTrue(ready, "node did not become ready: " + logs)
                                # Include positive frozen time, not only zero-clock startup.
                                if mode != "wall":
                                    deadline = time.monotonic() + 2.0
                                    while clocks.get_subscription_count() == 0:
                                        self.assertLess(time.monotonic(), deadline)
                                        rclpy.spin_once(node, timeout_sec=0.02)
                                    for _ in range(5):
                                        clocks.publish(
                                            Clock(
                                                clock=Time(
                                                    sec=10,
                                                    nanosec=(
                                                        tick * 1000000 if mode == "active" else 0
                                                    ),
                                                )
                                            )
                                        )
                                        if mode == "active":
                                            tick += 1
                                        rclpy.spin_once(node, timeout_sec=0.02)
                                process.send_signal(sig)
                                try:
                                    code = process.wait(timeout=3.0)
                                except subprocess.TimeoutExpired:
                                    self.fail("signal shutdown required forced termination")
                                output.seek(0)
                                self.assertEqual(code, 0, output.read())
                            finally:
                                if process.poll() is None:
                                    process.kill()
                                    process.wait(timeout=3.0)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
