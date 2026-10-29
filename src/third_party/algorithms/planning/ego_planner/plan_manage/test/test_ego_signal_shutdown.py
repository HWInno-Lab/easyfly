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

"""Real EGO startup/ready shutdown, active/frozen clocks, without flight goals."""
import ast
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rosgraph_msgs.msg import Clock

EXECUTABLE = (
    sys.argv.pop(1)
    if len(sys.argv) > 1
    else str(Path(get_package_prefix("ego_planner")) / "lib/ego_planner/ego_planner_node")
)


class EgoShutdownTest(unittest.TestCase):
    def test_main_thread_retirement(self):
        launch = (
            Path(get_package_share_directory("easyfly_local_planner"))
            / "launch/local_planner.launch.py"
        )
        tree = ast.parse(launch.read_text())
        assignment = next(
            n
            for n in ast.walk(tree)
            if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == "ego_params" for t in n.targets)
        )
        context = LaunchContext()
        for declaration in ast.walk(tree):
            if (
                isinstance(declaration, ast.Call)
                and isinstance(declaration.func, ast.Name)
                and declaration.func.id == "DeclareLaunchArgument"
            ):
                default = next(
                    (k.value for k in declaration.keywords if k.arg == "default_value"), None
                )
                if default is not None and isinstance(default, ast.Constant):
                    context.launch_configurations[
                        ast.literal_eval(declaration.args[0])
                    ] = ast.literal_eval(default)
        parameters = eval(
            compile(ast.Expression(assignment.value), str(launch), "eval"),
            {"__builtins__": {}},
            dict(
                use_sim_time=True,
                flight_type=1,
                managed=True,
                virtual_ceil_height=3.2,
                float=float,
                LaunchConfiguration=LaunchConfiguration,
                ParameterValue=lambda value, value_type: value_type(
                    value.perform(context) if hasattr(value, "perform") else value
                ),
            ),
        )
        os.environ["ROS_DOMAIN_ID"] = "196"
        rclpy.init()
        node = rclpy.create_node("ego_signal_regression")
        clock = node.create_publisher(Clock, "/clock", 10)
        try:
            with tempfile.TemporaryDirectory(prefix="ego-signal-test-") as directory:
                params = Path(directory) / "params.yaml"
                params.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
                for phase in ("initializing", "ready", "preset_wait"):
                    case_parameters = dict(parameters)
                    if phase == "preset_wait":
                        case_parameters.update(
                            {
                                "navigation_managed": False,
                                "fsm/flight_type": 2,
                                "fsm/realworld_experiment": True,
                            }
                        )
                    params.write_text(
                        yaml.safe_dump({"/**": {"ros__parameters": case_parameters}})
                    )
                    for active in (False, True):
                        for sig in (signal.SIGINT, signal.SIGTERM):
                            with self.subTest(phase=phase, active=active, signal=sig.name):
                                with tempfile.TemporaryFile(mode="w+t") as output:
                                    process = subprocess.Popen(
                                        [EXECUTABLE, "--ros-args", "--params-file", str(params)],
                                        stdout=output,
                                        stderr=output,
                                    )
                                    started = time.monotonic()

                                    def publish():
                                        msg = Clock()
                                        stamp = int(
                                            (10 + (time.monotonic() - started if active else 0))
                                            * 1e9
                                        )
                                        msg.clock.sec, msg.clock.nanosec = divmod(stamp, 10 ** 9)
                                        clock.publish(msg)
                                        rclpy.spin_once(node, timeout_sec=0.01)

                                    try:
                                        marker = (
                                            "GridMap trusted cloud"
                                            if phase == "initializing"
                                            else "EGO runtime ready"
                                        )
                                        if phase == "preset_wait":
                                            marker = "Waiting for trigger"
                                        deadline = time.monotonic() + 8
                                        while time.monotonic() < deadline:
                                            publish()
                                            output.seek(0)
                                            log = output.read()
                                            self.assertIsNone(process.poll(), log)
                                            if marker in log:
                                                break
                                        else:
                                            self.fail("native initialization not observed: " + log)
                                        process.send_signal(sig)
                                        deadline = time.monotonic() + 5
                                        while (
                                            process.poll() is None and time.monotonic() < deadline
                                        ):
                                            publish()
                                        output.seek(0)
                                        log = output.read()
                                        self.assertEqual(process.poll(), 0, log)
                                        self.assertNotIn("terminate called", log)
                                        self.assertNotIn("context is invalid", log)
                                    finally:
                                        if process.poll() is None:
                                            process.kill()
                                            process.wait(timeout=3)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    unittest.main()
