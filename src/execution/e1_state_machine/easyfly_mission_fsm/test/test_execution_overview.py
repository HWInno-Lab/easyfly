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
"""Native FSM publishes exact admitted ground-wait context; no flight is started."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from easyfly_msgs.action import (
    ExecuteMission,
    Explore,
    GoToWaypoint,
    Land,
    NavigateTo,
    PrecisionLand,
    SearchArea,
    Takeoff,
    TrackTarget,
)
from easyfly_msgs.msg import UAVState
from easyfly_msgs.srv import SetFlightMode


class ExecutionOverviewTest(unittest.TestCase):
    def test_native_hover_context_success_cancel_and_failure(self):
        rclpy.init()
        node = rclpy.create_node('execution_overview_fixture')
        temporary = tempfile.TemporaryDirectory(prefix='easyfly-execution-overview-')
        log = tempfile.TemporaryFile(mode='w+')
        messages, unexpected = [], []
        battery_valid = [True]
        qos = QoSProfile(
            depth=64,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription = node.create_subscription(
            String,
            '/mission/execution_overview',
            lambda message: messages.append(json.loads(message.data)),
            qos,
        )
        publisher = node.create_publisher(UAVState, '/uav_state', 10)

        def telemetry():
            state = UAVState(
                connected=True,
                pose_valid=True,
                battery_valid=battery_valid[0],
                battery_remaining=1.0,
                flight_mode=3,
                armed=False,
                position_z=0.0,
            )
            state.header.frame_id = 'odom'
            state.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(state)

        timer = node.create_timer(0.02, telemetry)
        servers = []

        def create_server(action, name):
            def execute(handle):
                unexpected.append(name)
                handle.abort()
                result = action.Result(success=False)
                setattr(result, 'reason' if action is Explore else 'message', 'unexpected flight')
                return result

            return ActionServer(node, action, name, execute)

        for action, name in (
            (Takeoff, '/takeoff'),
            (Land, '/land'),
            (GoToWaypoint, '/goto_waypoint'),
            (NavigateTo, '/navigate'),
            (TrackTarget, '/track_target'),
            (SearchArea, '/search_area'),
            (PrecisionLand, '/precision_land'),
            (Explore, '/explore'),
        ):
            servers.append(create_server(action, name))

        def mode(request, response):
            unexpected.append('set_flight_mode')
            response.success = False
            return response

        service = node.create_service(SetFlightMode, '/set_flight_mode', mode)
        client = ActionClient(node, ExecuteMission, '/execute_mission')
        process = subprocess.Popen(
            [
                'ros2',
                'run',
                'easyfly_mission_fsm',
                'mission_fsm_node',
                '--ros-args',
                '-p',
                'battery_invalid_grace_s:=0.1',
            ],
            stdout=log,
            stderr=log,
            start_new_session=True,
        )

        def until(predicate, timeout=6):
            end = time.monotonic() + timeout
            while not predicate() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertTrue(predicate(), 'condition not observed before timeout')

        def wait(future):
            until(future.done)
            return future.result()

        def settle(duration=0.3):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)

        def run(name, durations, stop=None):
            content = (
                'schema_version: 1\r\nframe_id: odom\r\n'
                'units: {position: m, angle: rad, time: s}\r\nsteps:\r\n'
                + ''.join(f'  - {{type: hover, duration: {d}, timeout: 8}}\r\n' for d in durations)
            ).encode()
            path = Path(temporary.name) / name
            path.write_bytes(content)
            handle = wait(client.send_goal_async(ExecuteMission.Goal(mission_file=str(path))))
            self.assertTrue(handle.accepted)
            identity = bytes(handle.goal_id.uuid).hex()

            def seen():
                return [m for m in messages if m['goal_id'] == identity]

            until(lambda: any(m['state'] == 'active' for m in seen()))
            if stop == 'cancel':
                self.assertTrue(wait(handle.cancel_goal_async()).goals_canceling)
            elif stop == 'fail':
                battery_valid[0] = False
            result = wait(handle.get_result_async())
            terminal = 'canceled' if stop == 'cancel' else 'failed' if stop else 'succeeded'
            until(lambda: any(m['state'] == terminal for m in seen()))
            current = seen()
            self.assertEqual(result.status, 5 if stop == 'cancel' else 6 if stop else 4)
            self.assertEqual(result.result.success, stop is None)
            previous_completed = 0
            for message in current:
                self.assertEqual(
                    set(message),
                    {
                        'schema_version',
                        'stamp_ns',
                        'goal_id',
                        'name',
                        'sha256',
                        'content',
                        'frame_id',
                        'state',
                        'current_step',
                        'completed_steps',
                        'total_steps',
                    },
                )
                self.assertEqual(message['schema_version'], 1)
                self.assertGreater(message['stamp_ns'], 0)
                self.assertRegex(message['goal_id'], r'^[0-9a-f]{32}$')
                self.assertEqual(message['name'], 'mission' if '\\' in name else name)
                self.assertEqual(message['content'].encode(), content)
                self.assertEqual(message['sha256'], hashlib.sha256(content).hexdigest())
                self.assertEqual(message['frame_id'], 'odom')
                self.assertEqual(message['total_steps'], len(durations))
                self.assertGreaterEqual(message['current_step'], -1)
                self.assertLess(message['current_step'], len(durations))
                self.assertGreaterEqual(message['completed_steps'], previous_completed)
                self.assertLessEqual(message['completed_steps'], len(durations))
                previous_completed = message['completed_steps']
            self.assertEqual(current[-1]['state'], terminal)
            self.assertEqual(current[-1]['current_step'], -1)
            self.assertEqual(current[-1]['completed_steps'], 0 if stop else len(durations))
            self.assertFalse(unexpected)
            battery_valid[0] = True
            settle()
            return current

        try:
            self.assertTrue(client.wait_for_server(timeout_sec=8))
            settle()
            success = run('native-hover.yaml', [0.7, 0.7])
            self.assertTrue(
                any(
                    m['state'] == 'active' and m['current_step'] == 1 and m['completed_steps'] == 1
                    for m in success
                )
            )
            retained = []
            late = node.create_subscription(
                String, '/mission/execution_overview',
                lambda message: retained.append(json.loads(message.data)), qos)
            until(lambda: bool(retained))
            self.assertEqual(retained[-1], success[-1])
            node.destroy_subscription(late)
            run('odd\\hover.yaml', [0.15])
            run('cancel-hover.yaml', [5], 'cancel')
            run('failed-hover.yaml', [5], 'fail')
            self.assertFalse(unexpected)
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            client.destroy()
            for server in servers:
                server.destroy()
            node.destroy_service(service)
            node.destroy_subscription(subscription)
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()
            temporary.cleanup()
            log.close()


if __name__ == '__main__':
    unittest.main()
