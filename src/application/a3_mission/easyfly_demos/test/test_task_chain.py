"""Installed templates -> facade -> FSM/SDK -> real search/coordinator, simulated flight backends."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from easyfly_msgs.action import Takeoff, Land, NavigateTo, TrackTarget
from easyfly_msgs.msg import UAVState
from easyfly_msgs.srv import ManageMission, GetCapabilities


class TaskChainTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('d3_task_chain_plant')
        cls.group = ReentrantCallbackGroup()
        cls.calls, cls.processes, cls.logs = [], [], []
        cls.position, cls.armed = (0., 0., 0.), False
        cls.nav_behavior, cls.navigation_active = 'normal', False
        cls.temp = tempfile.TemporaryDirectory(prefix='easyfly-d3-chain-')
        cls.catalog = Path(cls.temp.name)
        source = Path(get_package_share_directory('easyfly_mission_scripts')) / 'missions'
        for path in source.glob('*.yaml'):
            shutil.copyfile(path, cls.catalog / path.name)
        cls.servers = []
        for action, topic in ((Takeoff, '/takeoff'), (Land, '/land'),
                              (NavigateTo, '/navigate_astar'), (TrackTarget, '/track_target')):
            def execute(handle, action=action, topic=topic):
                cls.calls.append(topic)
                if action is TrackTarget:
                    while not handle.is_cancel_requested and all(p.poll() is None for p in cls.processes):
                        time.sleep(.01)
                    if handle.is_cancel_requested:
                        handle.canceled()
                        return action.Result(success=False, message='real track cancel terminal')
                    handle.abort()
                    return action.Result(success=False, message='test shutting down')
                if action is Takeoff:
                    cls.armed = True
                    cls.position = (cls.position[0], cls.position[1], handle.request.target_altitude)
                elif action is Land:
                    cls.armed = False
                    cls.position = (cls.position[0], cls.position[1], 0.)
                else:
                    cls.navigation_active = True
                    if cls.nav_behavior == 'late_child':
                        time.sleep(.8)
                        cls.navigation_active = False
                        if handle.is_cancel_requested:
                            handle.canceled()
                            return action.Result(success=False, message='real private child stopped')
                    cls.position = (handle.request.target_x, handle.request.target_y, handle.request.target_z)
                time.sleep(.05)
                handle.succeed()
                if action is NavigateTo:
                    cls.navigation_active = False
                return action.Result(success=True, message='software backend success')
            cls.servers.append(ActionServer(cls.node, action, topic, execute,
                               cancel_callback=lambda _: CancelResponse.ACCEPT,
                               callback_group=cls.group))
        cls.publisher = cls.node.create_publisher(UAVState, '/uav_state', 10)
        cls.telemetry_generation = 0
        def telemetry():
            state = UAVState(connected=True, pose_valid=True, velocity_valid=True, battery_valid=True,
                             armed=cls.armed, flight_mode=3, battery_remaining=1.)
            state.header.frame_id = 'odom'
            state.header.stamp = cls.node.get_clock().now().to_msg()
            state.position_x, state.position_y, state.position_z = cls.position
            cls.publisher.publish(state)
            cls.telemetry_generation += 1
        cls.timer = cls.node.create_timer(.02, telemetry)
        cls.client = cls.node.create_client(ManageMission, '/platform/mission/manage')
        cls.capabilities = cls.node.create_client(GetCapabilities, '/platform/mission/capabilities')
        cls.executor = MultiThreadedExecutor(num_threads=6)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin)
        cls.thread.start()
        commands = [
            ['ros2', 'run', 'easyfly_global_planner', 'navigation_coordinator_node',
             '--ros-args', '-p', 'cancel_timeout:=0.1'],
            ['ros2', 'run', 'easyfly_mission_fsm', 'search_area_node'],
            ['ros2', 'run', 'easyfly_mission_fsm', 'mission_fsm_node',
             '--ros-args', '-p', 'cancel_budget_s:=0.1'],
            ['ros2', 'run', 'easyfly_mission_srvs', 'mission_services', '--ros-args',
             '-p', 'mission_directory:=' + str(cls.catalog)],
        ]
        for command in commands:
            log = tempfile.TemporaryFile(mode='w+')
            cls.logs.append(log)
            cls.processes.append(subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True))
        if not cls.client.wait_for_service(timeout_sec=10):
            raise RuntimeError('platform service unavailable')
        if not cls.capabilities.wait_for_service(timeout_sec=10):
            raise RuntimeError('platform capabilities unavailable')
        deadline = time.monotonic() + 10
        matched_generation = None
        while time.monotonic() < deadline:
            # Service discovery does not establish the coordinator, search and
            # mission telemetry readers. Publish distinct fresh samples only
            # after all three readers match before requesting the first goal.
            if cls.publisher.get_subscription_count() < 3:
                matched_generation = None
            elif matched_generation is None:
                matched_generation = cls.telemetry_generation
            future = cls.capabilities.call_async(GetCapabilities.Request())
            while not future.done() and time.monotonic() < deadline:
                time.sleep(.01)
            if (future.done() and all(future.result().ready)
                    and matched_generation is not None
                    and cls.telemetry_generation - matched_generation >= 2):
                break
            time.sleep(.02)
        else:
            raise RuntimeError('managed mission/search endpoints unavailable')

    @classmethod
    def tearDownClass(cls):
        for process in reversed(cls.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        cls.executor.shutdown(timeout_sec=5)
        cls.thread.join(timeout=5)
        for server in cls.servers:
            server.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.temp.cleanup()
        for log in cls.logs:
            log.close()

    def call(self, operation, name='', execution=''):
        future = self.client.call_async(ManageMission.Request(operation=operation, mission_name=name,
                                                             execution_id=execution))
        end = time.monotonic() + 8
        while not future.done() and time.monotonic() < end:
            time.sleep(.01)
        self.assertTrue(future.done())
        return future.result()

    def test_all_four_installed_templates_execute_through_platform_api(self):
        names = ('takeoff_hold_land_v1.yaml', 'navigation_return_v1.yaml',
                 'search_land_v1.yaml', 'timed_track_land_v1.yaml')
        self.assertEqual(set(self.call('list').missions), set(names))
        try:
            for name in names:
                self.calls.clear()
                self.assertTrue(self.call('validate', name).success)
                started = self.call('start', name)
                self.assertTrue(started.success, started.message)
                until = time.monotonic() + 20
                while time.monotonic() < until:
                    result = self.call('query', execution=started.execution_id)
                    if not result.execution_pending:
                        break
                    time.sleep(.02)
                self.assertFalse(result.execution_pending)
                self.assertEqual(result.status, 'succeeded', name + ': ' + result.message)
                self.assertEqual(self.calls[0], '/takeoff')
                self.assertEqual(self.calls[-1], '/land')
                if name == 'search_land_v1.yaml':
                    self.assertEqual(self.calls.count('/navigate_astar'), 4)
                if name == 'timed_track_land_v1.yaml':
                    self.assertIn('/track_target', self.calls)
        except Exception:
            for log in self.logs:
                log.seek(0)
                print(log.read())
            raise

    def test_installed_cpp_and_python_sdk_cancel_examples(self):
        for executable in ('sdk_mission_demo', 'sdk_mission_demo.py'):
            # With no --execute, the installed examples MUST NOT send any action.
            self.calls.clear()
            result = subprocess.run(['ros2', 'run', 'easyfly_demos', executable], capture_output=True,
                                    text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            self.assertFalse(self.calls)
            result = subprocess.run(['ros2', 'run', 'easyfly_demos', executable, '--execute',
                                     '--cancel-after', '0.8'], capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            self.assertIn('/takeoff', self.calls)
            self.assertNotIn('/land', self.calls)  # No automatic landing on cancellation.

    def test_nested_public_failure_retains_platform_lock_until_private_child_stops(self):
        path = self.catalog / 'nested_failure.yaml'
        path.write_text('schema_version: 1\nframe_id: odom\nunits: {position: m, angle: rad, time: s}\n'
                        'steps:\n- {type: navigate, x: 1, y: 0, z: 1.5, timeout: 5}\n')
        type(self).nav_behavior = 'late_child'
        try:
            started = self.call('start', path.name)
            self.assertTrue(started.success)
            until = time.monotonic() + 5
            while not self.navigation_active and time.monotonic() < until:
                time.sleep(.005)
            self.assertTrue(self.navigation_active)
            self.call('cancel', execution=started.execution_id)
            current = self.call('query', execution=started.execution_id)
            while current.status not in ('failed', 'canceled') and time.monotonic() < until:
                current = self.call('query', execution=started.execution_id)
                time.sleep(.005)
            self.assertTrue(current.execution_pending)
            self.assertTrue(self.navigation_active)
            self.assertEqual(self.call('start', path.name).status, 'busy')
            while current.execution_pending and time.monotonic() < until:
                current = self.call('query', execution=started.execution_id)
                time.sleep(.005)
            self.assertFalse(current.execution_pending)
            self.assertFalse(self.navigation_active)
        finally:
            type(self).nav_behavior = 'normal'


if __name__ == '__main__':
    unittest.main()
