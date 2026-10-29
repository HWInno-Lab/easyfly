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

"""Typed ROS parameter services and legacy JSON compatibility."""
import json
import os
import signal
import subprocess
import sys
import time
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.context import Context
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from easyfly_msgs.srv import QueryParams
from std_srvs.srv import Trigger
from easyfly_params.runtime import ParameterManager
from easyfly_params import runtime


class RuntimeLifecycleTest(unittest.TestCase):
    def test_signals_finish_spin_before_cleanup(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=signum):
                events = []
                context, node, executor = MagicMock(), MagicMock(), MagicMock()
                context.ok.return_value = True

                def take():
                    events.append('take_started')
                    signal.raise_signal(signum)
                    self.assertTrue(context.ok())
                    context.try_shutdown.assert_not_called()
                    events.append('take_completed')

                executor.spin_once.side_effect = lambda **_: take()
                executor.shutdown.side_effect = lambda: events.append('executor_stopped')
                node.destroy_node.side_effect = lambda: events.append('node_destroyed')
                context.try_shutdown.side_effect = lambda: events.append('context_shutdown')
                previous = signal.getsignal(signum)
                with patch.object(runtime, 'Context', return_value=context), \
                        patch.object(runtime.rclpy, 'init') as init, \
                        patch.object(runtime, 'ParameterManager', return_value=node), \
                        patch.object(runtime, 'SingleThreadedExecutor', return_value=executor):
                    runtime.main()
                init.assert_called_once_with(
                    context=context, signal_handler_options=runtime.SignalHandlerOptions.NO)
                executor.spin_once.assert_called_once_with(timeout_sec=0.1)
                self.assertEqual(events, ['take_started', 'take_completed', 'executor_stopped',
                                          'node_destroyed', 'context_shutdown'])
                self.assertEqual(signal.getsignal(signum), previous)

    def test_real_errors_propagate_but_context_is_cleaned(self):
        for constructing in (True, False):
            context, node, executor = MagicMock(), MagicMock(), MagicMock()
            context.ok.return_value = True
            executor.spin_once.side_effect = RuntimeError('unexpected')
            error = ValueError('bad profile') if constructing else None
            with patch.object(runtime, 'Context', return_value=context), \
                    patch.object(runtime.rclpy, 'init'), \
                    patch.object(runtime, 'ParameterManager', return_value=node,
                                 side_effect=error), \
                    patch.object(runtime, 'SingleThreadedExecutor', return_value=executor):
                with self.assertRaisesRegex(ValueError if constructing else RuntimeError,
                                            'bad profile' if constructing else 'unexpected'):
                    runtime.main()
            context.try_shutdown.assert_called_once_with()
            if not constructing:
                executor.shutdown.assert_called_once_with()
                node.destroy_node.assert_called_once_with()

    def test_real_node_sigint_and_sigterm_exit_zero(self):
        # Dedicated domain: no platform/flight nodes and no parameter writes.
        context = Context()
        context.init(domain_id=126)
        observer = rclpy.create_node('parameter_shutdown_test', context=context)
        client = observer.create_client(QueryParams, '/platform/params/query')
        clock = observer.create_publisher(Clock, '/clock', 10)
        environment = dict(os.environ, ROS_DOMAIN_ID='126', ROS_LOCALHOST_ONLY='1')
        try:
            for signum in (signal.SIGINT, signal.SIGTERM) * 4:
                with self.subTest(signal=signum):
                    end = time.monotonic() + 5
                    while client.service_is_ready() and time.monotonic() < end:
                        time.sleep(0.05)
                    self.assertFalse(client.service_is_ready())
                    child = subprocess.Popen(
                        [sys.executable, '-c', 'from easyfly_params.runtime import main; main()',
                         '--ros-args', '-p', 'use_sim_time:=true'],
                        env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    )
                    try:
                        self.assertTrue(client.wait_for_service(timeout_sec=5))
                        end = time.monotonic() + 5
                        while clock.get_subscription_count() == 0 and time.monotonic() < end:
                            time.sleep(0.01)
                        self.assertGreater(clock.get_subscription_count(), 0)
                        for sample in range(100):
                            message = Clock()
                            message.clock.sec = 1
                            message.clock.nanosec = sample * 1000000
                            clock.publish(message)
                            time.sleep(0.001)
                        child.send_signal(signum)
                        stdout, stderr = child.communicate(timeout=5)
                        self.assertEqual(child.returncode, 0, (stdout + stderr).decode())
                        self.assertNotIn(b'Traceback', stderr)
                        self.assertNotIn(b'already called', stderr)
                    finally:
                        if child.poll() is None:
                            child.kill()
                            child.communicate(timeout=3)
        finally:
            observer.destroy_node()
            rclpy.try_shutdown(context=context)


class RuntimeTest(unittest.TestCase):
    def test_local_typed_parameters_are_atomic_and_exported(self):
        rclpy.init()
        manager = ParameterManager()
        node = rclpy.create_node('parameter_test')
        executor = SingleThreadedExecutor()
        executor.add_node(manager)
        executor.add_node(node)
        query = node.create_client(QueryParams, '/platform/params/query')
        typed = node.create_client(
            SetParametersAtomically,
            '/easyfly_params/set_parameters_atomically',
        )
        export = node.create_client(Trigger, '/platform/params/export')

        def call(client, request):
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            future = client.call_async(request)
            end = time.monotonic() + 3
            while not future.done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(future.done())
            return future.result()

        try:
            result = call(
                query, QueryParams.Request(param_name='operator_note')
            )
            self.assertTrue(result.success)
            self.assertEqual(json.loads(result.current_value), '')
            result = call(
                query,
                QueryParams.Request(
                    param_name='operator_note', param_value='"hello"'
                ),
            )
            self.assertTrue(result.success)
            result = call(
                query,
                QueryParams.Request(
                    param_name='operator_note', param_value='""'
                ),
            )
            self.assertTrue(
                result.success
            )  # JSON empty string writes; empty request reads.
            result = call(
                query,
                QueryParams.Request(
                    param_name='export_pretty', param_value='true'
                ),
            )
            self.assertTrue(result.success)
            self.assertFalse(
                call(
                    query,
                    QueryParams.Request(
                        param_name='export_pretty', param_value='1'
                    ),
                ).success
            )
            self.assertTrue(manager.get_parameter('export_pretty').value)
            for value in ('null', '{}', 'not-json', 'NaN'):
                self.assertFalse(
                    call(
                        query,
                        QueryParams.Request(
                            param_name='operator_note', param_value=value
                        ),
                    ).success
                )
            for name in (
                'planning_floor',
                'profile',
                'unknown',
                '/flight_control/max_altitude',
            ):
                self.assertFalse(
                    call(
                        query,
                        QueryParams.Request(param_name=name, param_value='2'),
                    ).success
                )
            result = call(
                typed,
                SetParametersAtomically.Request(
                    parameters=[
                        Parameter(
                            'operator_note', value='must not commit'
                        ).to_parameter_msg(),
                        Parameter('export_pretty', value=1).to_parameter_msg(),
                    ]
                ),
            )
            self.assertFalse(result.result.successful)
            self.assertEqual(manager.get_parameter('operator_note').value, '')
            self.assertTrue(manager.get_parameter('export_pretty').value)
            result = call(export, Trigger.Request())
            self.assertTrue(result.success)
            effective = json.loads(result.message)
            self.assertEqual(effective['schema_version'], 1)
            self.assertEqual(
                effective['launch_arguments']['navigation_strategy'],
                'combined',
            )
            self.assertEqual(effective['contracts']['planning_ceiling'], 3.2)
        finally:
            executor.shutdown()
            node.destroy_node()
            manager.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
