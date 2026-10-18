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

"""Actual prediction node, real ROS input/output, and explicit simulated clock."""
import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import rclpy
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from easyfly_msgs.msg import Target3D, Target3DArray, TargetPredictionArray
from easyfly_msgs.srv import GetCapabilities


class PredictionRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('prediction_ros_test')
        cls.clock_pub = cls.node.create_publisher(Clock, '/clock', 10)
        cls.publisher = cls.node.create_publisher(
            Target3DArray, '/target_positions', 10
        )
        cls.outputs = []
        cls.subscriber = cls.node.create_subscription(
            TargetPredictionArray,
            '/target_predictions',
            lambda message: cls.outputs.append(message),
            10,
        )
        cls.client = cls.node.create_client(
            GetCapabilities, '/platform/prediction/capabilities'
        )
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin)
        cls.thread.start()
        cls.log = tempfile.TemporaryFile(mode='w+')
        cls.process = subprocess.Popen(
            [
                'ros2',
                'run',
                'easyfly_target_prediction',
                'target_prediction_node',
                '--ros-args',
                '-p',
                'use_sim_time:=true',
                '-p',
                'max_tracks:=2',
            ],
            stdout=cls.log,
            stderr=cls.log,
            start_new_session=True,
        )
        if not cls.client.wait_for_service(timeout_sec=8):
            cls.log.seek(0)
            raise RuntimeError(cls.log.read())
        cls.ros_time = 100.0
        end = time.monotonic() + 4
        while (
            cls.publisher.get_subscription_count() == 0
            and time.monotonic() < end
        ):
            time.sleep(0.02)

    @classmethod
    def tearDownClass(cls):
        if cls.process.poll() is None:
            os.killpg(cls.process.pid, signal.SIGINT)
        try:
            code = cls.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(cls.process.pid, signal.SIGKILL)
            cls.process.wait(timeout=5)
            code = -9
        cls.executor.shutdown(timeout_sec=5)
        cls.thread.join(timeout=5)
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.log.seek(0)
        diagnostics = cls.log.read()
        cls.log.close()
        if code != 0 or cls.thread.is_alive():
            raise RuntimeError(
                f'prediction node/executor did not stop cleanly: {code}\n{diagnostics}')

    def until(self, predicate, timeout=3):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            time.sleep(0.005)
        if not predicate():
            self.log.seek(0)
            print(self.log.read())
        self.assertTrue(predicate())

    @staticmethod
    def stamp(seconds):
        ns = round(seconds * 1e9)
        return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)

    def advance(self, seconds):
        type(self).ros_time = seconds
        for _ in range(3):
            self.clock_pub.publish(Clock(clock=self.stamp(seconds)))
            time.sleep(0.01)

    def message(self, ids=(1,), x=0.0, stamp=None):
        message = Target3DArray()
        message.header.frame_id = 'odom'
        message.header.stamp = self.stamp(
            self.ros_time if stamp is None else stamp
        )
        for identity in ids:
            target = Target3D(
                track_id=identity,
                class_name='person',
                confidence=0.9,
                position_x=x,
                position_valid=True,
            )
            target.header.frame_id = message.header.frame_id
            target.header.stamp = self.stamp(
                self.ros_time if stamp is None else stamp
            )
            message.targets.append(target)
        return message

    def send(self, message):
        self.outputs.clear()
        self.publisher.publish(message)
        self.until(lambda: bool(self.outputs))
        time.sleep(0.025)

    def ready_pair(self, identity=1):
        self.advance(self.ros_time + 0.1)
        self.send(self.message((identity,), 0.0))
        self.assertFalse(
            any(p.valid for m in self.outputs for p in m.predictions)
        )
        self.advance(self.ros_time + 0.1)
        self.send(self.message((identity,), 0.1))
        self.until(
            lambda: any(p.valid for m in self.outputs for p in m.predictions)
        )
        return next(
            p for m in reversed(self.outputs) for p in m.predictions if p.valid
        )

    def capability(self):
        future = self.client.call_async(GetCapabilities.Request())
        self.until(future.done)
        return future.result()

    def setUp(self):
        self.advance(self.ros_time + 0.2)
        self.send(self.message(()))

    def test_constant_velocity_metadata_and_readonly_parameters(self):
        prediction = self.ready_pair()
        self.assertEqual(prediction.header.frame_id, 'odom')
        self.assertEqual(
            prediction.observation_header.stamp, self.stamp(self.ros_time)
        )
        self.assertAlmostEqual(prediction.velocity_x, 1.0, places=5)
        self.assertAlmostEqual(prediction.position_x, 0.6, places=5)
        self.assertAlmostEqual(prediction.horizon_seconds, 0.5)
        self.assertGreaterEqual(prediction.variance_x, 0.04)
        cap = self.capability()
        self.assertEqual(cap.implemented, [True])
        self.assertEqual(cap.ready, [True])
        self.assertIn('not flight permission', cap.details[0])
        service = self.node.create_client(
            SetParameters, '/target_prediction/set_parameters'
        )
        try:
            self.assertTrue(service.wait_for_service(timeout_sec=3))
            future = service.call_async(
                SetParameters.Request(
                    parameters=[
                        Parameter(
                            name='horizon_seconds',
                            value=ParameterValue(type=3, double_value=1.0),
                        )
                    ]
                )
            )
            self.until(future.done)
            self.assertFalse(future.result().results[0].successful)
        finally:
            self.node.destroy_client(service)

    def test_invalid_metadata_and_values_fail_closed(self):
        for kind in (
            'frame',
            'stamp',
            'entry_stamp',
            'future',
            'stale',
            'duplicate_id',
            'negative_id',
            'nan',
            'confidence',
            'invalid_position',
            'oversized',
        ):
            self.ready_pair()
            self.advance(self.ros_time + 0.1)
            message = self.message(x=0.2)
            if kind == 'frame':
                message.header.frame_id = 'map'
            elif kind == 'stamp':
                message.header.stamp = Time()
            elif kind == 'entry_stamp':
                message.targets[0].header.stamp = self.stamp(
                    self.ros_time - 0.01
                )
            elif kind == 'future':
                message = self.message(x=0.2, stamp=self.ros_time + 0.1)
            elif kind == 'stale':
                message = self.message(x=0.2, stamp=self.ros_time - 1.0)
            elif kind == 'duplicate_id':
                message = self.message((1, 1), 0.2)
            elif kind == 'negative_id':
                message.targets[0].track_id = -1
            elif kind == 'nan':
                message.targets[0].position_x = float('nan')
            elif kind == 'confidence':
                message.targets[0].confidence = 1.1
            elif kind == 'invalid_position':
                message.targets[0].position_valid = False
            else:
                message = self.message((1, 2, 3), 0.2)
            self.send(message)
            self.until(lambda: not self.outputs[-1].predictions)
            self.assertFalse(self.capability().ready[0], kind)

    def test_empty_identity_loss_reacquisition_and_jump(self):
        self.ready_pair()
        self.advance(self.ros_time + 0.1)
        self.send(self.message(()))
        self.assertFalse(self.outputs[-1].predictions)
        self.advance(self.ros_time + 0.1)
        self.send(self.message(x=0.2))
        self.assertFalse(self.outputs[-1].predictions[0].valid)
        self.advance(self.ros_time + 0.1)
        self.send(self.message(x=3.0))
        self.assertFalse(self.outputs[-1].predictions[0].valid)
        self.assertEqual(
            self.outputs[-1].predictions[0].reason, 'motion_discontinuity'
        )
        self.advance(self.ros_time + 0.1)
        self.send(self.message(x=3.1))
        self.assertTrue(self.outputs[-1].predictions[0].valid)
        self.advance(self.ros_time + 0.1)
        self.send(self.message((2,), x=3.2))
        self.assertEqual(
            [p.track_id for p in self.outputs[-1].predictions], [2]
        )
        self.assertFalse(self.outputs[-1].predictions[0].valid)

    def test_duplicate_and_out_of_order_do_not_refresh_cache(self):
        self.ready_pair()
        self.send(self.message(x=0.1))
        self.until(lambda: not self.outputs[-1].predictions)
        self.send(self.message(x=0.0, stamp=self.ros_time - 0.1))
        self.assertFalse(self.outputs[-1].predictions)

    def test_paused_clock_and_rollback_clear_valid_predictions(self):
        self.ready_pair()
        self.outputs.clear()
        # /clock remains paused even when an old observation is re-received.
        time.sleep(0.65)
        self.until(
            lambda: bool(self.outputs) and not self.outputs[-1].predictions
        )
        self.assertFalse(self.capability().ready[0])
        self.ready_pair()
        self.advance(self.ros_time - 10.0)
        self.until(
            lambda: bool(self.outputs) and not self.outputs[-1].predictions
        )
        self.advance(self.ros_time + 0.1)
        self.send(self.message())
        self.assertFalse(self.outputs[-1].predictions[0].valid)


if __name__ == '__main__':
    unittest.main()
