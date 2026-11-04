# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from easyfly_msgs.msg import (
    ObstacleInfoArray,
    Target3DArray,
    ClassifiedObstacleArray,
)
from easyfly_msgs.srv import GetCapabilities
from test_classification import fixtures


class ClassificationROS(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('classification_test')
        cls.output = None
        cls.node.create_subscription(
            ClassifiedObstacleArray,
            '/obstacles_classified',
            lambda m: setattr(cls, 'output', m),
            10,
        )
        cls.geometry = cls.node.create_publisher(
            ObstacleInfoArray,
            '/obstacles',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        cls.targets = cls.node.create_publisher(
            Target3DArray, '/target_positions', 10
        )
        cls.cap = cls.node.create_client(
            GetCapabilities, '/platform/classification/capabilities'
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
                'easyfly_semantic_map',
                'obstacle_classifier',
                '--ros-args',
                '-p',
                'use_sim_time:=false',
            ],
            stdout=cls.log,
            stderr=cls.log,
            start_new_session=True,
        )
        cls.addClassCleanup(cls.cleanup)
        if not cls.cap.wait_for_service(timeout_sec=8):
            cls.log.seek(0)
            raise RuntimeError(cls.log.read())

    @classmethod
    def cleanup(cls):
        if cls.process.poll() is None:
            os.killpg(cls.process.pid, signal.SIGINT)
        try:
            cls.process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(cls.process.pid, signal.SIGKILL)
            cls.process.wait(timeout=2)
        cls.executor.shutdown()
        cls.thread.join()
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.log.close()

    def until(self, predicate, pulse=None):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            if pulse:
                pulse()
            if predicate():
                return
            time.sleep(0.07)
        self.log.seek(0)
        self.fail(self.log.read() + str(self.output))

    def send(self, empty=False, invalid=False):
        geometry, targets = fixtures()
        header = geometry.header
        header.stamp = self.node.get_clock().now().to_msg()
        geometry.obstacles[0].header = header
        targets.header = targets.targets[0].header = header
        if empty:
            targets.targets.clear()
        elif invalid:
            targets.targets[0].position_x = float('nan')
        self.targets.publish(targets)
        self.geometry.publish(geometry)

    def test_classification_quality_and_lost_target_preserve_geometry(self):
        self.until(
            lambda: self.output
            and self.output.obstacles
            and self.output.obstacles[0].association_valid,
            pulse=self.send,
        )
        item = self.output.obstacles[0]
        self.assertEqual(item.obstacle.obstacle_type, 'person')
        self.assertEqual(item.target_id, 9)
        self.assertTrue(item.obstacle.is_dynamic)
        self.assertEqual(item.obstacle.velocity_x, 1.0)
        self.until(
            lambda: self.output
            and self.output.geometry_valid
            and self.output.obstacles
            and not self.output.obstacles[0].association_valid,
            pulse=lambda: self.send(empty=True),
        )
        self.assertEqual(self.output.obstacles[0].obstacle.obstacle_id, 7)
        self.assertEqual(self.output.obstacles[0].obstacle.size_x, 2.0)
        self.assertEqual(
            self.output.obstacles[0].obstacle.obstacle_type, 'unknown'
        )

    def test_invalid_target_and_stale_geometry_fail_without_map_mutation(self):
        self.until(
            lambda: self.output and 'invalid target' in self.output.reason,
            pulse=lambda: self.send(invalid=True),
        )
        self.assertTrue(self.output.geometry_valid)
        self.assertEqual(len(self.output.obstacles), 1)
        time.sleep(0.65)
        self.until(lambda: self.output and not self.output.geometry_valid)
        future = self.cap.call_async(GetCapabilities.Request())
        self.until(future.done)
        self.assertFalse(future.result().ready[0])
        self.assertEqual(self.node.count_publishers('/obstacles'), 1)
        self.assertEqual(self.node.count_publishers('/occupancy_grid'), 0)


if __name__ == '__main__':
    unittest.main()
