# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_lidar_camera_fusion/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Deterministically keep the worker from taking queued work across a fault."""
import unittest

import rclpy
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from easyfly_lidar_camera_fusion.node import FusionNode


class Pending(unittest.TestCase):
    def test_invalid_cloud_clears_pending_and_calibration_fault_cannot_retag_it(
        self,
    ):
        rclpy.init()
        node = FusionNode()
        try:
            # Reentrant lock blocks the actual worker until all transitions complete.
            with node.condition:
                cloud = PointCloud2()
                cloud.header.frame_id = 'odom'
                cloud.header.stamp = node.get_clock().now().to_msg()
                node.cloud(cloud)
                self.assertIsNotNone(node.pending)
                self.assertEqual(node.pending[2], node.epoch)
                node.cloud(PointCloud2())
                self.assertIsNone(node.pending)
                self.assertIsNone(node.ready_pair)

                cloud.header.stamp = node.get_clock().now().to_msg()
                node.cloud(cloud)
                over_budget = PointCloud2(height=1, width=node.max_points + 1)
                over_budget.header.frame_id = 'odom'
                over_budget.header.stamp = node.get_clock().now().to_msg()
                node.cloud(over_budget)
                self.assertIsNone(node.pending)
                self.assertEqual(node.reason, 'cloud budget')

                cloud.header.stamp = node.get_clock().now().to_msg()
                node.cloud(cloud)
                submitted_epoch = node.pending[2]
                node.camera(CameraInfo())
                self.assertLess(submitted_epoch, node.epoch)
                self.assertEqual(node.pending[2], submitted_epoch)
                node.image(Image())
                self.assertEqual(node.pending[2], submitted_epoch)
                self.assertLess(submitted_epoch, node.epoch)
                node.pending = None
            self.assertFalse(node.ready())
        finally:
            node.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
