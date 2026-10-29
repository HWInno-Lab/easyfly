#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Transparent native RGB-D admission after static fixture setup; no retiming."""
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

rclpy.init()
node=rclpy.create_node('orb_static_fixture_rgbd_relay')
qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE)
publishers=[]
for source,target in [('/camera/color/image_raw','/fixture/rgb'),
                      ('/camera/aligned_depth_to_color/depth/image_raw','/fixture/depth')]:
    publisher=node.create_publisher(Image,target,qos);publishers.append(publisher)
    node.create_subscription(Image,source,lambda message,p=publisher:p.publish(message),qos)
print('transparent native RGB-D relay ready',flush=True)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
