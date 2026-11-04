# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_lidar_camera_fusion/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Bounded independent projection using exact two-source TF times."""
from collections import deque
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock, ClockType
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from easyfly_msgs.srv import GetCapabilities
from .projection import camera_model, image_layout, cloud_points, project


def stamp_ns(header):
    t = header.stamp
    return (
        t.sec * 1000000000 + t.nanosec
        if t.sec >= 0 and t.nanosec < 1000000000
        else 0
    )


class FusionNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion')
        frozen = ParameterDescriptor(read_only=True)
        for name, default in (
            ('max_age', 0.5),
            ('tolerance', 0.05),
            ('max_points', 20000),
            ('camera_frame', 'camera_color_optical_frame'),
            ('fixed_frame', 'odom'),
            ('image_topic', '/camera/color/image_raw'),
            ('cloud_topic', '/pointcloud_filtered'),
            ('camera_info_topic', '/camera/color/camera_info'),
        ):
            self.declare_parameter(name, default, frozen)
        self.age = self.get_parameter('max_age').value
        self.tolerance = self.get_parameter('tolerance').value
        self.max_points = self.get_parameter('max_points').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        if (
            not math.isfinite(self.age)
            or not 0.05 <= self.age <= 10
            or not math.isfinite(self.tolerance)
            or not 0 <= self.tolerance <= min(0.5, self.age)
        ):
            raise ValueError('invalid fusion time budgets')
        if (
            not 1 <= self.max_points <= 200000
            or not self.camera_frame
            or not self.fixed_frame
        ):
            raise ValueError('invalid fusion resources/frames')
        self.condition = threading.Condition(threading.RLock())
        self.images = deque(maxlen=8)
        self.info = None
        self.pending = None
        self.epoch = 0
        self.stopping = False
        self.last_clock = 0
        self.progress = time.monotonic()
        self.last_cloud_stamp = 0
        self.ready_pair = None
        self.reason = 'not_ready'
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.colored = self.create_publisher(
            PointCloud2, '/lidar_camera/colored_points', 10
        )
        self.depth = self.create_publisher(
            Image, '/lidar_camera/depth', qos_profile_sensor_data
        )
        self.diagnostics = self.create_publisher(
            DiagnosticArray, '/lidar_camera/status', 10
        )
        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter('cloud_topic').value,
            self.cloud,
            qos_profile_sensor_data,
        )
        self.create_service(
            GetCapabilities,
            '/platform/lidar_camera/capabilities',
            self.capabilities,
        )
        self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.worker = threading.Thread(target=self.run)
        self.worker.start()

    def clock_health(self):
        now, wall = self.get_clock().now().nanoseconds, time.monotonic()
        rollback = now < self.last_clock
        if rollback:
            self.images.clear()
            self.pending = self.info = self.ready_pair = None
            self.last_cloud_stamp = 0
            self.epoch += 1
            self.progress = wall
        if now > self.last_clock:
            self.progress = wall
        self.last_clock = now
        return not rollback and now > 0 and wall - self.progress <= self.age

    def fresh(self, message, received):
        stamp = stamp_ns(message.header)
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return (
            stamp > 0
            and -0.2 <= age <= self.age
            and time.monotonic() - received <= self.age
        )

    def fault(self, reason):
        self.reason, self.ready_pair = reason, None

    def image(self, message):
        with self.condition:
            try:
                image_layout(message)
                if (
                    not self.clock_health()
                    or message.header.frame_id != self.camera_frame
                    or not self.fresh(message, time.monotonic())
                ):
                    raise ValueError('invalid/stale image time/frame')
                if self.images and stamp_ns(message.header) <= stamp_ns(
                    self.images[-1][0].header
                ):
                    raise ValueError('image time not increasing')
                self.images.append((message, time.monotonic()))
            except ValueError as error:
                self.images.clear()
                self.epoch += 1
                self.fault(str(error))

    def camera(self, message):
        with self.condition:
            try:
                model = camera_model(message)
                if (
                    not self.clock_health()
                    or message.header.frame_id != self.camera_frame
                    or not self.fresh(message, time.monotonic())
                ):
                    raise ValueError('invalid/stale CameraInfo')
                if self.info and model != self.info[2]:
                    self.epoch += 1
                    self.fault('camera_model_changed')
                self.info = (message, time.monotonic(), model)
            except ValueError as error:
                self.info = None
                self.epoch += 1
                self.fault(str(error))

    def cloud(self, message):
        with self.condition:
            if (
                not self.clock_health()
                or not message.header.frame_id
                or not self.fresh(message, time.monotonic())
                or stamp_ns(message.header) <= self.last_cloud_stamp
            ):
                self.epoch += 1
                self.pending = None
                self.fault('invalid/stale/nonmonotonic cloud')
                return
            if (
                message.width * message.height > self.max_points
                or len(message.data) > 8388608
            ):
                self.epoch += 1
                self.pending = None
                self.fault('cloud budget')
                return
            self.last_cloud_stamp = stamp_ns(message.header)
            self.epoch += 1
            self.pending = (message, time.monotonic(), self.epoch)
            self.condition.notify()

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(
                    lambda: self.stopping or self.pending is not None
                )
                if self.stopping:
                    return
                cloud, cloud_received, epoch = self.pending
                self.pending = None
                candidates = [
                    entry for entry in self.images if self.fresh(*entry)
                ]
                info = self.info
                image_entry = (
                    min(
                        candidates,
                        key=lambda e: abs(
                            stamp_ns(e[0].header) - stamp_ns(cloud.header)
                        ),
                    )
                    if candidates
                    else None
                )
            try:
                if (
                    not image_entry
                    or not info
                    or not self.fresh(info[0], info[1])
                ):
                    raise ValueError('image/CameraInfo unavailable')
                image, image_received = image_entry
                if (
                    abs(stamp_ns(image.header) - stamp_ns(cloud.header)) / 1e9
                    > self.tolerance
                ):
                    raise ValueError('image/cloud outside tolerance')
                points = cloud_points(cloud, self.max_points)
                tf = self.buffer.lookup_transform_full(
                    self.camera_frame,
                    Time.from_msg(image.header.stamp),
                    cloud.header.frame_id,
                    Time.from_msg(cloud.header.stamp),
                    self.fixed_frame,
                    timeout=Duration(seconds=0.02),
                )
                colors, depths, count = project(
                    points, image, info[2], tf.transform
                )
                output = PointCloud2(
                    header=cloud.header,
                    height=1,
                    width=count,
                    point_step=16,
                    row_step=16 * count,
                    is_dense=True,
                    is_bigendian=False,
                    data=colors,
                    fields=[
                        PointField(
                            name=n,
                            offset=i * 4,
                            datatype=7 if i < 3 else 6,
                            count=1,
                        )
                        for i, n in enumerate(('x', 'y', 'z', 'rgb'))
                    ],
                )
                depth = Image(
                    header=image.header,
                    width=image.width,
                    height=image.height,
                    encoding='32FC1',
                    step=image.width * 4,
                    is_bigendian=False,
                    data=depths,
                )
                with self.condition:
                    if (
                        self.stopping
                        or epoch != self.epoch
                        or not self.clock_health()
                        or not all(
                            (
                                self.fresh(cloud, cloud_received),
                                self.fresh(image, image_received),
                                self.fresh(info[0], info[1]),
                            )
                        )
                    ):
                        self.fault(
                            'projection expired/invalidated before publish'
                        )
                        continue
                    self.colored.publish(output)
                    self.depth.publish(depth)
                    self.ready_pair = (
                        cloud,
                        cloud_received,
                        image,
                        image_received,
                        info[0],
                        info[1],
                    )
                    self.reason = (
                        'valid_projection'
                        if count
                        else 'valid_empty_projection'
                    )
            except Exception as error:
                with self.condition:
                    self.fault('projection_failed: ' + str(error))

    def ready(self):
        return (
            self.clock_health()
            and self.ready_pair is not None
            and all(
                self.fresh(self.ready_pair[i], self.ready_pair[i + 1])
                for i in (0, 2, 4)
            )
        )

    def tick(self):
        with self.condition:
            valid = self.ready()
            status = DiagnosticStatus(
                name='easyfly/lidar_camera',
                hardware_id=self.camera_frame,
                level=bytes([0 if valid else 2]),
                message=self.reason if valid else 'not_ready: ' + self.reason,
            )
            self.diagnostics.publish(
                DiagnosticArray(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    status=[status],
                )
            )

    def capabilities(self, request, response):
        with self.condition:
            response.header.stamp = self.get_clock().now().to_msg()
            response.names, response.implemented, response.ready = (
                ['colored_cloud_depth'],
                [True],
                [self.ready()],
            )
            response.details = [
                self.reason
                + '; exact source-time TF; no mapping/flight output'
            ]
        return response

    def close(self):
        with self.condition:
            self.stopping = True
            self.condition.notify_all()
        self.worker.join()


def main():
    rclpy.init()
    node = None
    try:
        node = FusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
