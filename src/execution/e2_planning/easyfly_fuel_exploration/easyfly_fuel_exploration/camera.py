# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Measured fixed forward camera geometry for the explicitly enabled FUEL profile."""
from collections import OrderedDict
import math
import time

import numpy as np
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


def ns(header):
    if header.stamp.sec < 0 or not 0 <= header.stamp.nanosec < 10**9:
        raise ValueError('invalid camera source time')
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def transform(message):
    t, q = message.transform.translation, message.transform.rotation
    values = np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w])
    if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1.) > 1e-3:
        raise ValueError('invalid camera transform')
    x, y, z, w = values[3:] / np.linalg.norm(values[3:])
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return rotation, values[:3]


def calibration(info):
    matrix = np.asarray(info.k).reshape(3, 3)
    if (not 160 <= info.width <= 1280 or not 120 <= info.height <= 720 or
            not np.isfinite(matrix).all() or not 50 < matrix[0, 0] < 5000 or
            not 50 < matrix[1, 1] < 5000 or
            not np.allclose(matrix[2], [0., 0., 1.], atol=1e-9, rtol=0) or
            abs(matrix[0, 1])+abs(matrix[1, 0]) > 1e-9 or
            not 0 < matrix[0, 2] < info.width or not 0 < matrix[1, 2] < info.height or
            info.distortion_model != 'plumb_bob' or len(info.d) != 5 or
            not np.isfinite(info.d).all() or not np.allclose(info.d, 0., atol=1e-9) or
            info.binning_x > 1 or info.binning_y > 1 or
            any([info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height])):
        raise ValueError('unsupported camera calibration')
    horizontal = min(math.atan(matrix[0, 2]/matrix[0, 0]),
                     math.atan((info.width-matrix[0, 2])/matrix[0, 0]))-.08
    vertical = min(math.atan(matrix[1, 2]/matrix[1, 1]),
                   math.atan((info.height-matrix[1, 2])/matrix[1, 1]))-.08
    if min(horizontal, vertical) < .1 or max(horizontal, vertical) > 1.3:
        raise ValueError('insufficient camera field of view')
    return matrix, horizontal, vertical


class CameraView:
    """No control publisher: source identity and measured projection only."""

    def __init__(self, node):
        self.node = node
        self.streams = {key: OrderedDict() for key in ('image', 'info')}
        self.topics = {key: node.resolve_topic_name(topic) for key, topic in
                       [('image', '/camera/color/image_raw'), ('info', '/camera/color/camera_info')]}
        self.subscriptions = [node.create_subscription(kind, self.topics[key],
            lambda msg, name=key: self.receive(name, msg), qos_profile_sensor_data)
            for key, kind in [('image', Image), ('info', CameraInfo)]]
        self.buffer = Buffer(cache_time=Duration(seconds=2.))
        self.listener = TransformListener(self.buffer, node)
        self.writers = self.candidates = self.signature = self.mount = self.latest = None
        self.candidate_at = self.discovery_at = self.clock_at = 0.
        self.clock_ns = 0
        self.fault = ''
        self.reason = 'camera discovery pending'

    def receive(self, key, message):
        try:
            source = ns(message.header)
        except ValueError:
            return
        queue = self.streams[key]
        if source <= 0 or (queue and source <= next(reversed(queue))):
            return
        if (key == 'image' and (message.encoding not in ('rgb8', 'bgr8') or
                message.is_bigendian or not 160 <= message.width <= 1280 or
                not 120 <= message.height <= 720 or message.step != message.width*3 or
                len(message.data) != message.height*message.step)):
            return
        queue[source] = (time.monotonic(), message)
        while len(queue) > 4:
            queue.popitem(last=False)

    def discover(self, now):
        if now-self.discovery_at < .2:
            return
        self.discovery_at = now
        topics = list(self.topics.values())+['/tf', '/tf_static']
        if self.node.get_parameter('use_sim_time').value:
            topics.append('/clock')
        current = {topic: sorted(bytes(item.endpoint_gid).hex() for item in
                   self.node.get_publishers_info_by_topic(topic)) for topic in topics}
        if any(not gids or (topic not in ('/tf', '/tf_static') and len(gids) != 1)
               for topic, gids in current.items()):
            if self.writers is not None:
                self.fault = 'camera source absent or duplicated'
            self.candidates = None
        elif self.writers is not None:
            if current != self.writers:
                self.fault = 'camera or TF source generation changed'
        elif current != self.candidates:
            self.candidates, self.candidate_at = current, now
        elif now-self.candidate_at >= 1.:
            self.writers = current

    def sample(self):
        now = time.monotonic()
        clock = self.node.get_clock().now().nanoseconds
        if clock < self.clock_ns:
            self.fault = 'camera clock moved backwards'
        if clock != self.clock_ns:
            self.clock_ns, self.clock_at = clock, now
        self.discover(now)
        if self.fault or self.writers is None or clock <= 0 or now-self.clock_at > .5:
            raise RuntimeError(self.fault or 'camera source discovery or clock pending')
        common = sorted(set(self.streams['image']) & set(self.streams['info']), reverse=True)
        for source in common:
            if self.latest and source <= self.latest['source_ns']:
                break
            if not -.1 <= (clock-source)/1e9 <= .5:
                continue
            rows = [self.streams[key][source] for key in ('image', 'info')]
            if any(now-received > .5 for received, _ in rows):
                continue
            image, info = [message for _, message in rows]
            if (image.header.frame_id != 'camera_color_optical_frame' or
                    info.header.frame_id != image.header.frame_id or
                    (image.width, image.height) != (info.width, info.height)):
                raise RuntimeError('invalid exact camera pair')
            timestamp = Time(nanoseconds=source)
            if not all(self.buffer.can_transform(frame, image.header.frame_id, timestamp)
                       for frame in ('base_link', 'odom')):
                continue
            rotation, translation = transform(self.buffer.lookup_transform(
                'base_link', image.header.frame_id, timestamp))
            expected = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
            if not np.allclose(rotation, expected, atol=1e-3, rtol=0) or np.linalg.norm(translation)>1.:
                self.fault = 'camera is not a supported fixed forward mount'
                raise RuntimeError(self.fault)
            signature = (info.width, info.height, tuple(info.k), tuple(info.d), tuple(info.r),
                         tuple(info.p), info.distortion_model, info.binning_x, info.binning_y,
                         info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height)
            matrix, horizontal, vertical = calibration(info)
            mount = np.r_[rotation.flatten(), translation]
            if ((self.signature is not None and signature != self.signature) or
                    (self.mount is not None and not np.allclose(mount, self.mount, atol=1e-7, rtol=0))):
                self.fault = 'camera calibration or extrinsics changed'
                raise RuntimeError(self.fault)
            self.signature, self.mount = signature, mount
            world_rotation, world_translation = transform(self.buffer.lookup_transform(
                'odom', image.header.frame_id, timestamp))
            self.latest = dict(source_ns=source, received=min(row[0] for row in rows),
                matrix=matrix, width=info.width, height=info.height, image=image, info=info,
                rotation=world_rotation, translation=world_translation,
                configuration=dict(sensor_offset=translation.tolist(),
                    horizontal_half_angle=horizontal, vertical_half_angle=vertical, sensor_range=3.))
            break
        if (self.latest is None or now-self.latest['received']>.5 or
                not -.1 <= (clock-self.latest['source_ns'])/1e9 <= .5):
            raise RuntimeError('camera exact source pair or transform expired')
        return self.latest

    def diagnostics(self):
        now=time.monotonic();clock=self.node.get_clock().now().nanoseconds
        rows={}
        for name, queue in self.streams.items():
            rows[name]=[dict(source_ns=source,source_age=(clock-source)/1e9,
                receipt_age=now-received,base_tf=self.buffer.can_transform('base_link',message.header.frame_id,Time(nanoseconds=source)),
                odom_tf=self.buffer.can_transform('odom',message.header.frame_id,Time(nanoseconds=source)))
                for source,(received,message) in queue.items()]
        return dict(clock_ns=clock,fault=self.fault,reason=self.reason,streams=rows,
                    latest_source_ns=None if self.latest is None else self.latest['source_ns'])

    def ready(self):
        try:
            self.sample()
            self.reason="ready"
            return True
        except (ValueError, RuntimeError, TransformException) as exc:
            self.reason = str(exc)
            return False

    def witness(self, points, after_ns):
        """Project native visible frontier through the actual source-time optical TF."""
        view = self.sample()
        values = np.asarray(points, dtype=float)
        if values.ndim != 2 or values.shape[1] != 3 or not 4 <= len(values) <= 256 or not np.isfinite(values).all():
            raise ValueError('invalid native frontier witness')
        camera = (values-view['translation'])@view['rotation']
        z = camera[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            pixels = (camera@view['matrix'].T)[:, :2]/z[:, None]
        visible = ((z>.1) & (np.linalg.norm(camera, axis=1)<=3.15) &
                   (pixels[:, 0]>=8) & (pixels[:, 0]<view['width']-8) &
                   (pixels[:, 1]>=8) & (pixels[:, 1]<view['height']-8))
        return dict(source_ns=view['source_ns'], after_arrival=view['source_ns']>after_ns,
                    visible_count=int(np.count_nonzero(visible)), native_count=len(points),
                    pixels=pixels[visible].tolist(), frontier=values[visible].tolist(),
                    optical_rotation=view['rotation'].tolist(), optical_translation=view['translation'].tolist())
