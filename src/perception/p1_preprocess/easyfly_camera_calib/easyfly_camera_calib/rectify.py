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

"""Apply an explicitly selected, hash-verified calibration candidate to fresh stereo images."""

import argparse
from collections import OrderedDict
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import stat
import threading
import time
import uuid

import cv2
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from .online import pixels


def load(path, expected_sha256):
    """Verify exact candidate bytes and bounded stereo geometry before allocating remap grids."""
    if not re.fullmatch('[0-9a-f]{64}', expected_sha256):
        raise ValueError('a reviewed calibration SHA-256 is required')
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or not 0 < info.st_size <= 131072:
            raise ValueError('candidate must be a bounded regular JSON file')
        with os.fdopen(descriptor, 'rb', closefd=False) as stream:
            raw = stream.read(131073)
    finally:
        os.close(descriptor)
    if hashlib.sha256(raw).hexdigest() != expected_sha256:
        raise ValueError('calibration candidate content changed after selection')
    value = json.loads(raw)
    if (value['version'] != 1 or value['units'] != 'metre' or
            value['status'] != 'validated_candidate_not_activated' or
            value['distortion_model'] != 'plumb_bob' or
            not 320 <= value['width'] <= 1280 or not 240 <= value['height'] <= 960):
        raise ValueError('unsupported calibration candidate format or resolution')
    maps = {}
    for side in ['left', 'right']:
        frame = value['source_frames'][side]
        if not re.fullmatch('[A-Za-z_][A-Za-z0-9_/]{0,110}', frame):
            raise ValueError('invalid source optical frame')
        matrix = np.array(value['cameras'][side]['matrix'], dtype=np.float64)
        distortion = np.array(value['cameras'][side]['distortion'], dtype=np.float64)
        rotation = np.array(value['stereo'][side + '_rectification'], dtype=np.float64)
        projection = np.array(value['stereo'][side + '_projection'], dtype=np.float64)
        if (matrix.shape != (3, 3) or distortion.shape != (5,) or rotation.shape != (3, 3) or
                projection.shape != (3, 4) or not np.isfinite(np.r_[
                    matrix.ravel(), distortion.ravel(), rotation.ravel(),
                    projection.ravel()]).all() or
                not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6) or
                abs(np.linalg.det(rotation) - 1.) > 1e-6 or
                not np.allclose(projection[2], [0., 0., 1., 0.], atol=1e-8) or
                not 50 <= projection[0, 0] <= 5000 or not 50 <= projection[1, 1] <= 5000):
            raise ValueError('invalid finite rectification matrices')
        maps[side] = cv2.initUndistortRectifyMap(
            matrix, distortion, rotation, projection[:, :3],
            (value['width'], value['height']), cv2.CV_32FC1)
    left, right = np.array(value['stereo']['left_projection']), np.array(
        value['stereo']['right_projection'])
    if (value['source_frames']['left'] == value['source_frames']['right'] or
            not np.allclose(left[:, :3], right[:, :3], atol=1e-6) or
            abs(left[0, 3]) > 1e-8 or abs(right[1, 3]) > 1e-8 or
            not .02 <= -right[0, 3] / right[0, 0] <= 1.):
        raise ValueError('candidate is not a supported horizontal metric stereo pair')
    return value, maps


class Rectifier(Node):
    """Keep source time and optical transforms, and stop on camera writer replacement."""

    def __init__(self, calibration, maps, digest, topics, simulated, context):
        super().__init__('calibrated_stereo_rectifier', context=context, parameter_overrides=[
            Parameter('use_sim_time', value=simulated)])
        cv2.setNumThreads(2)
        self.calibration, self.maps, self.digest = calibration, maps, digest
        self.topics, self.session = topics, uuid.uuid4().hex
        self.queues = {name: OrderedDict() for name in ['left', 'right']}
        self.images, self.infos = {}, {}
        self.clock_ns = self.last_source = 0
        self.clock_at = self.last_output_at = self.discovery_at = self.status_at = 0.
        self.writers = None
        self.fault = ''
        self.transforms = StaticTransformBroadcaster(self)
        transforms = []
        for side in ['left', 'right']:
            self.create_subscription(Image, topics[side],
                                     lambda message, name=side: self.receive(name, message),
                                     qos_profile_sensor_data)
            prefix = '/stereo/rectified/' + side
            self.images[side] = self.create_publisher(
                Image, prefix + '/image_raw', qos_profile_sensor_data)
            self.infos[side] = self.create_publisher(CameraInfo, prefix + '/camera_info', 5)
            transform = TransformStamped()
            transform.header.frame_id = calibration['source_frames'][side]
            transform.child_frame_id = transform.header.frame_id + '_rectified'
            rotation = np.array(calibration['stereo'][side + '_rectification']).T
            quaternion = Rotation.from_matrix(rotation).as_quat()
            (transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w) = quaternion.tolist()
            transforms.append(transform)
        self.transforms.sendTransform(transforms)
        self.status = self.create_publisher(String, '/stereo/rectified/status', 5)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, side, message):
        """Keep bounded source pairs; source replacements never inherit a previous calibration."""
        source = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if (self.fault or message.header.stamp.sec < 0 or message.header.stamp.nanosec >= 10**9 or
                source <= self.last_source or
                message.header.frame_id != self.calibration['source_frames'][side] or
                message.width != self.calibration['width'] or
                message.height != self.calibration['height']):
            return
        try:
            pixels(message)
        except ValueError:
            return
        queue = self.queues[side]
        if source not in queue:
            queue[source] = (time.monotonic(), message)
        while len(queue) > 3:
            queue.popitem(last=False)

    def tick(self):
        """Remap only exact fresh pairs from pinned sources and an advancing clock."""
        now, clock = time.monotonic(), self.get_clock().now().nanoseconds
        if clock < self.clock_ns:
            self.fault = 'clock_moved_backwards_restart_required'
        if clock != self.clock_ns:
            self.clock_ns, self.clock_at = clock, now
        if now - self.discovery_at > .2:
            self.discovery_at = now
            endpoints = [self.get_publishers_info_by_topic(self.topics[side])
                         for side in ['left', 'right']]
            writers = tuple(bytes(items[0].endpoint_gid).hex() if len(items) == 1 else ''
                            for items in endpoints)
            if self.writers is not None and self.writers != writers:
                self.fault = 'camera_publisher_changed_restart_required'
            elif self.writers is None and all(writers):
                self.writers = writers
        for queue in self.queues.values():
            for source, (received, _) in list(queue.items()):
                if now - received > .5 or not -.1 <= (clock - source) / 1e9 <= .5:
                    del queue[source]
        candidates = self.queues['left'].keys() & self.queues['right'].keys()
        if (candidates and not self.fault and self.writers is not None and
                now - self.clock_at <= .5):
            source = max(candidates)
            pair = {side: queue.pop(source) for side, queue in self.queues.items()}
            self.last_source = source
            for queue in self.queues.values():
                for old in list(queue):
                    if old <= source:
                        del queue[old]
            self.publish_pair(pair)
        if now - self.status_at > .2:
            self.status_at = now
            valid = (not self.fault and now - self.last_output_at <= .5 and
                     now - self.clock_at <= .5 and -.1 <= (clock - self.last_source) / 1e9 <= .5)
            self.status.publish(String(data=json.dumps(dict(
                session=self.session, calibration_sha256=self.digest, valid=valid,
                source_ns=self.last_source, clock_ns=clock,
                reason=self.fault or ('rectified_pair' if valid else 'source_unavailable')))))

    def publish_pair(self, pair):
        """Prepare both images with original stamps and corrected optical axes."""
        outputs = {}
        for side, (_, message) in pair.items():
            channels = 1 if message.encoding == 'mono8' else 3
            raw = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
            raw = raw[:, :message.width * channels]
            if channels == 3:
                raw = raw.reshape(message.height, message.width, 3)
            remapped = cv2.remap(
                raw, *self.maps[side], interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT)
            image = Image(header=copy.deepcopy(message.header), encoding=message.encoding,
                          width=message.width, height=message.height, is_bigendian=0,
                          step=message.width * channels, data=remapped.tobytes())
            image.header.frame_id += '_rectified'
            projection = np.array(self.calibration['stereo'][side + '_projection'])
            info = CameraInfo(header=copy.deepcopy(image.header), width=image.width,
                              height=image.height, distortion_model='plumb_bob')
            info.k = projection[:, :3].reshape(-1).tolist()
            info.p = projection.reshape(-1).tolist()
            info.r, info.d = np.eye(3).reshape(-1).tolist(), [0.] * 5
            outputs[side] = image, info
        now, clock = time.monotonic(), self.get_clock().now().nanoseconds
        if (any(now - received > .5 for received, _ in pair.values()) or
                not -.1 <= (clock - self.last_source) / 1e9 <= .5):
            return
        for side, (image, info) in outputs.items():
            self.images[side].publish(image)
            self.infos[side].publish(info)
        self.last_output_at = now


def main():
    """Require an explicit calibration hash before exposing calibrated image outputs."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--sha256', required=True)
    parser.add_argument('--left-topic', default='/stereo/left/image_raw')
    parser.add_argument('--right-topic', default='/stereo/right/image_raw')
    parser.add_argument('--system-clock', action='store_true')
    args = parser.parse_args()
    topics = dict(left=args.left_topic, right=args.right_topic)
    if len(set(topics.values())) != 2 or any(
            not re.fullmatch('/[A-Za-z_][A-Za-z0-9_/]{0,190}', topic)
            for topic in topics.values()):
        raise ValueError('stereo source topics must be distinct absolute names')
    calibration, maps = load(Path(args.calibration), args.sha256)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor(context=context)
    node = None
    try:
        node = Rectifier(calibration, maps, args.sha256, topics, not args.system_clock, context)
        executor.add_node(node)
        while not stopping.is_set():
            executor.spin_once(timeout_sec=.05)
    finally:
        if node is not None:
            executor.remove_node(node)
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.shutdown()
