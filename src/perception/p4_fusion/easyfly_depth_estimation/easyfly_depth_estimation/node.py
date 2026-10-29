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

"""Expose exact-pair stereo depth with bounded queues and independent source/clock validity."""

import argparse
from collections import OrderedDict
import copy
import hashlib
import json
import os
import re
import signal
import threading
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .stereo import calibration, grayscale, StereoDepth

INPUTS = ('left_image', 'right_image', 'left_info', 'right_info')


def stamp(message):
    """Return a canonical positive source time, never a receipt-time replacement."""
    value = message.header.stamp
    return value.sec * 10**9 + value.nanosec if value.sec >= 0 and value.nanosec < 10**9 else 0


def configuration(path):
    """Freeze explicit topic/frame bindings and finite matcher limits at startup."""
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(descriptor) as stream:
        value = json.loads(stream.read(32769))
    fields = {'version', 'use_sim_time', 'disparities', 'minimum_m', 'maximum_m',
              'left_frame', 'right_frame', *INPUTS}
    if set(value) != fields or value['version'] != 1 or type(value['use_sim_time']) is not bool:
        raise ValueError('invalid stereo configuration fields, version or clock mode')
    topics = [value[key] for key in INPUTS]
    if len(set(topics)) != 4 or any(
            not re.fullmatch('/[A-Za-z_][A-Za-z0-9_/]{0,190}', name) for name in topics):
        raise ValueError('stereo input topics must be distinct absolute names')
    frames = [value['left_frame'], value['right_frame']]
    if frames[0] == frames[1] or any(
            not re.fullmatch('[A-Za-z_][A-Za-z0-9_/]{0,127}', name) for name in frames):
        raise ValueError('left and right optical frames must be valid and distinct')
    StereoDepth(value['disparities'], value['minimum_m'], value['maximum_m'])
    return value


class DepthNode(Node):
    """Compute metric output solely from current rectified images and calibration."""

    def __init__(self, config, context):
        super().__init__('stereo_depth_estimation', context=context, parameter_overrides=[
            Parameter('use_sim_time', value=config['use_sim_time'])])
        self.config = config
        self.matcher = StereoDepth(config['disparities'], config['minimum_m'], config['maximum_m'])
        self.session = uuid.uuid4().hex
        self.queues = {key: OrderedDict() for key in INPUTS}
        self.writers = None
        self.last_source, self.clock_ns = 0, 0
        self.clock_at = self.discovery_at = self.last_result_at = 0.
        self.calibration_signature = None
        self.last_header, self.shape = None, None
        self.valid, self.reason, self.ratio = False, 'awaiting_exact_rectified_pair', 0.
        self.output = {name: self.create_publisher(
            Image, '/depth_estimation/' + name, qos_profile_sensor_data)
                       for name in ['depth', 'disparity', 'variance']}
        self.info_output = self.create_publisher(CameraInfo, '/depth_estimation/camera_info', 5)
        self.status = self.create_publisher(String, '/depth_estimation/status', 5)
        for key in INPUTS:
            kind = CameraInfo if key.endswith('info') else Image
            self.create_subscription(kind, config[key], lambda message, name=key: self.receive(
                name, message), qos_profile_sensor_data)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.status_at = 0.
        self.focal = self.baseline = 0.

    def reset(self, reason):
        """Discard pending pairs when starting a new source or calibration epoch."""
        self.session = uuid.uuid4().hex
        self.last_source = 0
        self.calibration_signature = None
        for queue in self.queues.values():
            queue.clear()
        self.fault(reason)

    def fault(self, reason):
        """Invalidate previously published pixels without giving them a new observation time."""
        if self.valid and self.last_header is not None:
            invalid = np.full(self.shape, np.nan, dtype='<f4')
            for publisher in self.output.values():
                publisher.publish(self.image(invalid))
        self.valid, self.reason, self.ratio = False, reason, 0.

    def receive(self, key, message):
        """Keep at most three immutable source samples per stream; duplicates do not renew age."""
        source = stamp(message)
        frame = self.config['left_frame' if key.startswith('left') else 'right_frame']
        if source <= self.last_source or message.header.frame_id != frame:
            return
        if key.endswith('image'):
            try:
                grayscale(message)
            except ValueError:
                return
        elif len(message.d) > 8:
            return
        queue = self.queues[key]
        if source not in queue:
            queue[source] = (time.monotonic(), message)
        while len(queue) > 3:
            queue.popitem(last=False)

    def image(self, array):
        """Encode original left optical header and IEEE NaN invalidity without scale ambiguity."""
        message = Image(header=copy.deepcopy(self.last_header), encoding='32FC1', is_bigendian=0)
        message.height, message.width = array.shape
        message.step = message.width * 4
        message.data = array.astype('<f4', copy=False).tobytes()
        return message

    def tick(self):
        """Pin native writers and require advancing clock, exact stamps and bounded age."""
        now, clock = time.monotonic(), self.get_clock().now().nanoseconds
        if clock < self.clock_ns:
            self.reset('clock_moved_backwards')
        if clock != self.clock_ns:
            self.clock_ns, self.clock_at = clock, now
        if now - self.discovery_at >= .2:
            self.discovery_at = now
            sources = [self.get_publishers_info_by_topic(self.config[key]) for key in INPUTS]
            writers = tuple(bytes(items[0].endpoint_gid).hex() if len(items) == 1 else ''
                            for items in sources)
            if writers != self.writers:
                self.writers = writers
                self.reset('input_publisher_generation_changed')
        for queue in self.queues.values():
            for source, (received, _) in list(queue.items()):
                if now - received > .5 or not -.1 <= (clock - source) / 1e9 <= .5:
                    del queue[source]
        candidates = set.intersection(*(set(queue) for queue in self.queues.values()))
        clock_valid = clock > 0 and now - self.clock_at <= .5
        if candidates and clock_valid and self.writers and all(self.writers):
            source = max(candidates)
            pair = {key: queue.pop(source) for key, queue in self.queues.items()}
            self.last_source = source
            for queue in self.queues.values():
                for old in list(queue):
                    if old <= source:
                        del queue[old]
            try:
                self.process(pair)
            except (ValueError, cv2.error) as error:
                self.fault(str(error)[:256])
        elif self.valid and (not clock_valid or now - self.last_result_at > .5 or
                             (clock - self.last_source) / 1e9 > .5):
            self.fault('source_or_clock_expired')
        if now - self.status_at >= .2:
            self.status_at = now
            self.status.publish(String(data=json.dumps(dict(
                session=self.session, valid=self.valid, reason=self.reason,
                source_ns=self.last_source, clock_ns=clock, valid_fraction=self.ratio,
                focal_px=self.focal, baseline_m=self.baseline,
                calibration_sha256=self.calibration_signature, method='stereo_sgbm_lr',
                depth_unit='metre', invalid_value='NaN', variance_unit='square_metre',
                variance_scope='0.5 pixel disparity sensitivity; calibration error excluded'))))

    def process(self, pair):
        """Recheck lifetime after native image matching before publishing its measured pixels."""
        left, right = grayscale(pair['left_image'][1]), grayscale(pair['right_image'][1])
        left_info, right_info = pair['left_info'][1], pair['right_info'][1]
        if left.shape != right.shape:
            raise ValueError('stereo image dimensions differ')
        focal, baseline = calibration(left_info, right_info, left.shape)
        signature = hashlib.sha256(json.dumps([
            [info.width, info.height, list(info.k), list(info.r), list(info.p), list(info.d)]
            for info in [left_info, right_info]], sort_keys=True).encode()).hexdigest()
        if self.calibration_signature is not None and signature != self.calibration_signature:
            source = self.last_source
            self.reset('calibration_generation_changed')
            self.last_source = source
        self.calibration_signature = signature
        depth, disparity, variance = self.matcher.compute(left, right, focal, baseline)
        now, clock = time.monotonic(), self.get_clock().now().nanoseconds
        if (any(now - received > .5 for received, _ in pair.values()) or
                not -.1 <= (clock - self.last_source) / 1e9 <= .5 or now - self.clock_at > .5):
            raise ValueError('source expired during correspondence computation')
        self.last_header, self.shape = copy.deepcopy(pair['left_image'][1].header), depth.shape
        self.focal, self.baseline = focal, baseline
        self.ratio = float(np.isfinite(depth).mean())
        self.valid = self.ratio >= .05
        self.reason = 'metric_stereo_depth' if self.valid else 'insufficient_valid_correspondence'
        self.last_result_at = now
        for key, array in zip(['depth', 'disparity', 'variance'], [depth, disparity, variance]):
            self.output[key].publish(self.image(array))
        self.info_output.publish(left_info)


def main():
    """Stop computation before destroying DDS entities and their context."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    config = configuration(parser.parse_args().config)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor(context=context)
    node = None
    try:
        node = DepthNode(config, context)
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
