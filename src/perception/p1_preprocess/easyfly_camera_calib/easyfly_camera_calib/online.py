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

"""Collect an auditable live-image calibration session and write a new candidate only."""

import argparse
from collections import OrderedDict
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import threading
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String
import yaml

from .solver import detect, object_points, solve, view_descriptor


def atomic(path, value):
    """Commit a small session record atomically before reporting its availability."""
    temporary = path.with_suffix(path.suffix + '.pending')
    with temporary.open('w') as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def configuration(path):
    """Freeze board scale, source bindings and bounded acquisition budget."""
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(descriptor) as stream:
        value = json.loads(stream.read(32769))
    fields = {'version', 'stereo', 'use_sim_time', 'left_image', 'right_image', 'left_frame',
              'right_frame', 'columns', 'rows', 'square_m', 'samples', 'timeout_s'}
    if (set(value) != fields or value['version'] != 1 or
            type(value['stereo']) is not bool or type(value['use_sim_time']) is not bool or
            type(value['samples']) is not int or not 20 <= value['samples'] <= 40 or
            not 10 <= value['timeout_s'] <= 600):
        raise ValueError('invalid bounded online calibration configuration')
    object_points(value['columns'], value['rows'], value['square_m'])
    for side in ['left', 'right']:
        if (not re.fullmatch('/[A-Za-z_][A-Za-z0-9_/]{0,190}', value[side + '_image']) or
                not re.fullmatch('[A-Za-z_][A-Za-z0-9_/]{0,127}', value[side + '_frame'])):
            raise ValueError('invalid image source topic or optical frame')
    if value['left_image'] == value['right_image'] or value['left_frame'] == value['right_frame']:
        raise ValueError('left and right sources must be distinct')
    return value


def pixels(message):
    """Decode bounded original image rows, retaining source resolution and colour convention."""
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3}.get(message.encoding, 0)
    if (not channels or not 320 <= message.width <= 1280 or not 240 <= message.height <= 960 or
            not message.width * channels <= message.step <= message.width * channels + 256 or
            len(message.data) != message.height * message.step):
        raise ValueError('invalid raw calibration image')
    value = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
    value = value[:, :message.width * channels]
    if channels == 1:
        return np.ascontiguousarray(value)
    value = value.reshape(message.height, message.width, 3)
    code = cv2.COLOR_RGB2GRAY if message.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(value, code)


class Acquisition(Node):
    """Require source continuity and real corner diversity before solving a frozen data set."""

    def __init__(self, config, output, context):
        super().__init__('online_camera_calibration', context=context, parameter_overrides=[
            Parameter('use_sim_time', value=config['use_sim_time'])])
        cv2.setNumThreads(2)
        self.config, self.output = config, output
        self.session = uuid.uuid4().hex
        self.started = time.monotonic()
        self.cameras = ['left', 'right'] if config['stereo'] else ['left']
        self.queues = {name: OrderedDict() for name in self.cameras}
        self.samples, self.records, self.descriptors = [], [], []
        self.size, self.writers = None, None
        self.clock_ns = self.last_pair = 0
        self.clock_at = self.discovery_at = self.status_at = 0.
        self.reason, self.phase = 'awaiting_diverse_checkerboard_views', 'collecting'
        self.status = self.create_publisher(String, '/camera_calibration/status', 5)
        for name in self.cameras:
            self.create_subscription(Image, config[name + '_image'],
                                     lambda message, side=name: self.receive(side, message),
                                     qos_profile_sensor_data)
        atomic(output / 'configuration.json', config)
        (output / 'images').mkdir()

    def receive(self, camera, message):
        """Cache three exact source samples; repeated data cannot renew a sample's lifetime."""
        source = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if (message.header.stamp.sec < 0 or message.header.stamp.nanosec >= 10**9 or
                source <= self.last_pair or
                message.header.frame_id != self.config[camera + '_frame']):
            return
        try:
            pixels(message)
        except ValueError:
            return
        queue = self.queues[camera]
        if source not in queue:
            queue[source] = (time.monotonic(), message)
        while len(queue) > 3:
            queue.popitem(last=False)

    def save(self):
        """Expose acquisition progress separately from a qualified completed solution."""
        self.status_at = time.monotonic()
        value = dict(session=self.session, phase=self.phase, reason=self.reason,
                     samples=len(self.samples), required_samples=self.config['samples'],
                     source_ns=self.last_pair, clock_ns=self.clock_ns,
                     source_writers=self.writers, elapsed_s=time.monotonic() - self.started)
        atomic(self.output / 'status.json', value)
        self.status.publish(String(data=json.dumps(value)))

    def tick(self):
        """Acquire current exact pairs from one pinned camera generation."""
        now, clock = time.monotonic(), self.get_clock().now().nanoseconds
        if clock < self.clock_ns:
            raise RuntimeError('calibration clock moved backwards; new acquisition required')
        if clock != self.clock_ns:
            self.clock_ns, self.clock_at = clock, now
        if now - self.discovery_at > .2:
            self.discovery_at = now
            endpoints = [self.get_publishers_info_by_topic(self.config[name + '_image'])
                         for name in self.cameras]
            writers = tuple(bytes(items[0].endpoint_gid).hex() if len(items) == 1 else ''
                            for items in endpoints)
            if self.samples and writers != self.writers:
                raise RuntimeError('camera source generation changed during calibration')
            self.writers = writers
        if now - self.started > self.config['timeout_s']:
            raise TimeoutError('not enough valid diverse checkerboard samples before deadline')
        for queue in self.queues.values():
            for source, (received, _) in list(queue.items()):
                if now - received > .7 or not -.1 <= (clock - source) / 1e9 <= .7:
                    del queue[source]
        common = set.intersection(*(set(queue) for queue in self.queues.values()))
        if common and clock > 0 and now - self.clock_at <= .7 and all(self.writers or ()):
            source = max(common)
            pair = {name: queue.pop(source) for name, queue in self.queues.items()}
            self.last_pair = source
            for queue in self.queues.values():
                for old in list(queue):
                    if old <= source:
                        del queue[old]
            self.measure(pair, source)
        if now - self.status_at > .2:
            self.save()

    def measure(self, pair, source):
        """Detect the board in actual pixels and persist only diverse, fully observed samples."""
        images = {name: pixels(message) for name, (_, message) in pair.items()}
        shape = next(iter(images.values())).shape
        if any(image.shape != shape for image in images.values()):
            raise ValueError('stereo calibration image sizes differ')
        size = tuple(reversed(shape))
        if self.size is not None and self.size != size:
            raise RuntimeError('camera image dimensions changed during calibration')
        corners = {name: detect(image, self.config['columns'], self.config['rows'])
                   for name, image in images.items()}
        if any(value is None for value in corners.values()):
            self.reason = 'checkerboard_not_fully_visible'
            return
        if any(time.monotonic() - received > .7 for received, _ in pair.values()):
            self.reason = 'frame_expired_during_corner_detection'
            return
        descriptor = view_descriptor(corners['left'], size)
        if any(np.linalg.norm(descriptor - previous) < .035 for previous in self.descriptors):
            self.reason = 'need_a_different_board_pose'
            return
        self.size = size
        record = dict(source_ns=source, source_writers=self.writers, images={}, corners={})
        for name, image in images.items():
            filename = f'images/{len(self.samples):02d}-{name}.png'
            if not cv2.imwrite(str(self.output / filename), image):
                raise OSError('calibration sample image write failed')
            record['images'][name] = dict(file=filename, sha256=hashlib.sha256(
                (self.output / filename).read_bytes()).hexdigest(),
                frame=self.config[name + '_frame'])
            record['corners'][name] = corners[name].tolist()
        self.samples.append(corners)
        self.descriptors.append(descriptor)
        self.records.append(record)
        atomic(self.output / 'samples.json', self.records)
        self.reason = 'diverse_sample_recorded'

    def finish(self):
        """Write new candidates with evidence and withheld-view checks, never activate them."""
        endpoints = [self.get_publishers_info_by_topic(self.config[name + '_image'])
                     for name in self.cameras]
        current = tuple(bytes(items[0].endpoint_gid).hex() if len(items) == 1 else ''
                        for items in endpoints)
        if not all(current) or current != self.writers:
            raise RuntimeError('camera source changed before calibration solve')
        self.phase, self.reason = 'solving', 'frozen_source_samples'
        self.save()
        candidate = solve(self.samples, self.size,
                          (self.config['columns'], self.config['rows'], self.config['square_m']),
                          self.config['stereo'])
        candidate.update(session=self.session, source_writers=self.writers,
                         source_frames={name: self.config[name + '_frame']
                                        for name in self.cameras},
                         samples_sha256=hashlib.sha256(
                             (self.output / 'samples.json').read_bytes()).hexdigest(),
                         status='validated_candidate_not_activated')
        atomic(self.output / 'candidate.json', candidate)
        for name, value in candidate['cameras'].items():
            matrix = np.array(value['matrix'])
            intrinsics = dict(width=self.size[0], height=self.size[1], fx=float(matrix[0, 0]),
                              fy=float(matrix[1, 1]), cx=float(matrix[0, 2]),
                              cy=float(matrix[1, 2]),
                              distortion_model='plumb_bob', d=value['distortion'])
            (self.output / ('intrinsics-' + name + '.yaml')).write_text(
                yaml.safe_dump(dict(intrinsics=intrinsics), sort_keys=False))
        self.phase, self.reason = 'complete', 'qualified_candidate_available'
        self.save()


def main():
    """Acquire a new private calibration evidence directory and stop owned ROS cleanly."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    arguments = parser.parse_args()
    config = configuration(arguments.config)
    output = Path(arguments.output)
    if not output.is_absolute():
        raise ValueError('calibration output must be a new absolute directory')
    output.mkdir(mode=0o700, exist_ok=False)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor(context=context)
    node = None
    try:
        node = Acquisition(config, output, context)
        executor.add_node(node)
        while len(node.samples) < config['samples']:
            executor.spin_once(timeout_sec=.01)
            if stopping.is_set():
                raise InterruptedError('online calibration acquisition canceled')
            node.tick()
        node.finish()
    except BaseException as error:
        if node is not None:
            node.phase, node.reason = 'failed', str(error)
            node.save()
        raise
    finally:
        if node is not None:
            executor.remove_node(node)
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.shutdown()
