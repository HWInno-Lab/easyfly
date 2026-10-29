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

"""Pair exact observed boxes and camera frames, emitting only fresh appearance identities."""

import argparse
from collections import OrderedDict
import json
import math
import os
from pathlib import Path
import re
import signal
import threading
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from easyfly_msgs.msg import TrackInfoArray, VehicleIdentity, VehicleIdentityArray
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

from .appearance import AppearanceModel, Gallery, MODEL_SHA256
from .enrollment import load as load_enrollment


def stamp(header):
    """Read a canonical positive source stamp without substituting receipt time."""
    if not 0 <= header.stamp.nanosec < 10**9 or header.stamp.sec < 0:
        return 0
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def configuration(path):
    """Require an explicit finite input set and valid, distinct camera bindings."""
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(descriptor) as stream:
        value = json.loads(stream.read(32769))
    if set(value) != {'version', 'use_sim_time', 'cameras', 'threshold', 'margin'}:
        raise ValueError('unexpected vehicle ReID configuration fields')
    if value['version'] != 1 or type(value['use_sim_time']) is not bool:
        raise ValueError('invalid version or clock selection')
    cameras = value['cameras']
    if not isinstance(cameras, list) or not 1 <= len(cameras) <= 4:
        raise ValueError('configure one to four actual camera inputs')
    identifiers, topics = set(), set()
    for camera in cameras:
        if set(camera) != {'id', 'image', 'observed_tracks', 'frame'}:
            raise ValueError('invalid camera binding fields')
        if not re.fullmatch('[a-z][a-z0-9_]{0,31}', camera['id']):
            raise ValueError('invalid camera identity')
        if camera['id'] in identifiers:
            raise ValueError('camera identities must be unique')
        identifiers.add(camera['id'])
        if not re.fullmatch('[A-Za-z_][A-Za-z0-9_/]{0,127}', camera['frame']):
            raise ValueError('invalid optical frame')
        for key in ['image', 'observed_tracks']:
            topic = camera[key]
            if not re.fullmatch('/[A-Za-z_][A-Za-z0-9_/]{0,190}', topic) or topic in topics:
                raise ValueError('input topics must be unique absolute ROS names')
            topics.add(topic)
    Gallery(value['threshold'], value['margin'])
    return value


class Reidentification(Node):
    """Bound work and invalidate the gallery when publishers or clock generations change."""

    def __init__(self, config, model, context, references=None, gallery_sha256=''):
        super().__init__('vehicle_reidentification', context=context, parameter_overrides=[
            Parameter('use_sim_time', value=config['use_sim_time'])])
        self.config, self.model = config, model
        self.references, self.gallery_sha256 = references or {}, gallery_sha256
        self.publisher = self.create_publisher(VehicleIdentityArray, '/vehicle_identities', 5)
        self.session = uuid.uuid4().hex
        self.gallery = Gallery(config['threshold'], config['margin'])
        self.gallery.seed(self.references)
        self.inputs = {camera['id']: dict(
            config=camera, images=OrderedDict(), tracks=OrderedDict(), last=0, processed_at=0.,
            header=None, gids=None, valid=False, reason='awaiting_input')
            for camera in config['cameras']}
        self.clock_ns, self.clock_at, self.last_discovery = 0, 0., 0.
        self.last_invalid = 0.
        for camera in config['cameras']:
            self.create_subscription(
                Image, camera['image'],
                lambda message, key=camera['id']: self.receive(key, 'images', message),
                qos_profile_sensor_data)
            self.create_subscription(
                TrackInfoArray, camera['observed_tracks'],
                lambda message, key=camera['id']: self.receive(key, 'tracks', message),
                qos_profile_sensor_data)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def reset(self, reason):
        """Start a new identity epoch; never carry a prior source gallery into replacement data."""
        self.session = uuid.uuid4().hex
        self.gallery = Gallery(self.config['threshold'], self.config['margin'])
        self.gallery.seed(self.references)
        for row in self.inputs.values():
            row['images'].clear()
            row['tracks'].clear()
            row.update(last=0, valid=False, processed_at=0., reason=reason)

    def receive(self, camera, kind, message):
        """Keep four original samples per input; repeated stamps cannot replace fresh data."""
        row = self.inputs[camera]
        source = stamp(message.header)
        if (not source or source <= row['last'] or
                message.header.frame_id != row['config']['frame']):
            return
        if kind == 'images':
            if (message.encoding not in ('rgb8', 'bgr8') or
                    not 24 <= message.width <= 1920 or not 24 <= message.height <= 1080 or
                    not message.width * 3 <= message.step <= message.width * 3 + 256 or
                    len(message.data) != message.height * message.step):
                return
        elif len(message.tracks) > 8 or any(
                track.header != message.header for track in message.tracks):
            return
        cache = row[kind]
        if source not in cache:
            cache[source] = (time.monotonic(), message)
        while len(cache) > 4:
            cache.popitem(last=False)

    def discovery(self):
        """Pin both actual DDS writer identities; missing/duplicate sources remain invalid."""
        changed = False
        for row in self.inputs.values():
            sources = [self.get_publishers_info_by_topic(row['config'][key])
                       for key in ['image', 'observed_tracks']]
            gids = tuple(bytes(items[0].endpoint_gid).hex() if len(items) == 1 else ''
                         for items in sources)
            if row['gids'] != gids:
                changed = True
                row['gids'] = gids
        if changed:
            self.reset('input_publisher_generation_changed')

    def invalid(self, camera, row, reason):
        """Send explicit invalid state without re-dating an old image."""
        message = VehicleIdentityArray(camera_id=camera, session_id=self.session,
                                       model_sha256=MODEL_SHA256,
                                       gallery_sha256=self.gallery_sha256,
                                       valid=False, reason=reason)
        if row['header'] is not None:
            message.header = row['header']
        self.publisher.publish(message)

    def tick(self):
        """Process the newest exact pair and recheck source age after actual inference."""
        now, clock = time.monotonic(), self.get_clock().now().nanoseconds
        if clock < self.clock_ns:
            self.reset('clock_moved_backwards')
        if clock != self.clock_ns:
            self.clock_ns, self.clock_at = clock, now
        if now - self.last_discovery > .2:
            self.last_discovery = now
            self.discovery()
        for camera, row in self.inputs.items():
            for cache in [row['images'], row['tracks']]:
                for source, (received, _) in list(cache.items()):
                    if now - received > 1. or not -.2 <= (clock - source) / 1e9 <= 1.:
                        del cache[source]
            candidates = row['images'].keys() & row['tracks'].keys()
            if candidates and row['gids'] and all(row['gids']) and now - self.clock_at <= 1.:
                source = max(candidates)
                received_image, image = row['images'].pop(source)
                received_tracks, tracks = row['tracks'].pop(source)
                row['last'], row['header'] = source, image.header
                for cache in [row['images'], row['tracks']]:
                    for old in list(cache):
                        if old <= source:
                            del cache[old]
                try:
                    self.process(camera, row, image, tracks,
                                 min(received_image, received_tracks))
                except (ValueError, RuntimeError) as error:
                    row.update(valid=False, reason=str(error)[:256])
            fresh = (row['valid'] and now - row['processed_at'] <= 1. and
                     -.2 <= (clock - row['last']) / 1e9 <= 1. and
                     now - self.clock_at <= 1. and row['gids'] and all(row['gids']))
            if not fresh and now - self.last_invalid > .2:
                row['valid'] = False
                self.invalid(camera, row, row['reason'] if row['reason'] else 'input_expired')
        if now - self.last_invalid > .2:
            self.last_invalid = now

    def process(self, camera, row, image, tracks, received):
        """Infer whole-vehicle crops; source track IDs are exposed solely as correlation fields."""
        rgb = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
        rgb = rgb[:, :image.width * 3].reshape(image.height, image.width, 3)
        if image.encoding == 'bgr8':
            rgb = rgb[:, :, ::-1]
        output = VehicleIdentityArray(header=image.header, camera_id=camera,
                                      session_id=self.session, model_sha256=MODEL_SHA256,
                                      gallery_sha256=self.gallery_sha256)
        vectors, valid_indices = [], []
        for index, track in enumerate(tracks.tracks):
            item = VehicleIdentity(observation_index=index, source_track_id=track.track_id,
                                   class_name=track.class_name[:32], state='invalid_crop',
                                   cosine_distance=2., distance_margin=0.)
            output.identities.append(item)
            values = [track.confidence, track.bbox_center_x, track.bbox_center_y,
                      track.bbox_width, track.bbox_height]
            if (not all(math.isfinite(value) for value in values) or
                    track.class_name not in ('car', 'truck', 'bus') or
                    not .5 <= track.confidence <= 1. or
                    not all(0. <= value <= 1. for value in values[1:])):
                continue
            x, y, width, height = values[1:]
            left = round((x - width / 2) * image.width)
            right = round((x + width / 2) * image.width)
            top = round((y - height / 2) * image.height)
            bottom = round((y + height / 2) * image.height)
            if not (0 <= left < right <= image.width and 0 <= top < bottom <= image.height):
                continue
            try:
                vectors.append(self.model.embed(rgb[top:bottom, left:right]))
            except ValueError:
                continue
            item.bbox_center_x, item.bbox_center_y = x, y
            item.bbox_width, item.bbox_height = width, height
            valid_indices.append(index)
        clock = self.get_clock().now().nanoseconds
        if (time.monotonic() - received > 1. or
                not -.2 <= (clock - stamp(image.header)) / 1e9 <= 1.):
            raise ValueError('source expired during inference')
        matches = self.gallery.associate(camera, stamp(image.header), vectors, time.monotonic())
        for index, match in zip(valid_indices, matches):
            item = output.identities[index]
            item.identity_id, item.state = match['identity'], match['state']
            item.cosine_distance, item.distance_margin = match['distance'], match['margin']
        output.valid, output.reason = True, 'appearance_observations'
        row.update(valid=True, reason='', processed_at=time.monotonic())
        self.publisher.publish(output)


def main():
    """Keep all input bindings explicit and retire DDS entities before their context."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--gallery')
    args = parser.parse_args()
    config = configuration(args.config)
    model = AppearanceModel(Path(get_package_share_directory('easyfly_vehicle_reid')) /
                            'models/osnet_ain_x1_0_vehicle_reid.onnx')
    references, gallery_hash = load_enrollment(args.gallery, model) if args.gallery else ({}, '')
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = SingleThreadedExecutor(context=context)
    try:
        node = Reidentification(config, model, context, references, gallery_hash)
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
