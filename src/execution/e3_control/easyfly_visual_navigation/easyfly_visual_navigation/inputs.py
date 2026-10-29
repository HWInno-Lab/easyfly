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

"""Consume exact visual/depth/calibration tuples and source-time transforms for one task."""

from collections import OrderedDict
import math
import time

from easyfly_msgs.msg import (
    ClassifiedObstacleArray, ColorLineObservationArray, EllipseObservationArray,
    TargetPredictionArray, UAVState,
)
from easyfly_msgs.srv import GetTrajectoryState
import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import stamp, visual_target


class Inputs(Node):
    """Pin source generations; source stamps, receipts and clock progress expire independently."""

    def __init__(self, config, context):
        super().__init__('visual_navigation', context=context, parameter_overrides=[
            Parameter('use_sim_time', value=config['use_sim_time'])])
        self.config = config
        self.streams = {}
        self.topics = dict(observation=config['observation_topic'], depth=config['depth_topic'],
                           info=config['camera_info_topic'], state='/uav_state',
                           semantic='/obstacles_classified')
        kinds = dict(observation=(ColorLineObservationArray if config['mode'] == 'line'
                                  else EllipseObservationArray), depth=Image, info=CameraInfo,
                     state=UAVState, semantic=ClassifiedObstacleArray)
        if config['use_predictions']:
            self.topics['prediction'] = '/target_predictions'
            kinds['prediction'] = TargetPredictionArray
        for key, topic in self.topics.items():
            self.streams[key] = OrderedDict()
            self.create_subscription(kinds[key], topic,
                                     lambda message, name=key: self.receive(name, message),
                                     qos_profile_sensor_data)
        self.clock_ns = 0
        self.clock_at = self.discovery_at = 0.
        self.writers = None
        self.discovered_candidates = None
        self.candidates_since = 0.
        self.fault = ''
        self.visual = None
        self.calibration = None
        self.visual_reason = 'awaiting measured visual geometry'
        self.last_measured = 0
        self.samples = 0
        self.buffer = Buffer(cache_time=Duration(seconds=2.))
        self.listener = TransformListener(self.buffer, self)
        self.trajectory_state = self.create_client(GetTrajectoryState,
                                                   '/flight_control/trajectory_state')

    def receive(self, key, message):
        """Bound caches and never renew receipt time by repeating a source stamp."""
        try:
            source = stamp(message.header.stamp)
        except ValueError:
            return
        queue = self.streams[key]
        if source <= 0 or (queue and source <= next(reversed(queue))):
            return
        if key == 'depth' and (message.encoding != '32FC1' or message.is_bigendian or
                               not 160 <= message.width <= 1280 or
                               not 120 <= message.height <= 720 or
                               message.step != message.width * 4 or
                               len(message.data) != message.height * message.step):
            return
        if key == 'semantic' and len(message.obstacles) > 128:
            self.fault = 'semantic budget exceeded'
            return
        if key == 'prediction' and len(message.predictions) > 128:
            self.fault = 'prediction budget exceeded'
            return
        if key == 'observation' and len(message.observations) > 32:
            self.fault = 'visual observation budget exceeded'
            return
        queue[source] = (time.monotonic(), message)
        while len(queue) > (4 if key in ('observation', 'depth', 'info') else 2):
            queue.popitem(last=False)

    def latest(self, key):
        """Require a current distinct source message, never a newer delivery of stale data."""
        queue = self.streams[key]
        if not queue:
            raise RuntimeError(key + ' unavailable')
        source = next(reversed(queue))
        received, message = queue[source]
        if (time.monotonic() - received > .5 or
                not -.1 <= (self.clock_ns - source) / 1e9 <= .5):
            raise RuntimeError(key + ' source expired')
        return message

    def discover(self):
        """Freeze actual DDS writer sets, including the transform and simulation clock owners."""
        now = time.monotonic()
        if now - self.discovery_at < .2:
            return
        self.discovery_at = now
        topics = list(self.topics.values()) + ['/tf', '/tf_static']
        if self.config['use_sim_time']:
            topics.append('/clock')
        current = {}
        for topic in topics:
            endpoints = self.get_publishers_info_by_topic(topic)
            gids = sorted(bytes(item.endpoint_gid).hex() for item in endpoints)
            if not gids or (topic not in ('/tf', '/tf_static') and len(gids) != 1):
                if self.writers is not None:
                    self.fault = 'source missing or duplicated: ' + topic
                else:
                    self.discovered_candidates = None
                return
            current[topic] = gids
        if self.writers is None:
            # DDS discovery arrives incrementally. Establish a stable complete initial
            # set before accepting measurements or sending any flight intent.
            if current != self.discovered_candidates:
                self.discovered_candidates, self.candidates_since = current, now
            elif now - self.candidates_since >= 1.:
                self.writers = current
        elif current != self.writers:
            changed = [topic for topic in current if current[topic] != self.writers[topic]]
            self.fault = 'source writer generation changed: ' + ', '.join(changed)

    def state(self):
        """Validate native local position, attitude, velocity and battery, including frame."""
        message = self.latest('state')
        values = [message.position_x, message.position_y, message.position_z,
                  message.velocity_x, message.velocity_y, message.velocity_z,
                  message.roll, message.pitch, message.yaw, message.battery_remaining]
        if (message.header.frame_id != 'odom' or not all(math.isfinite(v) for v in values) or
                not all([message.connected, message.pose_valid, message.velocity_valid,
                         message.attitude_valid, message.battery_valid]) or
                not .15 <= message.battery_remaining <= 1. or
                abs(message.roll) > .35 or abs(message.pitch) > .35):
            raise RuntimeError('native vehicle state invalid')
        return message, np.array(values[:3]), np.array(values[3:6])

    def update(self):
        """Refresh measured geometry without rewriting source time or using current-time TF."""
        now, clock = time.monotonic(), self.get_clock().now().nanoseconds
        if clock < self.clock_ns:
            self.fault = 'clock moved backwards'
        if clock != self.clock_ns:
            self.clock_ns, self.clock_at = clock, now
        self.discover()
        if self.fault or self.writers is None or clock <= 0 or now - self.clock_at > .5:
            raise RuntimeError(self.fault or 'source discovery or clock progress unavailable')
        state, position, velocity = self.state()
        semantic = self.latest('semantic')
        if semantic.header.frame_id != 'odom':
            raise RuntimeError('semantic coordinate frame invalid')
        if self.config['use_predictions']:
            prediction = self.latest('prediction')
            if prediction.header.frame_id != 'odom':
                raise RuntimeError('prediction coordinate frame invalid')
            predictions = prediction.predictions
        else:
            predictions = []
        self.measure(position)
        if self.fault:
            raise RuntimeError(self.fault)
        if (self.visual is None or now - self.visual['received'] > .5 or
                not -.1 <= (clock - self.visual['source_ns']) / 1e9 <= .5):
            reason = self.visual_reason if self.visual is None else (
                'visual observation expired: ' + self.visual_reason)
            raise RuntimeError(reason)
        return state, position, velocity, semantic, predictions, self.visual

    def measure(self, position):
        """Require an exact source tuple and calibrated optical axes."""
        names = ('observation', 'depth', 'info')
        common = set.intersection(*(set(self.streams[key]) for key in names))
        common = [source for source in common if source > self.last_measured]
        if not common:
            return
        # Camera callbacks may precede pose/TF callbacks by one native sample.
        # Select the newest still-fresh exact tuple whose source-time TF exists;
        # never replace it with a current-time transform or future extrapolation.
        common = sorted(common, reverse=True)
        available = [source for source in common if
                     -.1 <= (self.clock_ns - source) / 1e9 <= .5 and
                     self.buffer.can_transform('odom', self.config['optical_frame'],
                                               Time(nanoseconds=source))]
        if not available:
            self.visual_reason = 'waiting for exact source-time optical transform'
            return
        source = available[0]
        pair = [self.streams[key][source] for key in names]
        if any(time.monotonic() - received > .5 for received, _ in pair):
            return
        observation, depth_message, info = [message for _, message in pair]
        if (any(message.header.frame_id != self.config['optical_frame']
                for message in [observation, depth_message, info]) or
                not observation.valid or not observation.observations or
                (observation.image_width, observation.image_height) !=
                (depth_message.width, depth_message.height) or
                (info.width, info.height) != (depth_message.width, depth_message.height)):
            self.visual_reason = 'invalid or empty exact visual tuple'
            return
        signature = (info.width, info.height, tuple(info.k), tuple(info.d), tuple(info.r),
                     tuple(info.p), info.distortion_model, info.binning_x, info.binning_y,
                     info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height)
        if self.calibration is not None and signature != self.calibration:
            self.fault = 'visual camera calibration changed'
            return
        matrix = np.array(info.k).reshape(3, 3)
        if (not np.isfinite(matrix).all() or not 50 < matrix[0, 0] < 5000 or
                not 50 < matrix[1, 1] < 5000 or
                not np.allclose(matrix[2], [0., 0., 1.]) or
                info.distortion_model != 'plumb_bob' or
                len(info.d) != 5 or not np.allclose(info.d, 0.) or
                info.binning_x > 1 or info.binning_y > 1 or
                info.roi.x_offset or info.roi.y_offset):
            self.visual_reason = 'unsupported visual camera calibration'
            return
        self.calibration = signature
        try:
            transform = self.buffer.lookup_transform('odom', self.config['optical_frame'],
                                                     Time(nanoseconds=source))
            if stamp(transform.header.stamp) != source:
                raise ValueError('transform is not at the visual source timestamp')
            q = transform.transform.rotation
            quaternion = np.array([q.x, q.y, q.z, q.w])
            if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion) - 1) > .001:
                raise ValueError('invalid optical transform rotation')
            rotation = Rotation.from_quat(quaternion).as_matrix()
            t = transform.transform.translation
            translation = np.array([t.x, t.y, t.z])
            if not np.isfinite(translation).all():
                raise ValueError('invalid optical transform translation')
            depth = np.frombuffer(depth_message.data, '<f4').reshape(info.height, info.width)
            desired, center, direction = visual_target(
                self.config['mode'], observation.observations, depth, matrix, rotation,
                translation, position, np.array(self.config['target']), self.config['plane_z'])
            if self.visual is not None:
                old = self.visual
                if self.config['mode'] == 'ellipse':
                    changed = np.linalg.norm(center - old['center']) > .25
                else:
                    axis = old['direction']
                    changed = (abs(np.cross(axis, center[:2] - old['center'][:2])) > .2 or
                               abs(direction @ axis) < .9)
                if changed:
                    self.fault = 'measured visual route changed discontinuously'
                    raise ValueError(self.fault)
            self.visual = dict(desired=desired, center=center, direction=direction,
                               source_ns=source, received=min(row[0] for row in pair))
            self.last_measured = source
            self.samples += 1
            self.visual_reason = 'metric visual geometry'
        except (TransformException, ValueError) as error:
            self.visual_reason = str(error)
