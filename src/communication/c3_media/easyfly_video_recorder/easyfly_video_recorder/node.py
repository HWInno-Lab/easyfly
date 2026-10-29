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

"""Independent ROS image/trajectory recorder with bounded asynchronous storage."""

from collections import deque
import json
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .storage import Session, stamp_ns


class Recorder(Node):
    """Accept explicit start/finish; storage completion is separately observable."""

    def __init__(self):
        super().__init__('video_recorder')
        defaults = {'directory': '', 'image_topic': '/camera/color/image_raw',
                    'odom_topic': '/mavros/local_position/odom', 'fps': 10,
                    'max_width': 1280, 'max_height': 720, 'segment_frames': 300,
                    'byte_limit': 512 * 1024 * 1024, 'reserve_bytes': 256 * 1024 * 1024,
                    'record_bag': False, 'source_timeout_s': 3.0,
                    'auto_start': False, 'report_stdout': False}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.options = {k: self.get_parameter(k).value for k in defaults}
        o = self.options
        if (not 1 <= o['fps'] <= 60 or not 1 <= o['segment_frames'] <= 600 or
                not 1 <= o['max_width'] <= 1920 or not 1 <= o['max_height'] <= 1080 or
                not 1024 * 1024 <= o['byte_limit'] <= 8 * 1024**3 or
                not 0 <= o['reserve_bytes'] <= 1024**4 or
                not 0.5 <= o['source_timeout_s'] <= 30.0):
            raise ValueError('invalid recorder limits')
        if any(not o[k].startswith('/') or o[k].startswith('/_')
               for k in ('image_topic', 'odom_topic')):
            raise ValueError('record source topics must be absolute public topics')
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.commands = deque()
        self.images = deque(maxlen=2)
        self.poses = deque(maxlen=128)
        self.closed = False
        self.token = 0
        self.phase = 'idle'
        self.path = ''
        self.reason = ''
        self.frames = 0
        self.pose_count = 0
        self.counters = {}
        self.last_image = 0.
        self.last_sample = 0.
        self.last_clock = 0
        self.publisher = self.create_publisher(String, '~/status', 5)
        self.create_service(Trigger, '~/start', self.start_recording)
        self.create_service(Trigger, '~/finish', self.finish_recording)
        self.create_service(Trigger, '~/query', self.query)
        self.create_subscription(Image, o['image_topic'], self.image, qos_profile_sensor_data)
        self.create_subscription(Odometry, o['odom_topic'], self.pose, qos_profile_sensor_data)
        self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.worker = threading.Thread(target=self.run_storage, name='record-storage')
        self.worker.start()
        if o['auto_start']:
            self.start_recording(Trigger.Request(), Trigger.Response())

    def state(self):
        """Return an atomic status snapshot without touching disk."""
        with self.lock:
            return {'phase': self.phase, 'session_generation': self.token,
                    'directory': self.path, 'reason': self.reason, 'frames': self.frames,
                    'poses': self.pose_count, 'counters': dict(self.counters),
                    'record_bag': self.options['record_bag'],
                    'clock': ('simulation' if self.get_parameter('use_sim_time').value
                              else 'system')}

    def query(self, request, response):
        """Expose the real completion state through a read-only service."""
        del request
        response.success = True
        response.message = json.dumps(self.state())
        return response

    def start_recording(self, request, response):
        """Queue a new session only after the previous worker finished."""
        del request
        with self.lock:
            if self.phase not in ('idle', 'complete', 'failed') or self.closed:
                response.message = 'recording or finalization is already active'
                return response
            if not self.options['directory']:
                response.message = 'directory parameter is required'
                return response
            self.token += 1
            self.phase, self.path, self.reason = 'starting', '', ''
            self.frames = self.pose_count = 0
            self.counters = {'image_queue_replaced': 0, 'pose_queue_replaced': 0,
                             'rate_dropped': 0, 'invalid_input': 0, 'discarded_on_finish': 0}
            self.images.clear()
            self.poses.clear()
            self.last_image = time.monotonic()
            self.last_sample = 0.
            self.last_clock = 0
            self.commands.append(('start', self.token, 'operator_start', False))
            self.wake.set()
        response.success = True
        response.message = 'start accepted; query status for actual storage readiness'
        return response

    def stop(self, reason, failed=False):
        """Latch finalization and prevent later callbacks from replacing its cause."""
        with self.lock:
            if self.phase not in ('starting', 'recording'):
                return False
            self.phase = 'stopping'
            self.reason = reason
            self.counters['discarded_on_finish'] += len(self.images) + len(self.poses)
            self.images.clear()
            self.poses.clear()
            self.commands.append(('stop', self.token, reason, failed))
            self.wake.set()
            return True

    def finish_recording(self, request, response):
        """Accept operator stop without claiming that files already flushed."""
        del request
        response.success = self.stop('operator_stop')
        response.message = 'finalization requested' if response.success else 'no active recording'
        return response

    def input_valid(self, message):
        """Require fresh source time in the explicitly configured clock domain."""
        try:
            now = self.get_clock().now().nanoseconds
            source = stamp_ns(message.header.stamp)
            valid = now > 0 and source > 0 and -200000000 <= now - source <= 2000000000
        except (ValueError, OverflowError):
            valid = False
        if not valid:
            self.counters['invalid_input'] += 1
        return valid

    def image(self, message):
        """Bound callback work; compression and writes stay on the storage thread."""
        with self.lock:
            if self.phase != 'recording' or not self.input_valid(message):
                return
            now = time.monotonic()
            self.last_image = now
            if now - self.last_sample < 1. / self.options['fps']:
                self.counters['rate_dropped'] += 1
                return
            self.last_sample = now
            if len(self.images) == self.images.maxlen:
                self.counters['image_queue_replaced'] += 1
            self.images.append((self.token, message))
            self.wake.set()

    def pose(self, message):
        """Keep a bounded native trajectory queue separate from sampled images."""
        with self.lock:
            if self.phase != 'recording' or not self.input_valid(message):
                return
            if len(self.poses) == self.poses.maxlen:
                self.counters['pose_queue_replaced'] += 1
            self.poses.append((self.token, message))
            self.wake.set()

    def tick(self):
        """Report loss/clock reset and never restart recording automatically."""
        with self.lock:
            if self.phase == 'recording':
                now = self.get_clock().now().nanoseconds
                if now < self.last_clock:
                    self.stop('clock_moved_backwards', failed=True)
                elif time.monotonic() - self.last_image > self.options['source_timeout_s']:
                    self.stop('image_source_timeout')
                self.last_clock = now
        snapshot = json.dumps(self.state())
        self.publisher.publish(String(data=snapshot))
        if self.options['report_stdout']:
            print(snapshot, flush=True)

    def run_storage(self):
        """Serialize disk ownership; drain accepted controls before later input."""
        session = None
        while True:
            self.wake.wait(.05)
            self.wake.clear()
            with self.lock:
                command = self.commands.popleft() if self.commands else None
                image = self.images.popleft() if self.images and command is None else None
                pose = self.poses.popleft() if self.poses and command is None else None
                closed = self.closed
            try:
                if command:
                    action, generation, reason, failed = command
                    if action == 'start':
                        session = Session(self.options['directory'], self.options)
                        with self.lock:
                            self.path = str(session.path)
                            if self.phase == 'starting':
                                self.phase = 'recording'
                    elif session:
                        result = session.finish(reason, failed, dict(self.counters))
                        session = None
                        with self.lock:
                            self.phase = result['status']
                            self.reason = result['error'] or result['reason']
                if session and image and image[0] == self.token:
                    session.image(image[1], self.options['image_topic'])
                if session and pose and pose[0] == self.token:
                    session.pose(pose[1], self.options['odom_topic'])
                if session:
                    with self.lock:
                        self.frames, self.pose_count = session.frames, session.poses
            except Exception as exc:
                reason = str(exc)
                if session:
                    try:
                        session.finish(reason, True, dict(self.counters))
                    except Exception as final_error:
                        reason += '; finalization: ' + str(final_error)
                session = None
                with self.lock:
                    self.phase, self.reason = 'failed', reason
                    self.images.clear()
                    self.poses.clear()
                    self.commands.clear()
            with self.lock:
                pending = self.commands or self.images or self.poses
                if pending:
                    self.wake.set()
            if closed and not pending:
                if session:
                    try:
                        session.finish('node_shutdown', counters=dict(self.counters))
                    except Exception as exc:
                        with self.lock:
                            self.phase, self.reason = 'failed', str(exc)
                return

    def close(self):
        """Join the actual writer; do not abandon or detach unfinished file I/O."""
        self.stop('node_shutdown')
        with self.lock:
            self.closed = True
        self.wake.set()
        self.worker.join()
        if self.options['report_stdout']:
            print(json.dumps(self.state()), flush=True)


def main():
    """Start the standalone node without auto-starting any recording."""
    rclpy.init()
    node = None
    exit_code = 0
    try:
        node = Recorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            exit_code = int(node.state()['phase'] == 'failed')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code
