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

"""ROS image sender and receiver with bounded workers and observable disconnects."""

import http.client
import ipaddress
import json
import socket
import ssl
import threading
import time
from urllib.parse import urlsplit

import cv2
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from easyfly_video_recorder.storage import image_bgr, stamp_ns
from .transport import LatestFrame, read_frame, read_token, StreamServer


def options(node, defaults):
    """Read launch-time configuration without silently changing it at runtime."""
    for name, value in defaults.items():
        node.declare_parameter(name, value)
    return {name: node.get_parameter(name).value for name in defaults}


class Sender(Node):
    """Encode outside the executor; retain at most one waiting source image."""

    def __init__(self):
        super().__init__('video_sender')
        o = self.options = options(self, {
            'image_topic': '/camera/color/image_raw', 'bind': '127.0.0.1', 'port': 8088,
            'fps': 10, 'max_width': 1280, 'max_height': 720, 'jpeg_quality': 80,
            'source_timeout_s': 2.0, 'max_clients': 2, 'bytes_per_second': 2097152,
            'token_file': '', 'certificate': '', 'private_key': ''})
        if (not 0 <= o['port'] <= 65535 or not 1 <= o['fps'] <= 30 or
                not 16 <= o['max_width'] <= 1920 or not 16 <= o['max_height'] <= 1080 or
                not 30 <= o['jpeg_quality'] <= 95 or not .5 <= o['source_timeout_s'] <= 10 or
                not 1 <= o['max_clients'] <= 8 or
                not 65536 <= o['bytes_per_second'] <= 16 * 1024 * 1024):
            raise ValueError('invalid video sender limits')
        self.frames = LatestFrame(o['source_timeout_s'])
        self.server = StreamServer((o['bind'], o['port']), self.frames,
                                   read_token(o['token_file']), o['certificate'],
                                   o['private_key'], o['max_clients'], o['bytes_per_second'])
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.last_sample = 0.
        self.metrics = {'encoded': 0, 'queue_replaced': 0, 'invalid': 0, 'rate_dropped': 0}
        self.status = self.create_publisher(String, '~/status', 5)
        self.create_subscription(Image, o['image_topic'], self.receive, qos_profile_sensor_data)
        self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.encoder = threading.Thread(target=self.encode, name='video-encode')
        self.http = threading.Thread(target=self.server.serve_forever,
                                     kwargs={'poll_interval': .1}, name='video-http')
        self.encoder.start()
        self.http.start()

    def receive(self, msg):
        """Reject malformed or already stale source headers before enqueueing."""
        now = time.monotonic()
        try:
            source = stamp_ns(msg.header.stamp)
            age = (self.get_clock().now().nanoseconds - source) / 1e9
            if (not 0 < msg.width <= 4096 or not 0 < msg.height <= 2160 or
                    len(msg.data) > 32 * 1024 * 1024 or not source or
                    not -.2 <= age <= self.options['source_timeout_s']):
                raise ValueError('invalid or stale input')
        except (ValueError, TypeError):
            self.metrics['invalid'] += 1
            return
        if now - self.last_sample < 1. / self.options['fps']:
            self.metrics['rate_dropped'] += 1
            return
        self.last_sample = now
        with self.condition:
            if self.pending:
                self.metrics['queue_replaced'] += 1
            self.pending = (msg, source, time.monotonic_ns())
            self.condition.notify()

    def encode(self):
        """Compress only fresh images with bounded pixel and encoded-byte sizes."""
        o = self.options
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                msg, source, received = self.pending
                self.pending = None
            try:
                pixels = image_bgr(msg)
                scale = min(1., o['max_width'] / pixels.shape[1],
                            o['max_height'] / pixels.shape[0])
                if scale < 1:
                    size = (max(1, int(pixels.shape[1] * scale)),
                            max(1, int(pixels.shape[0] * scale)))
                    pixels = cv2.resize(pixels, size)
                ok, encoded = cv2.imencode(
                    '.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, o['jpeg_quality']])
                if (not ok or len(encoded) > min(1024 * 1024, o['bytes_per_second'] - 1024) or
                        (time.monotonic_ns() - received) / 1e9 > o['source_timeout_s']):
                    raise ValueError('encoded frame exceeds budget')
                self.frames.offer(encoded.tobytes(), source, msg.header.frame_id, received)
                self.metrics['encoded'] += 1
            except (ValueError, cv2.error, OverflowError):
                self.metrics['invalid'] += 1

    def tick(self):
        """Expose transport and encoding budgets using measured steady-clock age."""
        with self.server.metrics_lock:
            value = dict(self.server.metrics)
        with self.frames.condition:
            frame = self.frames.frame
            fresh = bool(frame and (time.monotonic_ns() - frame.received_ns) / 1e9 <=
                         self.options['source_timeout_s'])
        value.update(self.metrics)
        value.update(phase='streaming' if fresh else 'source_unavailable',
                     port=self.server.server_port, stream_id=self.frames.identity,
                     bytes_per_second_per_client=self.options['bytes_per_second'],
                     max_clients=self.options['max_clients'])
        self.status.publish(String(data=json.dumps(value)))

    def close(self):
        """Stop intake and join actual encoding, listener and client workers."""
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()
        self.frames.close()
        self.server.shutdown()
        self.server.server_close()
        self.http.join()
        self.encoder.join()


class Receiver(Node):
    """Decode a verified stream, retain one output and never repeat stale images."""

    def __init__(self):
        super().__init__('video_receiver')
        o = self.options = options(self, {
            'url': 'http://127.0.0.1:8088/stream.mjpg', 'output_topic': '/stream/image',
            'token_file': '', 'ca_file': '', 'source_timeout_s': 2.0})
        self.url = urlsplit(o['url'])
        u = self.url
        if (u.scheme not in ('http', 'https') or not u.hostname or u.username or u.password or
                u.query or u.fragment or u.path != '/stream.mjpg' or
                not .5 <= o['source_timeout_s'] <= 10):
            raise ValueError('invalid video receiver URL or freshness limit')
        try:
            local = ipaddress.ip_address(u.hostname).is_loopback
        except ValueError:
            local = u.hostname == 'localhost'
        self.token = read_token(o['token_file'])
        if not local and (u.scheme != 'https' or not self.token):
            raise ValueError('remote video requires verified TLS and an explicit bearer token')
        self.tls_context = ssl.create_default_context(cafile=o['ca_file'] or None)
        self.tls_context.minimum_version = ssl.TLSVersion.TLSv1_2
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.connection = None
        self.pending = None
        self.last_received = 0.
        self.phase = 'connecting'
        self.identity = ''
        self.sequence = 0
        self.source = 0
        self.metrics = {'received': 0, 'published': 0, 'reconnects': 0,
                        'invalid': 0, 'queue_replaced': 0}
        self.output = self.create_publisher(Image, o['output_topic'], qos_profile_sensor_data)
        self.status = self.create_publisher(String, '~/status', 5)
        self.create_timer(.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_status = 0.
        self.worker = threading.Thread(target=self.download, name='video-receive')
        self.worker.start()

    def download(self):
        """Reconnect with bounded delay; discard old state at every disconnect."""
        while not self.stop.is_set():
            connection = None
            try:
                cls = (http.client.HTTPSConnection if self.url.scheme == 'https'
                       else http.client.HTTPConnection)
                args = {'context': self.tls_context} if self.url.scheme == 'https' else {}
                connection = cls(self.url.hostname, self.url.port, timeout=1., **args)
                with self.lock:
                    self.connection = connection
                headers = {'Authorization': 'Bearer ' + self.token} if self.token else {}
                connection.request('GET', '/stream.mjpg', headers=headers)
                response = connection.getresponse()
                if response.status != 200 or response.getheader('Content-Type') != (
                        'multipart/x-mixed-replace; boundary=easyfly-frame'):
                    raise ValueError('stream authentication or content type rejected')
                while not self.stop.is_set():
                    frame = read_frame(response)
                    if frame.stream_id == self.identity and (
                            frame.sequence <= self.sequence or frame.source_ns <= self.source):
                        raise ValueError('nonmonotonic source epoch')
                    age = (self.get_clock().now().nanoseconds - frame.source_ns) / 1e9
                    if not -.2 <= age <= self.options['source_timeout_s']:
                        raise ValueError('source image outside freshness window')
                    pixels = cv2.imdecode(
                        np.frombuffer(frame.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if pixels is None:
                        raise ValueError('JPEG decode failed')
                    msg = Image()
                    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(frame.source_ns, 10**9)
                    msg.header.frame_id = frame.frame_id
                    msg.height, msg.width = pixels.shape[:2]
                    msg.step, msg.encoding = msg.width * 3, 'bgr8'
                    msg.data = pixels.tobytes()
                    self.identity, self.sequence, self.source = (
                        frame.stream_id, frame.sequence, frame.source_ns)
                    with self.lock:
                        self.metrics['queue_replaced'] += int(self.pending is not None)
                        self.pending = msg
                        self.last_received = time.monotonic()
                        self.metrics['received'] += 1
                        self.phase = 'receiving'
            except (OSError, ValueError, KeyError, UnicodeError, cv2.error,
                    http.client.HTTPException, OverflowError):
                with self.lock:
                    self.metrics['invalid'] += 1
            finally:
                if connection:
                    connection.close()
                with self.lock:
                    self.connection = None
                    self.pending = None
                    self.phase = 'disconnected'
                    self.metrics['reconnects'] += 1
            self.stop.wait(.5)

    def tick(self):
        """Publish only a newly received, still fresh frame in the ROS executor."""
        now = time.monotonic()
        with self.lock:
            pending, self.pending = self.pending, None
            if now - self.last_received > self.options['source_timeout_s']:
                pending = None
                if self.phase == 'receiving':
                    self.phase = 'source_unavailable'
            value = dict(self.metrics, phase=self.phase, stream_id=self.identity,
                         source_ns=self.source)
        if pending:
            age = (self.get_clock().now().nanoseconds - stamp_ns(pending.header.stamp)) / 1e9
            if -.2 <= age <= self.options['source_timeout_s']:
                self.output.publish(pending)
                self.metrics['published'] += 1
        if now - self.last_status >= .2:
            self.status.publish(String(data=json.dumps(value)))
            self.last_status = now

    def close(self):
        """Interrupt a blocked read and join the owned network worker."""
        self.stop.set()
        with self.lock:
            connection = self.connection
        if connection and connection.sock:
            try:
                connection.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.worker.join()


def main(kind):
    """Run one explicit media role; no flight-control API is exposed."""
    rclpy.init()
    node = None
    try:
        node = Sender() if kind == 'sender' else Receiver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
