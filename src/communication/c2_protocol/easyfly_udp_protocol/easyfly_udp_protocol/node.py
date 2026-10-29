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

"""Independent signed UDP peer with bounded native odometry and heartbeat transport."""

import argparse
from collections import OrderedDict
import ipaddress
import json
import secrets
import socket
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .odometry import decode_odometry, encode_odometry
from .protocol import (decode, Encoder, generate_keys, identity, MAX_DATAGRAM, ODOMETRY,
                       PING, PONG, read_key, ReplayWindow)


class Peer(Node):
    """Exchange data with one explicit pinned peer; no flight-command endpoint is exposed."""

    def __init__(self):
        super().__init__('udp_peer')
        defaults = {'bind': '127.0.0.1', 'port': 9010, 'peer_host': '127.0.0.1',
                    'peer_port': 9011, 'private_key_file': '', 'peer_public_key_file': '',
                    'input_topic': '/odom', 'output_topic': '/network/remote_odom',
                    'max_hz': 20.0, 'source_timeout_s': 1.0, 'link_timeout_s': 2.0,
                    'ttl_ms': 1500, 'max_datagrams_per_second': 100}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        o = self.options = {name: self.get_parameter(name).value for name in defaults}
        ipaddress.IPv4Address(o['bind'])
        ipaddress.IPv4Address(o['peer_host'])
        if (not 1 <= o['port'] <= 65535 or not 1 <= o['peer_port'] <= 65535 or
                not .1 <= o['max_hz'] <= 60 or not .2 <= o['source_timeout_s'] <= 5 or
                not .5 <= o['link_timeout_s'] <= 10 or not 100 <= o['ttl_ms'] <= 5000 or
                not 10 <= o['max_datagrams_per_second'] <= 1000 or
                o['input_topic'] == o['output_topic']):
            raise ValueError('invalid UDP endpoint, rate, freshness or topic configuration')
        self.encoder = Encoder(read_key(o['private_key_file'], private=True))
        remote = read_key(o['peer_public_key_file'])
        self.peer_id = identity(remote)
        if self.peer_id == self.encoder.source:
            raise ValueError('a UDP peer must have its own distinct signing identity')
        self.trusted = {self.peer_id: remote}
        self.endpoint = (o['peer_host'], o['peer_port'])
        self.window = ReplayWindow(maximum_peers=1)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
        self.socket.bind((o['bind'], o['port']))
        self.socket.settimeout(.1)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.pending = None
        self.pings = OrderedDict()
        self.last_received = self.last_pong = self.last_sent = self.last_output = 0.
        self.last_source = self.last_input = 0
        self.last_peer_epoch = self.last_frame = None
        self.last_clock = 0
        self.clock_at = time.monotonic()
        self.clock_fault = False
        self.rtt_ms = None
        self.network_error = ''
        self.metrics = {'sent': 0, 'received': 0, 'invalid': 0, 'rate_dropped': 0,
                        'input_rejected': 0, 'queue_replaced': 0, 'published': 0,
                        'send_errors': 0, 'bytes_sent': 0, 'bytes_received': 0}
        self.status = self.create_publisher(String, '~/status', 5)
        self.output = self.create_publisher(Odometry, o['output_topic'], qos_profile_sensor_data)
        self.create_subscription(Odometry, o['input_topic'], self.receive_input,
                                 qos_profile_sensor_data)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_status = 0.
        self.worker = threading.Thread(target=self.network, name='signed-udp-peer')
        self.worker.start()

    def send(self, kind, payload):
        """Sign one bounded datagram and account for actual socket transmission."""
        data = self.encoder.encode(kind, self.peer_id, payload, self.options['ttl_ms'])
        try:
            sent = self.socket.sendto(data, self.endpoint)
            with self.lock:
                self.metrics['sent'] += 1
                self.metrics['bytes_sent'] += sent
        except OSError:
            with self.lock:
                self.metrics['send_errors'] += 1

    def receive_input(self, message):
        """Transmit only fresh advancing native source observations at a bounded rate."""
        now = time.monotonic()
        source = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - source) / 1e9
        if (self.clock_fault or source <= self.last_input or
                not -.2 <= age <= self.options['source_timeout_s']):
            self.metrics['input_rejected'] += 1
            return
        if now - self.last_sent < 1. / self.options['max_hz']:
            return
        try:
            data = encode_odometry(message)
        except (ValueError, OverflowError):
            self.metrics['input_rejected'] += 1
            return
        self.last_input = source
        self.last_sent = now
        self.send(ODOMETRY, data)

    def receive_packet(self, data, address):
        """Check endpoint, cryptographic identity, replay state and original source freshness."""
        if address != self.endpoint:
            raise ValueError('datagram came from an unconfigured endpoint')
        packet = decode(data, self.trusted, destination=self.encoder.source)
        if packet.kind not in (ODOMETRY, PING, PONG):
            raise ValueError('message kind is unavailable on a point-to-point peer')
        self.window.accept(packet)
        now = time.monotonic()
        if packet.kind == ODOMETRY:
            message = decode_odometry(packet.payload)
            source = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - source) / 1e9
            frames = (message.header.frame_id, message.child_frame_id)
            if (not -.2 <= age <= self.options['source_timeout_s'] or
                    (packet.session == self.last_peer_epoch and
                     (source <= self.last_source or frames != self.last_frame))):
                raise ValueError('invalid, repeated or changed odometry source within session')
            with self.lock:
                self.metrics['queue_replaced'] += int(self.pending is not None)
                self.pending = (now, packet.session, message)
                self.last_peer_epoch, self.last_source, self.last_frame = (
                    packet.session, source, frames)
        else:
            if len(packet.payload) != 8:
                raise ValueError('invalid heartbeat payload')
            if packet.kind == PING:
                self.send(PONG, packet.payload)
            else:
                with self.lock:
                    sent = self.pings.pop(packet.payload, None)
                    if sent is None:
                        raise ValueError('pong does not match an outstanding owned ping')
                    self.rtt_ms = (now - sent) * 1000
                    self.last_pong = now
        with self.lock:
            self.last_received = now
            self.metrics['received'] += 1
            self.metrics['bytes_received'] += len(data)

    def network(self):
        """Keep network memory and signature verification rate bounded under malformed traffic."""
        next_ping = 0.
        budget_at, budget = time.monotonic(), self.options['max_datagrams_per_second']
        while not self.stop.is_set():
            now = time.monotonic()
            if now >= next_ping:
                nonce = secrets.token_bytes(8)
                with self.lock:
                    self.pings[nonce] = now
                    while len(self.pings) > 4:
                        self.pings.popitem(last=False)
                self.send(PING, nonce)
                next_ping = now + .5
            try:
                data, address = self.socket.recvfrom(MAX_DATAGRAM + 1)
            except socket.timeout:
                continue
            except OSError as exc:
                self.network_error = str(exc)
                return
            now = time.monotonic()
            maximum = self.options['max_datagrams_per_second']
            budget = min(maximum, budget + (now - budget_at) * maximum)
            budget_at = now
            if budget < 1:
                with self.lock:
                    self.metrics['rate_dropped'] += 1
                self.stop.wait(min(.01, (1 - budget) / maximum))
                continue
            budget -= 1
            try:
                self.receive_packet(data, address)
            except (ValueError, UnicodeError, OverflowError):
                with self.lock:
                    self.metrics['invalid'] += 1

    def tick(self):
        """Expire observations independently of link heartbeats and simulation clock progress."""
        now = time.monotonic()
        clock = self.get_clock().now().nanoseconds
        if clock < self.last_clock:
            self.clock_fault = True
        if clock != self.last_clock:
            self.clock_at = now
        self.last_clock = clock
        clock_valid = clock > 0 and now - self.clock_at <= self.options['source_timeout_s']
        with self.lock:
            pending, self.pending = self.pending, None
            seen = now - self.last_received <= self.options['link_timeout_s']
            value = dict(self.metrics, source_id=self.encoder.source, peer_id=self.peer_id,
                         session=self.encoder.session, peer_session=self.last_peer_epoch,
                         phase='receiving' if seen else 'peer_timeout',
                         round_trip_recent=now - self.last_pong <= self.options['link_timeout_s'],
                         rtt_ms=self.rtt_ms, clock_reset_requires_restart=self.clock_fault)
            if self.network_error:
                value.update(phase='network_failed', error=self.network_error)
        if pending and clock_valid and not self.clock_fault:
            received, epoch, message = pending
            del epoch
            source = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            if (now - received <= self.options['source_timeout_s'] and
                    -.2 <= (clock - source) / 1e9 <= self.options['source_timeout_s']):
                self.output.publish(message)
                self.last_output = now
                self.metrics['published'] += 1
        value['odometry_valid'] = bool(
            clock_valid and not self.clock_fault and
            now - self.last_output <= self.options['source_timeout_s'])
        if now - self.last_status >= .2:
            self.status.publish(String(data=json.dumps(value)))
            self.last_status = now

    def close(self):
        """Join the actual bounded receive loop before closing its socket."""
        self.stop.set()
        self.worker.join()
        self.socket.close()


def main():
    """Run one explicitly configured UDP peer with its own pinned identity."""
    rclpy.init()
    node = None
    try:
        node = Peer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def keygen():
    """Create a new identity directory and print only its public identifier."""
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    args = parser.parse_args()
    print(generate_keys(args.directory))
