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

"""Single-owner UDP mesh with signed immutable forwarding and source-faithful ROS output."""

from collections import OrderedDict
import json
import secrets
import socket
import time

from easyfly_udp_protocol.odometry import decode_odometry, encode_odometry
from easyfly_udp_protocol.protocol import (BROADCAST, decode, Encoder, MAX_DATAGRAM,
                                           MESH_LINKS, ODOMETRY, PING, PONG, ReplayWindow)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .configuration import load
from .topology import decode_links, encode_links, Topology


class Mesh(Node):
    """Route observations among at most sixteen administrator-pinned identities."""

    def __init__(self):
        super().__init__('mesh_peer')
        self.declare_parameter('config_file', '')
        key, binding, self.trusted, self.direct, outputs, input_topic = load(
            self.get_parameter('config_file').value)
        self.encoder = Encoder(key)
        self.window = ReplayWindow(maximum_peers=15)
        self.topology = Topology(self.encoder.source, self.trusted)
        self.addresses = {address: peer for peer, address in self.direct.items()}
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
        self.socket.bind(binding)
        self.socket.setblocking(False)
        resolved = [self.resolve_topic_name(topic) for topic in [input_topic, *outputs.values()]]
        if len(set(resolved)) != len(resolved):
            self.socket.close()
            raise ValueError('remapped mesh input and output topics must remain distinct')
        self.outputs = {peer: self.create_publisher(Odometry, topic, qos_profile_sensor_data)
                        for peer, topic in outputs.items()}
        self.status = self.create_publisher(String, '~/status', 5)
        self.create_subscription(Odometry, input_topic, self.source, qos_profile_sensor_data)
        self.pings = OrderedDict()
        self.pongs, self.rtt, self.last_sources, self.last_outputs = {}, {}, {}, {}
        self.routes = {}
        self.last_source = self.last_clock = 0
        self.last_sent = self.next_ping = self.next_links = self.next_status = 0.
        self.clock_at = self.budget_at = time.monotonic()
        self.clock_fault = False
        self.budget = 300.
        self.network_error = ''
        self.metrics = {'sent': 0, 'bytes_sent': 0, 'send_errors': 0, 'received': 0,
                        'invalid': 0, 'rate_dropped': 0, 'forwarded': 0,
                        'published': 0, 'input_rejected': 0, 'no_route': 0}
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def raw_send(self, data, peer):
        """Send a bounded immutable packet without a hidden retransmit queue."""
        try:
            count = self.socket.sendto(data, self.direct[peer])
            self.metrics['sent'] += 1
            self.metrics['bytes_sent'] += count
        except OSError:
            self.metrics['send_errors'] += 1

    def send(self, kind, peer, payload):
        """Sign direct heartbeat traffic with the local process session."""
        self.raw_send(self.encoder.encode(kind, peer, payload), peer)

    def active_neighbors(self, now):
        """Require a recent response to an owned random challenge for each active link."""
        return {peer for peer, received in self.pongs.items() if now - received <= 1.5}

    def clock_valid(self, now):
        """Separate advancing ROS source time from transport wall time."""
        return self.last_clock > 0 and not self.clock_fault and now - self.clock_at <= 1.

    def source(self, message):
        """Fan out each new source sample once along currently verified routes."""
        now = time.monotonic()
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if not self.clock_valid(now) or stamp <= self.last_source or not -.2 <= age <= 1.:
            self.metrics['input_rejected'] += 1
            return
        if now - self.last_sent < .05:
            return
        try:
            payload = encode_odometry(message)
        except (ValueError, OverflowError):
            self.metrics['input_rejected'] += 1
            return
        self.last_source, self.last_sent = stamp, now
        self.routes = self.topology.routes(self.active_neighbors(now), now)
        for peer in self.trusted:
            route = self.routes.get(peer)
            if route:
                self.raw_send(self.encoder.encode(ODOMETRY, peer, payload, 1500), route[1])
            else:
                self.metrics['no_route'] += 1

    def receive(self, data, address, now):
        """Authenticate every hop input and every original source before forwarding unchanged."""
        previous = self.addresses.get(address)
        if previous is None:
            raise ValueError('unconfigured previous-hop endpoint')
        packet = decode(data, self.trusted)
        if (packet.destination not in self.trusted and
                packet.destination not in (self.encoder.source, BROADCAST)):
            raise ValueError('unknown mesh destination')
        if packet.kind in (PING, PONG):
            if (packet.source != previous or packet.destination != self.encoder.source or
                    len(packet.payload) != 8):
                raise ValueError('heartbeats must be direct and correctly addressed')
        elif packet.kind == MESH_LINKS:
            if packet.destination != BROADCAST:
                raise ValueError('link advertisements must be broadcast')
        elif packet.kind != ODOMETRY:
            raise ValueError('unsupported mesh message kind')
        self.window.accept(packet)
        if packet.kind == PING:
            self.send(PONG, packet.source, packet.payload)
        elif packet.kind == PONG:
            sent = self.pings.pop((packet.source, packet.payload), None)
            if sent is None or now - sent > 1.5:
                raise ValueError('no matching live challenge')
            self.pongs[packet.source] = now
            self.rtt[packet.source] = (now - sent) * 1000
        elif packet.kind == MESH_LINKS:
            neighbors = decode_links(packet.payload, packet.source, self.topology.members)
            remaining = (packet.issued_ns + packet.ttl_ms * 10**6 - time.time_ns()) / 1e9
            self.topology.update(packet.source, neighbors, now + max(0., min(1.5, remaining)))
            for peer in self.active_neighbors(now) - {previous, packet.source}:
                self.raw_send(data, peer)
                self.metrics['forwarded'] += 1
        elif packet.destination == self.encoder.source:
            message = decode_odometry(packet.payload)
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            prior = self.last_sources.get(packet.source)
            frames = (message.header.frame_id, message.child_frame_id)
            if (not self.clock_valid(now) or not -.2 <= age <= 1. or
                    (prior and prior[0] == packet.session and
                     (stamp <= prior[1] or frames != prior[2]))):
                raise ValueError('invalid native odometry source freshness or frame')
            self.last_sources[packet.source] = (packet.session, stamp, frames)
            self.last_outputs[packet.source] = now
            self.outputs[packet.source].publish(message)
            self.metrics['published'] += 1
        else:
            routes = self.topology.routes(self.active_neighbors(now), now)
            path = routes.get(packet.destination)
            if path and path[1] != previous:
                self.raw_send(data, path[1])
                self.metrics['forwarded'] += 1
            else:
                self.metrics['no_route'] += 1
        self.metrics['received'] += 1

    def tick(self):
        """Bound receive work and expire routes independently of frozen ROS time."""
        now = time.monotonic()
        clock = self.get_clock().now().nanoseconds
        if clock < self.last_clock:
            self.clock_fault = True
        if clock != self.last_clock:
            self.clock_at = now
        self.last_clock = clock
        self.budget = min(300., self.budget + (now - self.budget_at) * 300.)
        self.budget_at = now
        for _ in range(32):
            try:
                data, address = self.socket.recvfrom(MAX_DATAGRAM + 1)
            except BlockingIOError:
                break
            except OSError as exc:
                self.network_error = str(exc)
                break
            if self.budget < 1:
                self.metrics['rate_dropped'] += 1
                continue
            self.budget -= 1
            try:
                self.receive(data, address, now)
            except (ValueError, UnicodeError, OverflowError):
                self.metrics['invalid'] += 1
        if now >= self.next_ping:
            for peer in self.direct:
                nonce = secrets.token_bytes(8)
                self.pings[(peer, nonce)] = now
                self.send(PING, peer, nonce)
            while len(self.pings) > 60:
                self.pings.popitem(last=False)
            self.next_ping = now + .5
        active = self.active_neighbors(now)
        if now >= self.next_links:
            data = self.encoder.encode(MESH_LINKS, BROADCAST, encode_links(active), 1500)
            for peer in active:
                self.raw_send(data, peer)
            self.next_links = now + .4
        self.routes = self.topology.routes(active, now)
        if now >= self.next_status:
            value = dict(self.metrics, source_id=self.encoder.source,
                         session=self.encoder.session, phase='online' if active else 'isolated',
                         active_neighbors=sorted(active), routes=self.routes,
                         rtt_ms={peer: self.rtt[peer] for peer in active},
                         odometry_valid={peer: self.clock_valid(now) and
                                         now - self.last_outputs.get(peer, 0.) <= 1.
                                         for peer in self.trusted},
                         clock_reset_requires_restart=self.clock_fault)
            if self.network_error:
                value.update(phase='network_failed', error=self.network_error)
            self.status.publish(String(data=json.dumps(value)))
            self.next_status = now + .2

    def close(self):
        """Release the owned UDP socket without orphaned worker threads."""
        self.socket.close()


def main():
    """Run one mesh identity with an explicitly provisioned trust graph."""
    rclpy.init()
    node = None
    try:
        node = Mesh()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
