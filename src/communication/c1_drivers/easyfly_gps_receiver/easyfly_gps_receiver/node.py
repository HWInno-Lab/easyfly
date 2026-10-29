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

"""Own a serial GPS endpoint and publish fresh fixes plus original UTC time references."""

import fcntl
import json
import math
import os
from pathlib import Path
import select
import stat
import termios
import threading
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import String

from .nmea import Framer, InvalidFix, Pairer


BAUDS = {rate: getattr(termios, 'B' + str(rate))
         for rate in (4800, 9600, 19200, 38400, 57600, 115200, 230400)}


def open_serial(path, baud):
    """Configure one character device for raw 8N1 and exclude a second reader."""
    descriptor = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK | os.O_NOFOLLOW)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISCHR(info.st_mode):
            raise ValueError('GPS input must be a character device')
        fcntl.ioctl(descriptor, termios.TIOCEXCL)
        attributes = termios.tcgetattr(descriptor)
        attributes[0] = attributes[1] = attributes[3] = 0
        attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attributes[4] = attributes[5] = BAUDS[baud]
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
        termios.tcflush(descriptor, termios.TCIFLUSH)
        return descriptor, '%d:%d' % (os.major(info.st_rdev), os.minor(info.st_rdev))
    except Exception:
        os.close(descriptor)
        raise


class Receiver(Node):
    """Bound serial parsing and keep invalidity visible without repeating old GPS positions."""

    def __init__(self):
        super().__init__('gps_receiver')
        defaults = {'device': '', 'baud': 9600, 'frame_id': 'gps_link',
                    'fix_topic': '/gps/fix', 'time_topic': '/gps/time_reference',
                    'source_timeout_s': 1.5, 'max_utc_offset_s': 60.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        o = self.options = {name: self.get_parameter(name).value for name in defaults}
        if (not Path(o['device']).is_absolute() or o['baud'] not in BAUDS or
                not o['frame_id'] or len(o['frame_id']) > 128 or
                not .2 <= o['source_timeout_s'] <= 30 or
                not 1 <= o['max_utc_offset_s'] <= 86400):
            raise ValueError('explicit GPS device and supported receiver limits are required')
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.pending = None
        self.last_fix_at = 0.
        self.phase, self.reason = 'disconnected', 'serial_not_open'
        self.generation, self.device_identity = '', ''
        self.metrics = {'bytes': 0, 'sentences': 0, 'invalid_sentences': 0,
                        'framing_discarded': 0, 'fixes': 0, 'reconnects': 0, 'queue_replaced': 0}
        self.fix_pub = self.create_publisher(NavSatFix, o['fix_topic'], 5)
        self.time_pub = self.create_publisher(TimeReference, o['time_topic'], 5)
        self.status = self.create_publisher(String, '~/status', 5)
        self.last_invalid = None
        self.last_status = 0.
        self.last_clock = 0
        self.last_clock_progress = time.monotonic()
        self.details = {}
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.worker = threading.Thread(target=self.read_serial, name='gps-serial')
        self.worker.start()

    def invalidate(self, phase, reason):
        """Clear pending output immediately on explicit loss or invalid receiver status."""
        with self.lock:
            self.phase, self.reason = phase, reason
            self.pending = None
            self.last_fix_at = 0.

    def read_serial(self):
        """Read bounded chunks and reconnect with a fresh parser/time epoch."""
        while not self.stop.is_set():
            descriptor = None
            try:
                descriptor, device = open_serial(self.options['device'], self.options['baud'])
                pairer, framer = Pairer(), Framer()
                with self.lock:
                    self.generation = uuid.uuid4().hex
                    self.device_identity = device
                    self.phase, self.reason = 'waiting_fix', 'waiting_paired_rmc_gga'
                    self.last_fix_at = 0.
                    self.pending = None
                    self.metrics['reconnects'] += 1
                while not self.stop.is_set():
                    if not select.select([descriptor], [], [], .1)[0]:
                        continue
                    data = os.read(descriptor, 512)
                    if not data:
                        raise OSError('GPS serial source disconnected')
                    self.metrics['bytes'] += len(data)
                    for line in framer.feed(data):
                        self.metrics['sentences'] += 1
                        try:
                            observation = pairer.feed(line, time.monotonic())
                            if observation is None:
                                continue
                            offset = (time.time_ns() - observation['utc_ns']) / 1e9
                            if abs(offset) > self.options['max_utc_offset_s']:
                                raise InvalidFix('utc_offset_exceeds_configured_host_budget')
                            observation['utc_offset_s'] = offset
                            with self.lock:
                                self.metrics['queue_replaced'] += int(self.pending is not None)
                                self.pending = observation
                                self.last_fix_at = observation['receipt']
                                self.phase, self.reason = 'fix', ''
                        except InvalidFix as exc:
                            self.metrics['invalid_sentences'] += 1
                            self.invalidate('invalid_fix', str(exc))
                        except (ValueError, IndexError, OverflowError):
                            self.metrics['invalid_sentences'] += 1
                    self.metrics['framing_discarded'] = framer.discarded
            except (OSError, ValueError, termios.error) as exc:
                self.invalidate('disconnected', str(exc))
            finally:
                if descriptor is not None:
                    try:
                        fcntl.ioctl(descriptor, termios.TIOCNXCL)
                    except OSError:
                        pass
                    os.close(descriptor)
            self.stop.wait(.5)

    def tick(self):
        """Publish each accepted measurement once and emit explicit NO_FIX on loss."""
        now = time.monotonic()
        with self.lock:
            pending, self.pending = self.pending, None
            if self.phase == 'fix' and now - self.last_fix_at > self.options['source_timeout_s']:
                self.phase, self.reason = 'source_timeout', 'paired_source_timeout'
                pending = None
            phase, reason, generation = self.phase, self.reason, self.generation
            value = dict(self.metrics, phase=phase, reason=reason, generation=generation,
                         device_identity=self.device_identity)
        clock = self.get_clock().now().nanoseconds
        if clock != self.last_clock:
            self.last_clock_progress = now
        if (clock <= 0 or clock < self.last_clock or
                now - self.last_clock_progress > self.options['source_timeout_s']):
            self.invalidate('clock_invalid', 'ros_clock_unavailable_or_rewound')
            pending = None
            phase, reason = 'clock_invalid', 'ros_clock_unavailable_or_rewound'
            value.update(phase=phase, reason=reason)
        self.last_clock = clock
        if pending and now - pending['receipt'] <= self.options['source_timeout_s']:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.options['frame_id']
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = pending['service']
            msg.latitude, msg.longitude, msg.altitude = (
                pending['latitude'], pending['longitude'], pending['altitude'])
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            reference = TimeReference()
            reference.header = msg.header
            reference.time_ref.sec, reference.time_ref.nanosec = divmod(pending['utc_ns'], 10**9)
            reference.source = 'NMEA_RMC_UTC'
            self.fix_pub.publish(msg)
            self.time_pub.publish(reference)
            self.metrics['fixes'] += 1
            self.last_invalid = None
            self.details = dict(quality=pending['quality'], satellites=pending['satellites'],
                                hdop=pending['hdop'], utc_ns=pending['utc_ns'],
                                utc_offset_s=pending['utc_offset_s'])
        elif phase != 'fix' and self.last_invalid != (generation, phase, reason):
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.options['frame_id']
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = msg.longitude = msg.altitude = math.nan
            self.fix_pub.publish(msg)
            self.last_invalid = (generation, phase, reason)
        if now - self.last_status >= .2:
            if phase == 'fix':
                value.update(self.details)
            self.status.publish(String(data=json.dumps(value, allow_nan=False)))
            self.last_status = now

    def close(self):
        """Join the reader after its bounded select and restore serial exclusivity."""
        self.stop.set()
        self.worker.join()


def main():
    """Start the explicit serial receiver; never change host time or FCU configuration."""
    rclpy.init()
    node = None
    try:
        node = Receiver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
