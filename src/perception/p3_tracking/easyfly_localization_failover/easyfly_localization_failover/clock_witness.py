# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Read-only native SITL TIMESYNC witness; never supplies an estimator measurement.

PX4 1.16.2 substitutes receipt time until its 500-sample filter converges. Counting
consecutive original uORB observations prevents silently accepting that fallback.
The observer requires the exact local native process, rootfs and binary identity.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

COMMIT = '54f0455ffcd755534539a7cf33a09a20bf71d29d'


def parse(text):
    values = {}
    for key in ['timestamp', 'remote_timestamp', 'observed_offset', 'estimated_offset', 'round_trip_time', 'source_protocol']:
        matches = re.findall(r'^\s*'+key+r':\s*(-?\d+)(?:\s|$)', text, flags=re.M)
        if len(matches) != 1:
            raise ValueError('missing or ambiguous native TIMESYNC field: '+key)
        values[key] = int(matches[0])
    return values


class Qualification:
    def __init__(self):
        self.count = 0
        self.last = None
        self.last_receipt = float('-inf')
        self.reason = 'waiting_for_native_timesync'
        self.observation = None
        self.rejected = None
        self.rejected_receipt = float('-inf')

    def reset(self, reason):
        self.count = 0
        self.rejected = None
        self.reason = reason

    def update(self, row, now, clock):
        self.observation = dict(native=dict(row), receipt=now, clock_ns=clock)
        # Native IPC runs synchronously while ROS clock callbacks are queued.
        # Defer a bounded future snapshot until the original /clock catches up;
        # do not count it or renew the last verified receipt. A frozen clock
        # still expires valid() and the manager's independent source deadline.
        ahead = row['timestamp']*1000-clock
        if .02e9 < ahead <= .15e9:
            self.reason = 'native_snapshot_waiting_for_original_ros_clock'
            return False
        if (not 0 < row['timestamp'] or not 0 < row['remote_timestamp'] or
                row['source_protocol'] != 0 or row['round_trip_time'] < 0 or
                abs(row['estimated_offset']) > 20000 or
                not -.02e9 <= clock-row['timestamp']*1000 <= (.5e9 if self.count >= 510 else .25e9)):
            self.reset('native_clock_offset_rtt_or_freshness_outside_profile')
            return False
        if self.last is not None and row['timestamp'] == self.last['timestamp']:
            if row != self.last:
                self.reset('conflicting_native_timesync_snapshot')
            return False
        if self.rejected is not None and row['timestamp'] == self.rejected['timestamp']:
            if row != self.rejected:
                self.reset('conflicting_native_rejected_snapshot')
            return False
        if row['round_trip_time'] >= 10000:
            # Pinned native Timesync::update ignores high RTT *before* testing
            # offset deviation: it neither adds a sample nor resets its filter.
            # One distinct rejection can attest that the established estimate
            # survived a short delayed exchange. It never counts as convergence
            # or renews last_receipt. Bound the entire interruption from the last
            # accepted sample, with no chaining of rejected/duplicate snapshots.
            if (self.count >= 510 and self.last is not None and self.rejected is None and
                    row['estimated_offset'] == self.last['estimated_offset'] and
                    0 < row['timestamp']-self.last['timestamp'] <= 500000 and
                    0 < row['remote_timestamp']-self.last['remote_timestamp'] <= 500000 and
                    row['round_trip_time'] <= 350000 and now-self.last_receipt <= .8):
                self.rejected, self.rejected_receipt = dict(row), now
                self.reason = 'native_filter_unchanged_after_one_rejected_rtt'
            else:
                self.reset('native_rejected_rtt_not_proven_bounded_and_unchanged')
            return False
        if self.count >= 510 and self.last is not None:
            # After native convergence, sync_stamp() uses estimated_offset,
            # not the latest raw observation. The native low-RTT branch accepts
            # deviations up to 100 ms and smooths them. Preserve its proven
            # convergence while requiring the actual estimate to remain within
            # 20 ms (above), change by at most 1 ms, and keep observed cadence.
            # Larger native deviations can reset the native filter and are
            # rejected here immediately rather than trying to infer that count.
            bad_offset = (abs(row['observed_offset']-self.last['estimated_offset']) > 100000 or
                          abs(row['estimated_offset']-self.last['estimated_offset']) > 1000)
        else:
            bad_offset = (abs(row['observed_offset']) > 20000 or
                          abs(row['observed_offset']-row['estimated_offset']) > 20000)
        if bad_offset:
            self.reset('native_clock_offset_rtt_or_freshness_outside_profile')
            return False
        recovered = bool(self.rejected is not None and self.last is not None and
                         self.rejected['timestamp'] < row['timestamp'] and
                         self.rejected['remote_timestamp'] < row['remote_timestamp'] and
                         50000 <= row['timestamp']-self.last['timestamp'] <= 500000 and
                         50000 <= row['remote_timestamp']-self.last['remote_timestamp'] <= 500000 and
                         now-self.last_receipt <= .8)
        if (self.last is not None and
                not recovered and
                (not 50000 <= row['timestamp']-self.last['timestamp'] <= 150000 or
                 not 50000 <= row['remote_timestamp']-self.last['remote_timestamp'] <= 150000 or
                 now-self.last_receipt > .35)):
            self.count = 0
        self.last, self.last_receipt = dict(row), now
        self.rejected = None
        self.count += 1
        self.reason = 'native_timesync_verified' if self.count >= 510 else 'collecting_510_native_timesync_observations'
        return True

    def valid(self, now):
        # A native snapshot is the filter state, not a new measurement. PX4
        # preserves its converged filter when no exchange arrives. Bound that
        # unchanged state to 0.5 simulation seconds AND 0.8 steady seconds from
        # the last accepted exchange. Re-reading it never renews either bound.
        # Sensor/EV freshness and the manager's live-witness deadline remain
        # independently enforced. Startup and unobserved good-sample gaps still
        # require consecutive convergence; only an observed RTT rejection can
        # bridge the bounded exchange interruption.
        source_age = (self.observation['clock_ns']-self.last['timestamp']*1000
                      if self.last is not None and self.observation else float('inf'))
        return (self.count >= 510 and 0 <= now-self.last_receipt <= .8 and
                -.02e9 <= source_age <= .5e9)


class Witness(Node):
    def __init__(self):
        super().__init__('sitl_clock_witness')
        fixed = ParameterDescriptor(read_only=True)
        self.pid = self.declare_parameter('px4_pid', 0, fixed).value
        self.binary = Path(self.declare_parameter('px4_binary', '', fixed).value).resolve(strict=True)
        self.rootfs = Path(self.declare_parameter('px4_rootfs', '', fixed).value).resolve(strict=True)
        if not self.get_parameter('use_sim_time').value or self.pid <= 1:
            raise ValueError('explicit local native PX4 identity and simulation clock required')
        self.started = self.attest()
        self.client = self.binary.parent/'px4-listener'
        version = self.binary.parent/'px4-ver'
        if self.client.resolve(strict=True) != self.binary or version.resolve(strict=True) != self.binary:
            raise ValueError('native IPC clients must resolve to the exact owned binary')
        text = subprocess.check_output([str(version), '--instance', '0', 'all'], cwd=self.rootfs, text=True, timeout=3.)
        if COMMIT not in text or 'Release 1.16.2' not in text:
            raise ValueError('unreviewed native PX4 version')
        self.identity = dict(pid=self.pid, start_ticks=self.started, binary=str(self.binary),
                             rootfs=str(self.rootfs), sha256=hashlib.sha256(self.binary.read_bytes()).hexdigest(),
                             version=text, session=uuid.uuid4().hex)
        self.publisher = self.create_publisher(String, '/localization/failover/clock_status', 10)
        self.quality = Qualification()
        self.failure = ''
        self.create_timer(.04, self.observe, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def attest(self):
        process = Path('/proc')/str(self.pid)
        if process.stat().st_uid != os.getuid() or (process/'exe').resolve(strict=True) != self.binary or \
                (process/'cwd').resolve(strict=True) != self.rootfs:
            raise ValueError('native process owner/binary/rootfs identity mismatch')
        arguments = [a.decode() for a in (process/'cmdline').read_bytes().split(b'\0') if a]
        if '-i' in arguments and arguments[arguments.index('-i')+1] != '0':
            raise ValueError('only the explicit single native instance 0 is reviewed')
        return (process/'stat').read_text().rsplit(')', 1)[1].split()[19]

    def observe(self):
        if not self.failure:
            try:
                if self.attest() != self.started:
                    raise RuntimeError('native process identity changed')
                # With no -i, native `listener -n 1` copies its current uORB
                # snapshot immediately. The explicit-instance branch waits on
                # poll(), which can block on the simulator's native IPC path.
                # Multiple native instances yield duplicate fields and parse()
                # rejects them rather than selecting an arbitrary clock.
                result = subprocess.run([str(self.client), '--instance', '0', 'timesync_status', '-n', '1'],
                                        cwd=self.rootfs, text=True, capture_output=True, timeout=.3)
                if result.returncode != 0:
                    raise ValueError('native TIMESYNC observation unavailable')
                row = parse(result.stdout)
                self.quality.update(row, time.monotonic(), self.get_clock().now().nanoseconds)
            except (subprocess.TimeoutExpired, ValueError) as error:
                self.quality.reset(str(error))
            except (OSError, RuntimeError) as error:
                self.failure = str(error)
        value = dict(identity=self.identity, failure=self.failure, verified=not self.failure and self.quality.valid(time.monotonic()),
                     count=self.quality.count, native=self.quality.last,
                     clock_ns=self.get_clock().now().nanoseconds, reason=self.failure or self.quality.reason,
                     last_observation=self.quality.observation)
        self.publisher.publish(String(data=json.dumps(value)))


def main():
    stopping = False
    def stop(*unused):
        nonlocal stopping
        stopping = True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    node = None
    try:
        node = Witness()
        while not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
