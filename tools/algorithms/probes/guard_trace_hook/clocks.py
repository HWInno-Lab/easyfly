# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Observe the existing TimeSource callback; no new subscriptions or time writes."""

import hashlib
import inspect
from pathlib import Path
import time


def qos(profile):
    return dict(history=profile.history.name, depth=profile.depth,
                reliability=profile.reliability.name, durability=profile.durability.name)


def wrap_callback(original, append):
    def callback(self, message):
        begin = time.monotonic_ns()
        before = self._last_time_set.nanoseconds
        try:
            return original(self, message)
        finally:
            append(dict(kind='clock_callback',
                        stamp_ns=message.clock.sec*1000000000+message.clock.nanosec,
                        begin_ns=begin, end_ns=time.monotonic_ns(),
                        previous_clock_ns=before, cached_clock_ns=self._last_time_set.nanoseconds,
                        associated_clock_ns=[c.now().nanoseconds
                                             for c in self._associated_clocks]))
    return callback


def install(guard, append):
    from rclpy.time_source import TimeSource
    source = Path(inspect.getfile(TimeSource))
    audit = dict(time_source_file=str(source),
                 time_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                 endpoint_snapshots=[], reader=None)
    TimeSource.clock_callback = wrap_callback(TimeSource.clock_callback, append)
    original = guard.OpenVinsGuard.report
    next_snapshot = [0.]

    def report(self):
        result = original(self)
        now = time.monotonic()
        if now >= next_snapshot[0] and len(audit['endpoint_snapshots']) < 32:
            next_snapshot[0] = now+10
            begin = time.monotonic_ns()
            sub = self._time_source._clock_sub
            audit['reader'] = qos(sub.qos_profile) if sub else None
            endpoints = []
            for endpoint in self.get_publishers_info_by_topic('/clock'):
                endpoints.append(dict(node_name=endpoint.node_name,
                                      node_namespace=endpoint.node_namespace,
                                      gid=list(endpoint.endpoint_gid),
                                      qos=qos(endpoint.qos_profile)))
            audit['endpoint_snapshots'].append(dict(
                begin_ns=begin, end_ns=time.monotonic_ns(), publishers=endpoints))
        return result
    guard.OpenVinsGuard.report = report
    return audit
