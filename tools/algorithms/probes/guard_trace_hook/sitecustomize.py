# Copyright 2016 Open Source Robotics Foundation, Inc. (rclpy executor take pattern)
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Opt-in Humble diagnostic: preserve metadata from the executor's existing take.

No extra take, callback reordering, source retimestamp or guard-limit changes.
Activated only for the installed openvins_guard executable with a trace prefix.
"""

import atexit
import hashlib
import inspect
import json
import os
from pathlib import Path
import sys
import time


if Path(sys.argv[0]).name == 'openvins_guard' and os.environ.get('EASYFLY_GUARD_TAKE_TRACE'):
    from easyfly_openvins_integration import node as guard
    original_executor = guard.SingleThreadedExecutor
    source = inspect.getsource(original_executor._take_subscription)
    if 'msg_info = sub.handle.take_message(sub.msg_type, sub.raw)' not in source:
        raise RuntimeError('guard take audit supports the reviewed Humble executor only')
    records = []
    dropped = [0]

    def append(item):
        if len(records) < 200000:
            records.append(item)
        else:
            dropped[0] += 1

    class AuditedExecutor(original_executor):
        def _take_subscription(self, sub):
            begin = time.monotonic_ns()
            with sub.handle:
                pair = sub.handle.take_message(sub.msg_type, sub.raw)
                if pair is not None:
                    message, metadata = pair
                    stamp = guard.stamp_ns(message) if hasattr(message, 'header') else 0
                    if os.environ.get('EASYFLY_GUARD_CLOCK_TRACE') == '1' and \
                            sub.topic_name == '/clock':
                        stamp = message.clock.sec*1000000000+message.clock.nanosec
                    append(dict(kind='take', topic=sub.topic_name, stamp_ns=stamp,
                                begin_ns=begin, end_ns=time.monotonic_ns(),
                                real_ns=time.time_ns(), **metadata))
                    return message
            return None

    guard.SingleThreadedExecutor = AuditedExecutor

    def wrap(name):
        original = getattr(guard.OpenVinsGuard, name)

        def callback(self, message=None):
            begin = time.monotonic_ns()
            stamp = guard.stamp_ns(message) if message is not None else 0
            try:
                return original(self) if message is None else original(self, message)
            finally:
                append(dict(kind=name, stamp_ns=stamp, begin_ns=begin,
                            end_ns=time.monotonic_ns(), imu_ns=self.gate.imu_stamp,
                            info_ns=self.gate.infos[-1] if self.gate.infos else 0,
                            clock_ns=self.gate.clock, pending=len(self.gate.pending),
                            failure=self.gate.failure, reason=self.gate.reason))
        setattr(guard.OpenVinsGuard, name, callback)

    for name in ('on_imu', 'on_info', 'on_camera', 'report'):
        wrap(name)

    if os.environ.get('EASYFLY_GUARD_FRESHNESS_TRACE') == '1':
        from freshness import install
        install(guard, append)

    clock_audit = None
    if os.environ.get('EASYFLY_GUARD_CLOCK_TRACE') == '1':
        from clocks import install as install_clock
        clock_audit = install_clock(guard, append)

    def save():
        prefix = os.environ['EASYFLY_GUARD_TAKE_TRACE']
        Path(prefix + f'.{os.getpid()}.json').write_text(json.dumps(dict(
            scope=__doc__, clock_audit=clock_audit, capacity=200000,
            discarded=dropped[0], records=records,
            freshness_enabled=os.environ.get('EASYFLY_GUARD_FRESHNESS_TRACE') == '1',
            gate_source_sha256=hashlib.sha256(
                Path(inspect.getfile(guard.Gate)).read_bytes()).hexdigest())))

    atexit.register(save)

if Path(sys.argv[0]).name == 'openvins_alignment' and \
        os.environ.get('EASYFLY_GUARD_CLOCK_TRACE') == '1':
    from alignment_trace import install
    install()
