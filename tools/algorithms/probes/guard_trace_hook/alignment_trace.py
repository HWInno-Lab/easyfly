# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Bounded opt-in audit of actual reference freshness and existing clock takes."""

import atexit
import json
import os
from pathlib import Path
import time


def install():
    from easyfly_openvins_integration import alignment_node as module
    from rclpy import executors
    from rclpy.time_source import TimeSource
    from clocks import wrap_callback
    records, dropped, context = [], [0], ['outside_callback']

    def append(item):
        if len(records) < 200000:
            records.append(item)
        else:
            dropped[0] += 1

    class Executor(executors.SingleThreadedExecutor):
        def _take_subscription(self, sub):
            begin = time.monotonic_ns()
            with sub.handle:
                pair = sub.handle.take_message(sub.msg_type, sub.raw)
                if pair is not None:
                    message, metadata = pair
                    stamp = 0
                    if hasattr(message, 'header'):
                        stamp = module.stamp_ns(message)
                    elif hasattr(message, 'clock'):
                        stamp = message.clock.sec*1000000000+message.clock.nanosec
                    append(dict(kind='take', topic=sub.topic_name, stamp_ns=stamp,
                                begin_ns=begin, end_ns=time.monotonic_ns(),
                                real_ns=time.time_ns(), **metadata))
                    return message
            return None
    executors.SingleThreadedExecutor = Executor
    TimeSource.clock_callback = wrap_callback(TimeSource.clock_callback, append)
    original_fresh = module.Alignment.fresh

    def fresh(self, stamp, receipt, now):
        result = original_fresh(self, stamp, receipt, now)
        if not result:
            append(dict(kind='not_fresh', callback=context[0], stamp_ns=stamp,
                        clock_ns=self.clock, now=now, receipt=receipt,
                        receipt_age_s=now-receipt, source_age_s=(self.clock-stamp)/1e9,
                        begin_ns=time.monotonic_ns(), failure=self.failure))
        return result
    module.Alignment.fresh = fresh

    def wrap(name):
        original = getattr(module.OpenVinsAlignment, name)

        def callback(self, message=None):
            previous = context[0]
            context[0] = name
            begin = time.monotonic_ns()
            try:
                return original(self) if message is None else original(self, message)
            finally:
                append(dict(kind=name, begin_ns=begin, end_ns=time.monotonic_ns(),
                            clock_ns=self.engine.clock, failure=self.engine.failure,
                            reason=self.engine.reason))
                context[0] = previous
        setattr(module.OpenVinsAlignment, name, callback)
    for name in ('on_reference', 'on_attitude', 'on_health', 'report'):
        wrap(name)

    def save():
        prefix = os.environ['EASYFLY_GUARD_TAKE_TRACE'].replace('guard-take', 'alignment-take')
        Path(prefix+f'.{os.getpid()}.json').write_text(json.dumps(
            dict(capacity=200000, discarded=dropped[0], records=records)))
    atexit.register(save)
