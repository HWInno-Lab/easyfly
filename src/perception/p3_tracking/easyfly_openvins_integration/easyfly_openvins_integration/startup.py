# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Bounded, observation-only readiness before the single localization session."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .gate import Gate
from .node import OpenVinsGuard
from .shutdown import init_owned_context


class Readiness:
    """Steady-time windows; no data rewriting and no running-session recovery."""

    def __init__(self, now, minimum=14., quiet=5., healthy=2., timeout=60.):
        if not 0 <= minimum < timeout or not 0 < quiet < timeout or \
                not 0 < healthy < timeout:
            raise ValueError('invalid bounded readiness windows')
        self.started = self.graph_changed = now
        self.minimum, self.quiet, self.healthy, self.timeout = minimum, quiet, healthy, timeout
        self.graph = None
        self.healthy_since = None

    def observe(self, now, graph, healthy):
        if graph != self.graph:
            self.graph, self.graph_changed = graph, now
        if not healthy:
            self.healthy_since = None
        elif self.healthy_since is None:
            self.healthy_since = now
        if now - self.started >= self.timeout:
            return 'timeout'
        if now - self.started >= self.minimum and now - self.graph_changed >= self.quiet and \
                self.healthy_since is not None and now - self.healthy_since >= self.healthy:
            return 'ready'
        return 'waiting'


class StartupObserver(OpenVinsGuard):
    def __init__(self):
        super().__init__(observe_only=True)
        self.readiness = Readiness(time.monotonic())
        self.result = None
        self.unstable_windows = 0
        self.previous_state = None

    def report(self):
        self.update_clock()
        now = time.monotonic()
        state, reason = self.gate.status(now)
        if state != self.previous_state:
            self.get_logger().info(f'startup observation state={state}: {reason}')
            self.previous_state = state
        healthy = state == Gate.INITIALIZING
        # A permanent profile failure cannot become ready by waiting.
        if state == Gate.FAILED:
            self.result = 'invalid_profile'
        elif self.gate.failure is not None:
            self.unstable_windows += 1
            self.get_logger().warning(
                f'startup window rejected #{self.unstable_windows}: '
                f'{reason}; {self.gate.failure_detail}')
            limits, clock = self.gate.limits, self.gate.clock
            self.gate = Gate(limits)
            self.gate.clock_update(clock)
        graph = tuple(sorted(self.get_node_names_and_namespaces()))
        if self.result is None:
            outcome = self.readiness.observe(now, graph, healthy)
            if outcome != 'waiting':
                self.result = outcome
        if self.result is not None:
            self.get_logger().info(
                f'startup result={self.result}; elapsed={now-self.readiness.started:.3f}s; '
                f'rejected_windows={self.unstable_windows}; '
                'no sensor outputs published; live guard not started yet')


def main():
    init_owned_context()
    node = None
    executor = SingleThreadedExecutor()
    code = 2
    try:
        node = StartupObserver()
        executor.add_node(node)
        while node.result is None:
            executor.spin_once(timeout_sec=.1)
        code = 0 if node.result == 'ready' else 2
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # An interrupted readiness check must never start the estimator.
        code = 130
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
