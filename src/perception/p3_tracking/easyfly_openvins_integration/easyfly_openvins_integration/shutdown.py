# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Keep ROS handles valid until the owning executor has stopped taking messages."""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions

_requested = False


def _interrupt(signum, frame):
    # Raising inside a pybind message conversion can surface as RuntimeError,
    # losing the original KeyboardInterrupt. Finish the in-flight take first.
    global _requested
    _requested = True


def shutdown_requested():
    return _requested


def spin_owned_executor(executor, condition=lambda: True):
    """Stop between executor operations while all ROS handles remain valid."""
    while rclpy.ok() and not _requested and condition():
        executor.spin_once(timeout_sec=.1)


def init_owned_context():
    global _requested
    _requested = False
    # The default native signal handler invalidates the context asynchronously,
    # racing a Python executor's in-flight take/conversion. Let its owning
    # thread finish that take, then destroy nodes and the context in finally.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
