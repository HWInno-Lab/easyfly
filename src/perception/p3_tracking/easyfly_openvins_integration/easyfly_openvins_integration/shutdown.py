# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Keep ROS handles valid until the owning executor has stopped taking messages."""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def init_owned_context():
    # The default native signal handler invalidates the context asynchronously,
    # racing a Python executor's in-flight take/conversion. Unwind its owning
    # thread first; main's finally block then destroys nodes and the context.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
