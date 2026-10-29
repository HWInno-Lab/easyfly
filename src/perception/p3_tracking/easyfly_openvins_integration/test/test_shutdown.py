# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Signals during a take must not interrupt conversion or hide real errors."""
import os
import signal

import pytest
import rclpy

from easyfly_openvins_integration.shutdown import (
    init_owned_context, shutdown_requested, spin_owned_executor)


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_signal_finishes_current_take_before_context_shutdown(signum):
    previous = {s: signal.getsignal(s) for s in [signal.SIGINT, signal.SIGTERM]}
    completed = []

    class Executor:
        def spin_once(self, timeout_sec):
            assert timeout_sec <= .1
            os.kill(os.getpid(), signum)
            assert rclpy.ok()
            completed.append('message conversion completed')

    try:
        init_owned_context()
        spin_owned_executor(Executor())
        assert completed == ['message conversion completed']
        assert shutdown_requested()
        assert rclpy.ok()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        for s, handler in previous.items():
            signal.signal(s, handler)


def test_runtime_error_is_not_hidden_by_shutdown_loop():
    previous = {s: signal.getsignal(s) for s in [signal.SIGINT, signal.SIGTERM]}

    class Executor:
        def spin_once(self, timeout_sec):
            raise RuntimeError('real conversion failure')

    try:
        init_owned_context()
        with pytest.raises(RuntimeError, match='real conversion failure'):
            spin_owned_executor(Executor())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        for s, handler in previous.items():
            signal.signal(s, handler)
