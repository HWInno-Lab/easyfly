"""Deliver real signals inside an executor operation, including action drain."""
import os
import signal
from unittest.mock import patch
import pytest
import rclpy
from rclpy.node import Node
from easyfly_fuel_exploration import node as fuel


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('active', [False, True])
def test_signal_preserves_context_through_inflight_wait_and_cleanup(signum, active):
    calls = []

    class OwnedNode(Node):
        def __init__(self):
            super().__init__('fuel_shutdown_probe')
            self.stopping = False
            self.admitted = active

        def destroy_node(self):
            assert rclpy.ok(), 'destroy handles before invalidating context'
            calls.append('destroy')
            return super().destroy_node()

    original = fuel.SingleThreadedExecutor.spin_once

    def spin_once(executor, timeout_sec):
        n = executor.get_nodes()[0]
        calls.append('spin')
        if calls.count('spin') == 1:
            os.kill(os.getpid(), signum)
            os.kill(os.getpid(), signum)
            assert rclpy.ok(), 'signal must not invalidate in-flight WaitSet'
        elif active:
            assert n.stopping and rclpy.ok()
            n.admitted = False  # Existing bounded action cleanup has drained.
            calls.append('drained')
        original(executor, timeout_sec=0.)

    with patch.object(fuel, 'Exploration', OwnedNode), patch.object(
            fuel.SingleThreadedExecutor, 'spin_once', spin_once):
        fuel.main()
    assert calls.count('spin') == (2 if active else 1)
    assert calls[-1] == 'destroy'
    assert ('drained' in calls) == active
    assert not rclpy.ok()
