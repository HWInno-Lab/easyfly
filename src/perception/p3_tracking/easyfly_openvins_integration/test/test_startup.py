# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Readiness never admits an unstable window or bypasses its deadline."""
from easyfly_openvins_integration.startup import Readiness


def test_quiet_graph_and_continuous_inputs_after_minimum_observation():
    check = Readiness(0)
    assert check.observe(0, ('camera',), True) == 'waiting'
    assert check.observe(10, ('camera',), True) == 'waiting'
    assert check.observe(12, ('camera', 'detector'), True) == 'waiting'
    assert check.observe(16, ('camera', 'detector'), True) == 'waiting'
    assert check.observe(17, ('camera', 'detector'), False) == 'waiting'
    assert check.observe(18, ('camera', 'detector'), True) == 'waiting'
    assert check.observe(19.99, ('camera', 'detector'), True) == 'waiting'
    assert check.observe(20, ('camera', 'detector'), True) == 'ready'


def test_deadline_wins_even_if_input_becomes_ready_at_deadline():
    check = Readiness(0)
    assert check.observe(0, (), False) == 'waiting'
    assert check.observe(58, (), True) == 'waiting'
    assert check.observe(60, (), True) == 'timeout'


def test_continuing_graph_changes_or_bad_inputs_cannot_start_session():
    check = Readiness(0)
    for tick in range(60):
        assert check.observe(tick, (tick,), True) == 'waiting'
    assert check.observe(60, (60,), True) == 'timeout'
    check = Readiness(0)
    for tick in range(60):
        assert check.observe(tick, (), False) == 'waiting'
    assert check.observe(60, (), False) == 'timeout'
