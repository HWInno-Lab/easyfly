# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Contract tests for coordinate continuity, loss, recovery and immutable gauges."""
import copy
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from easyfly_localization_failover.policy import Policy, Sample, at


def pose(t, name='reference', moving=False, offset=0.):
    t = round(float(t), 9)
    p = np.array([.2*math.sin(t) if moving else 0., offset, .1])
    q = Rotation.from_euler('z', .12).as_quat()
    if name == 'openvins':
        rotation = Rotation.from_euler('z', -.3)
        p = rotation.apply(p-np.array([2., -3., .6]))
        q = (rotation*Rotation.from_quat(q)).as_quat()
    return Sample.make(round(t*1e9), t, p, q)


def prepared():
    engine, refs = Policy(), []
    for name in engine.names:
        engine.identity(name, name+'-session-1')
    for t in np.arange(1., 4.01, .05):
        engine.clock_update(round(t*1e9))
        refs.append(pose(t))
        for name in engine.names:
            engine.ingest(name, pose(t, name))
    engine.activate(refs, 4., int(1e9), {n:True for n in engine.names})
    return engine


def run(engine, begin, end, absent=(), moving=False, offset=None):
    result = []
    for t in np.arange(begin, end+.001, .05):
        t = round(float(t), 9)
        ns = round(t*1e9); engine.clock_update(ns)
        for name in engine.names:
            if name not in absent:
                engine.ingest(name, pose(t, name, moving, (offset or {}).get(name, 0.)))
        value = engine.step(t, {n:True for n in engine.names})
        if value:
            result.append((engine.selected, value))
    return result


def ready():
    engine = prepared(); run(engine, 4.05, 6.5)
    assert engine.selected == 'fast_lio'
    return engine


def test_frozen_ground_gauges_and_no_reference_refit():
    engine = ready(); frozen = copy.deepcopy(engine.activation)
    rows = run(engine, 6.55, 9., moving=True)
    assert len(rows) > 40
    for _, value in rows:
        assert np.linalg.norm(value.p-pose(value.stamp/1e9, moving=True).p) < 1e-10
    assert engine.activation == frozen
    with pytest.raises(ValueError, match='one-shot'):
        engine.activate([], 9., 1, {})


def test_actual_measurement_only_no_timer_republish():
    engine = ready(); last = engine.last_output
    for t in [6.51, 6.52, 6.53]:
        engine.clock_update(round(t*1e9))
        assert engine.step(t, {n:True for n in engine.names}) is None
    assert engine.last_output == last


def test_failover_recovery_hysteresis_and_monotonic_output():
    engine = ready()
    first = run(engine, 6.55, 7.15, absent=('fast_lio',), moving=True)
    assert engine.selected == 'openvins'
    switch = engine.switched
    second = run(engine, 7.2, 9.3, moving=True)
    assert engine.selected == 'openvins'  # qualified, but residence not yet complete
    third = run(engine, 9.35, 10.2, moving=True)
    assert engine.selected == 'fast_lio'
    assert engine.switched-switch >= 3e9
    stamps = [v.stamp for _, v in first+second+third]
    assert all(a < b for a, b in zip(stamps, stamps[1:]))
    assert all(np.linalg.norm(v.p-pose(v.stamp/1e9, moving=True).p) < 1e-10 for _, v in first+second+third)


def test_both_lost_needs_new_joint_qualification():
    engine = ready()
    run(engine, 6.55, 7.15, absent=engine.names)
    assert engine.selected is None
    assert not run(engine, 7.2, 8.95)
    assert run(engine, 9., 9.5)


def test_wall_clock_expiry_when_ros_clock_frozen():
    engine = ready()
    engine.step(7.3, {n:True for n in engine.names})
    assert engine.selected is None


def test_stale_catchup_never_rejuvenates_source():
    engine = ready(); engine.clock_update(int(7e9))
    assert not engine.ingest('fast_lio', pose(6.6))
    assert engine.sources['fast_lio'].samples[-1].stamp == int(6.5e9)
    assert engine.sources['fast_lio'].rejected_stale == 1
    assert engine.ingest('fast_lio', pose(7.))


@pytest.mark.parametrize('fault', ['identity', 'rollback', 'jump'])
def test_reset_source_quarantined_and_cannot_auto_restore(fault):
    engine = ready()
    if fault == 'identity':
        engine.identity('fast_lio', 'new-native-process')
    elif fault == 'rollback':
        engine.ingest('fast_lio', pose(6.45))
    else:
        engine.clock_update(int(6.55e9)); engine.ingest('fast_lio', pose(6.55, offset=10.))
    assert engine.sources['fast_lio'].failure
    run(engine, 6.6, 7.)
    assert engine.selected == 'openvins'
    assert engine.sources['fast_lio'].failure


def test_independent_disagreement_stops_both_not_arbitrary_selection():
    engine = ready()
    run(engine, 6.55, 7., offset={'openvins':.21})
    assert 'disagree' in engine.failure
    assert engine.selected is None


def test_solo_source_has_bounded_horizon():
    engine = ready()
    run(engine, 6.55, 12., absent=('openvins',))
    assert engine.selected is None
    assert not engine.failure


def test_clock_rollback_latches_whole_session():
    engine = ready(); engine.clock_update(int(6e9))
    assert engine.failure == 'simulation_clock_rollback'
    assert not run(engine, 6.55, 7.)


def test_brackets_never_extrapolate_or_bridge_large_gap():
    history = [pose(1.), pose(1.1)]
    assert at(history, int(1.05e9)) is not None
    assert at(history, int(.9e9)) is None
    assert at(history, int(1.2e9)) is None
    assert at([pose(1.), pose(2.)], int(1.5e9)) is None


@pytest.mark.parametrize('p,q', [([float('nan'), 0, 0], [0, 0, 0, 1]),
                                ([0, 0, 0], [0, 0, 0, 0]),
                                ([0, 0, 0], [0, 0, 0, 2])])
def test_malformed_pose_rejected(p, q):
    with pytest.raises(ValueError):
        Sample.make(1000000, 1., p, q)


def test_unqualified_backup_never_selected():
    engine = prepared()
    run(engine, 4.05, 4.3)
    assert engine.selected is None
    assert not run(engine, 4.35, 5., absent=('fast_lio',))


def test_source_expiry_precedes_native_half_second_receiver_pause_budget():
    engine = ready()
    engine.step(6.86, {n:True for n in engine.names})
    assert engine.selected is None
    assert 6.86-6.5 < .5
