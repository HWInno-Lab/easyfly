# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
import time
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from easyfly_lidar_atlas.geometry import (
    Edge, GeometryRejected, Registration, RegistrationTarget, apply, descriptor_proposals,
    inverse, optimize_graph, place_descriptor, pose_distance, unique_registration, voxel_sample,
)


def room(seed):
    rng = np.random.default_rng(seed)
    x, y = rng.uniform(-4, 5, (2, 1600))
    floor = np.column_stack([x, y, np.zeros(len(x))])
    y, z = rng.uniform(-4, 5, 1400), rng.uniform(.1, 3.5, 1400)
    east = np.column_stack([np.full(len(y), 5.), y, z])
    x, z = rng.uniform(-4, 5, 1400), rng.uniform(.1, 3.5, 1400)
    north = np.column_stack([x, np.full(len(x), 5.), z])
    # An asymmetric physical pillar distinguishes otherwise rectangular views.
    theta = rng.uniform(0, 2*np.pi, 900)
    pillar = np.column_stack([1.1+.4*np.cos(theta), -.7+.4*np.sin(theta), rng.uniform(0, 2.8, 900)])
    return np.concatenate([floor, east, north, pillar]) + rng.normal(0, .004, (5300, 3))


def test_independent_measurements_recover_metric_six_dof_and_holdout():
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler('xyz', [.035, -.025, .17]).as_matrix()
    expected[:3, 3] = [.31, -.23, .12]
    target = RegistrationTarget(room(11))
    query = apply(inverse(expected), room(47))
    result = target.register(query, np.eye(4))
    position, angle = pose_distance(expected, result.pose)
    assert position < .025 and angle < .012
    assert result.heldout_samples > 500 and result.rmse < .15 and result.overlap > .9
    assert result.information_eigenvalues[0] > 1e-4


def test_degenerate_wrong_place_and_expired_compute_are_rejected():
    rng = np.random.default_rng(3)
    floor = np.column_stack([rng.uniform(-5, 5, (1600, 2)), np.zeros(1600)])
    with pytest.raises(GeometryRejected, match='unobservable'):
        RegistrationTarget(floor).register(floor, np.eye(4))
    target = RegistrationTarget(room(2))
    with pytest.raises(GeometryRejected, match='overlap'):
        target.register(room(3)+100, np.eye(4))
    with pytest.raises(GeometryRejected, match='deadline'):
        target.register(room(4), np.eye(4), deadline=time.monotonic()-1)


def test_repeated_geometry_cannot_choose_between_distinct_places():
    a, b = np.eye(4), np.eye(4)
    b[0, 3] = 12
    measurements = [Registration(p, .04, .96, .01, np.ones(6), 20, 500) for p in [a, b]]
    with pytest.raises(GeometryRejected, match='ambiguous'):
        unique_registration(measurements)
    b[0, 3] = .05
    assert unique_registration(measurements) is measurements[0]


def test_descriptor_rotation_proposes_only_geometric_checks():
    scan = voxel_sample(room(3))
    query = place_descriptor(scan)
    wrong = place_descriptor(voxel_sample(room(41)*.5))
    rotated = np.roll(query, 7, axis=1)
    best = descriptor_proposals(query, [wrong, rotated])[0]
    assert best[1] == 1 and best[0] == pytest.approx(1.)
    assert best[2] == pytest.approx(7*2*np.pi/60)


def test_actual_odometry_and_loop_measurement_correct_drift_without_moving_gauge():
    truth = []
    for theta in np.linspace(0, 2*np.pi, 17):
        p = np.eye(4)
        p[:3, 3] = [2*np.sin(theta), 2*(1-np.cos(theta)), 0]
        truth.append(p)
    raw = [p.copy() for p in truth]
    for i, p in enumerate(raw):
        p[:3, 3] += [i*.04, i*.012, i*.004]
    edges = [Edge(i-1, i, inverse(raw[i-1]) @ raw[i]) for i in range(1, len(raw))]
    edges.append(Edge(0, len(raw)-1, inverse(truth[0]) @ truth[-1], loop=True))
    corrected, report = optimize_graph(raw, edges)
    np.testing.assert_array_equal(corrected[0], raw[0])
    assert pose_distance(corrected[-1], truth[-1])[0] < .08
    assert report['after_norm'] < report['before_norm']*.4
    np.testing.assert_array_equal(raw[-1][:3, 3], truth[-1][:3, 3]+[.64, .192, .064])
    with pytest.raises(GeometryRejected, match='disconnected'):
        optimize_graph(raw, [edges[0]]*len(raw))
