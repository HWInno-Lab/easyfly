# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
import hashlib
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from easyfly_lidar_atlas.engine import Engine
from easyfly_lidar_atlas.geometry import GeometryRejected, apply, inverse, pose_distance
from test_geometry import room


def observed(engine, pose, scan, seconds):
    original = scan.copy()
    digest = hashlib.sha256(scan.tobytes()).hexdigest()
    result = engine.process(scan, pose, int(seconds*1e9), digest)
    np.testing.assert_array_equal(scan, original)
    return result


@pytest.mark.parametrize('backend', ['fast_lio', 'fast_livo2'])
def test_cold_archive_relocalization_uses_independent_geometry_and_isolates_native(tmp_path, backend):
    first = Engine(backend, 'a'*64)
    p0, p1 = np.eye(4), np.eye(4)
    p1[0, 3] = .85
    observed(first, p0, room(3), 1)
    observed(first, p1, room(5), 7)
    destination = tmp_path/'immutable-atlas'
    identity = first.persist(destination)
    saved = {p.name: p.read_bytes() for p in destination.iterdir()}
    second = Engine(backend, 'a'*64, atlas_path=destination)
    # A different local odometry gauge; original physical room stays fixed.
    local_to_world = np.eye(4)
    local_to_world[:3, :3] = Rotation.from_euler('z', .3).as_matrix()
    local_to_world[:3, 3] = [4., -3., .5]
    expected = np.eye(4)
    expected[:3, 3] = [.2, -.1, .05]
    native = inverse(local_to_world) @ expected
    local_scan = apply(inverse(local_to_world), room(29))
    actual, keyframe = observed(second, native, local_scan, 2)
    assert keyframe and pose_distance(actual, expected)[0] < .03
    assert pose_distance(actual, expected)[1] < .015
    assert second.status()['relocalizations'] == 1 and second.status()['old_map_identity'] == identity
    np.testing.assert_array_equal(second.archive.keyframes[-1].native_pose, native)
    assert second.archive.edges[-1].loop
    assert {p.name: p.read_bytes() for p in destination.iterdir()} == saved
    with pytest.raises(GeometryRejected, match='two native session'):
        second.persist(tmp_path/'too-early')
    with pytest.raises(GeometryRejected, match='regressing'):
        observed(second, native, local_scan, 1)


def test_independent_revisited_scans_produce_real_loop_and_correct_drift():
    engine = Engine('fast_lio', 'b'*64)
    originals = []
    for i, theta in enumerate(np.linspace(0, 2*np.pi, 17)):
        truth = np.eye(4)
        truth[:3, 3] = [1.2*np.sin(theta), 1.2*(1-np.cos(theta)), .4]
        native = truth.copy()
        native[:3, 3] += [i*.025, i*.008, i*.002]
        # The actual scan is measured relative to the physical sensor, then
        # expressed in the drifting native world as a real frontend does.
        scan = apply(native @ inverse(truth), room(50+i))
        observed(engine, native, scan, 1+i*6)
        originals.append(native.copy())
    assert engine.loops >= 1
    assert pose_distance(engine.archive.keyframes[-1].atlas_pose, truth)[0] < .12
    for key, native in zip(engine.archive.keyframes, originals):
        np.testing.assert_array_equal(key.native_pose, native)
    assert engine.last_evidence['kind'] == 'geometric_loop_optimized'


def test_no_old_map_match_never_fabricates_pose_or_mutates_archive(tmp_path):
    first = Engine('fast_lio', 'c'*64)
    observed(first, np.eye(4), room(6), 1)
    observed(first, np.eye(4), room(7), 7)
    first.persist(tmp_path/'map')
    loaded = Engine('fast_lio', 'c'*64, atlas_path=tmp_path/'map')
    with pytest.raises(GeometryRejected):
        observed(loaded, np.eye(4), room(20)*5, 2)
    assert loaded.correction is None and len(loaded.archive.keyframes) == 2
    assert loaded.relocalizations == 0 and loaded.epoch == 0


def test_full_native_imu_scan_is_not_transformed_twice():
    from easyfly_lidar_atlas.geometry import voxel_sample
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('xyz', [.03,-.02,.7]).as_matrix()
    pose[:3, 3] = [2.,-1.,.6]
    scan = apply(inverse(pose), room(81))
    original = scan.copy()
    engine = Engine('fast_lio', 'd'*64)
    result, inserted = engine.process(scan, pose, 1_000_000_000,
        hashlib.sha256(scan.tobytes()).hexdigest(), scan_is_imu=True)
    assert inserted
    np.testing.assert_array_equal(engine.archive.keyframes[0].scan, voxel_sample(original))
    np.testing.assert_array_equal(scan, original)
    np.testing.assert_array_equal(result, pose)
