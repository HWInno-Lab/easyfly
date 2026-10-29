# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
import hashlib
import io
import json
from pathlib import Path

import numpy as np
import pytest

from easyfly_lidar_atlas.archive import Archive, Keyframe, load, save
from easyfly_lidar_atlas.geometry import Edge, GeometryRejected


def fixture():
    random = np.random.default_rng(39)
    a, b = np.eye(4), np.eye(4)
    b[0, 3] = 1.5
    keys = [Keyframe(random.normal(size=(500, 3)).astype('f4'), p, p, (i+1)*10**9,
                     'a'*32, hashlib.sha256(str(i).encode()).hexdigest())
            for i, p in enumerate([a, b])]
    return Archive('fast_lio', 'b'*64, keys, [Edge(0, 1, b)])


def test_roundtrip_preserves_native_measurements_graph_and_never_overwrites(tmp_path):
    source = fixture()
    directory = tmp_path/'map'
    identity = save(directory, source)
    actual = load(directory, 'fast_lio', 'b'*64)
    assert actual.identity == identity
    for old, new in zip(source.keyframes, actual.keyframes):
        np.testing.assert_array_equal(old.scan, new.scan)
        np.testing.assert_array_equal(old.native_pose, new.native_pose)
        np.testing.assert_array_equal(old.atlas_pose, new.atlas_pose)
        assert old.stamp_ns == new.stamp_ns and old.source_sha256 == new.source_sha256
    with pytest.raises(FileExistsError):
        save(directory, source)
    assert load(directory, 'fast_lio', 'b'*64).identity == identity


def test_calibration_backend_bit_corruption_and_incomplete_commit_fail_closed(tmp_path):
    directory = tmp_path/'map'
    save(directory, fixture())
    with pytest.raises(GeometryRejected, match='mismatch'):
        load(directory, 'fast_lio', 'c'*64)
    with pytest.raises(GeometryRejected, match='mismatch'):
        load(directory, 'fast_livo2', 'b'*64)
    path = directory/'scans.npy'
    data = bytearray(path.read_bytes())
    data[-4] ^= 1
    path.write_bytes(data)
    with pytest.raises(GeometryRejected, match='digest'):
        load(directory, 'fast_lio', 'b'*64)
    (directory/'manifest.json').unlink()
    with pytest.raises(FileNotFoundError):
        load(directory, 'fast_lio', 'b'*64)


@pytest.mark.parametrize('kind', ['object', 'huge_shape', 'symlink', 'extra_file'])
def test_untrusted_numeric_payload_cannot_allocate_or_load_objects(tmp_path, kind):
    directory = tmp_path/'map'
    save(directory, fixture())
    path = directory/'scans.npy'
    if kind == 'symlink':
        target = tmp_path/'external.npy'
        path.rename(target)
        path.symlink_to(target)
        with pytest.raises(OSError):
            load(directory, 'fast_lio', 'b'*64)
        return
    if kind == 'extra_file':
        (directory/'unregistered.npy').write_bytes(b'unknown')
        with pytest.raises(GeometryRejected, match='layout'):
            load(directory, 'fast_lio', 'b'*64)
        return
    buffer = io.BytesIO()
    if kind == 'object':
        np.save(buffer, np.array([{'object': 'must never deserialize'}], dtype=object))
    else:
        np.lib.format.write_array_header_1_0(buffer, dict(descr='<f8', fortran_order=False,
                                                        shape=(2**60, 3)))
    data = buffer.getvalue()
    path.write_bytes(data)
    manifest = directory/'manifest.json'
    doc = json.loads(manifest.read_text())
    doc['files']['scans.npy'] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
    manifest.write_text(json.dumps(doc))
    with pytest.raises(GeometryRejected, match='array'):
        load(directory, 'fast_lio', 'b'*64)


def test_archive_rejects_time_regression_and_disconnected_or_duplicate_edges(tmp_path):
    source = fixture()
    source.keyframes[1].stamp_ns = source.keyframes[0].stamp_ns
    with pytest.raises(GeometryRejected, match='regressing'):
        save(tmp_path/'bad_time', source)
    source = fixture()
    source.edges.append(source.edges[0])
    with pytest.raises(GeometryRejected, match='duplicate'):
        save(tmp_path/'bad_edges', source)
    assert not (tmp_path/'bad_time').exists() and not (tmp_path/'bad_edges').exists()
