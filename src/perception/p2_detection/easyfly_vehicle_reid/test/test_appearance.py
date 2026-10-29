# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test ambiguity, cross-camera chronology, bounded retention and input contracts."""

from easyfly_vehicle_reid.appearance import Gallery
from easyfly_vehicle_reid.node import configuration
import numpy as np
import pytest


def basis(index):
    """Return deterministic orthogonal appearance vectors for association-state tests."""
    result = np.zeros(512, dtype=np.float32)
    result[index] = 1.
    return result


def test_cross_camera_matching_does_not_depend_on_tracker_ids():
    """A later camera can reuse a learned identity with no tracker-ID input."""
    gallery = Gallery()
    original = gallery.associate('front', 10**9, [basis(0), basis(1)], 1.)
    moved = gallery.associate('side', 1100000000, [basis(1), basis(0)], 1.1)
    assert [row['state'] for row in moved] == ['matched', 'matched']
    assert [row['identity'] for row in moved] == [row['identity'] for row in original[::-1]]
    with pytest.raises(ValueError):
        gallery.associate('side', 1100000000, [basis(0)], 1.2)


def test_out_of_order_cameras_do_not_erase_newer_gallery_samples():
    """Independent current camera frames can arrive in a different source-time order."""
    gallery = Gallery()
    first = gallery.associate('front', 1100000000, [basis(0)], 1.)[0]
    second = gallery.associate('side', 10**9, [basis(0)], 1.1)[0]
    assert second['state'] == 'matched' and first['identity'] == second['identity']
    assert gallery.entries[first['identity']]['source_ns'] == 1100000000


def test_ambiguous_candidates_and_simultaneous_claims_remain_unknown():
    """Neither near ties nor two boxes claiming one identity are force-matched."""
    gallery = Gallery()
    gallery.associate('front', 10**9, [basis(0)], 1.)
    rows = gallery.associate('front', 2 * 10**9, [basis(0), basis(0)], 2.)
    assert all(row['state'] == 'ambiguous' and not row['identity'] for row in rows)
    gallery = Gallery(threshold=.5)
    alternate = (basis(0) + basis(1)) / np.sqrt(2.)
    gallery.associate('front', 10**9, [basis(0), alternate], 1.)
    between = basis(0) + alternate
    between /= np.linalg.norm(between)
    row = gallery.associate('side', 2 * 10**9, [between], 2.)[0]
    assert row['state'] == 'ambiguous' and not row['identity']


def test_expiry_capacity_and_invalid_embeddings_are_bounded():
    """Expiry creates a new identity; full capacity and invalid features cannot grow state."""
    gallery = Gallery(capacity=2, lifetime_s=2.)
    old = gallery.associate('front', 10**9, [basis(0), basis(1)], 1.)
    row = gallery.associate('front', 2 * 10**9, [basis(2)], 2.)[0]
    assert row['state'] == 'capacity_exhausted' and len(gallery.entries) == 2
    new = gallery.associate('front', 4 * 10**9, [basis(0)], 4.)[0]
    assert new['identity'] != old[0]['identity'] and new['state'] == 'new'
    for invalid in [np.zeros(512), np.ones(32), np.full(512, np.nan)]:
        with pytest.raises(ValueError):
            gallery.associate('front', 5 * 10**9, [invalid], 5.)


def test_camera_config_rejects_aliased_inputs_and_nonfinite_thresholds(tmp_path):
    """Physical source bindings must remain unique and model gates finite."""
    import json
    value = dict(version=1, use_sim_time=True, threshold=.25, margin=.05,
                 cameras=[dict(id='front', image='/camera', observed_tracks='/tracks_raw',
                               frame='optical')])
    path = tmp_path / 'config.json'
    path.write_text(json.dumps(value))
    assert configuration(path) == value
    value['cameras'][0]['observed_tracks'] = '/camera'
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        configuration(path)
    value['cameras'][0]['observed_tracks'] = '/tracks_raw'
    value['threshold'] = float('nan')
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        configuration(path)


def test_explicit_references_survive_expiry_but_not_bypass_appearance():
    """A static enrollment is not a current observation; every new crop must still match."""
    gallery = Gallery(lifetime_s=2.)
    gallery.seed({'vehicle_a': [basis(0)], 'vehicle_b': [basis(1)]})
    row = gallery.associate('front', 100 * 10**9, [basis(0)], 100.)[0]
    assert row['identity'] == 'vehicle_a' and row['state'] == 'matched'
    row = gallery.associate('side', 101 * 10**9, [basis(2)], 101.)[0]
    assert row['state'] == 'new' and row['identity'] not in ('vehicle_a', 'vehicle_b')
    gallery.associate('front', 200 * 10**9, [], 200.)
    assert set(gallery.entries) == {'vehicle_a', 'vehicle_b'}


def test_enrollment_rejects_wrong_model_hash_path_escape_and_tampering(tmp_path):
    """Reject invalid manifests before executing any feature inference."""
    import json
    from easyfly_vehicle_reid.appearance import MODEL_SHA256
    from easyfly_vehicle_reid.enrollment import load
    path = tmp_path / 'enrollment.json'
    value = dict(version=1, model_sha256='0' * 64, vehicles=[])
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        load(path, None)
    value.update(model_sha256=MODEL_SHA256, vehicles=[dict(identity='vehicle', views=[
        dict(file='../outside.png', sha256='0' * 64)])])
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='inside'):
        load(path, None)
    (tmp_path / 'crop.png').write_bytes(b'not the registered crop')
    value['vehicles'][0]['views'][0]['file'] = 'crop.png'
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='hash mismatch'):
        load(path, None)
