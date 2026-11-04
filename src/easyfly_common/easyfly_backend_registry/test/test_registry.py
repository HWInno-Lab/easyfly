# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_backend_registry/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
import copy
import json
import shutil
import pytest
from easyfly_backend_registry import Registry, DATA, loads

def test_defaults_and_role_isolation():
    r = Registry(); c = r.resolve()
    assert c['backends']['global_grid']['parameters']['max_cells'] == 4000000
    assert c['backends']['combined_grid']['parameters'] == dict(max_cells=1000000, unstick_cells=0)
    assert c['backends']['multi_tracking']['parameters']['max_lost_frames'] == 45
    assert set(r.ids('localization')) == {'fast_lio', 'openvins', 'fast_livo2', 'orbslam3'}
    assert 'tensorrt' not in r.ids('detection')

@pytest.mark.parametrize('overrides', [
    {'schema_version': 2}, {'unknown': 1},
    {'backends': {'multi_tracking': {'backend': 'missing', 'parameters': {}}}},
    {'backends': {'multi_tracking': {'backend': 'sort', 'parameters': {'max_lost_frames': True}}}},
    {'backends': {'global_grid': {'backend': 'astar', 'parameters': {'max_cells': 1.5, 'unstick_cells': 0}}}},
    {'platform': {'detection': {'unknown': 1}}},
    {'navigation': {'strategy': 'unknown'}},
])
def test_reject_invalid(overrides):
    with pytest.raises(Exception): Registry().resolve(overrides=overrides)

def test_duplicate_and_strict_json(tmp_path):
    for text in ['{"a": 1, "a": 2}', '{"a": NaN}']:
        with pytest.raises(ValueError): loads(text)
    shutil.copytree(DATA, tmp_path / 'data')
    p = tmp_path / 'data/backends.json'; data = json.loads(p.read_text())
    data['backends'].append(copy.deepcopy(data['backends'][0])); p.write_text(json.dumps(data))
    with pytest.raises(ValueError): Registry(tmp_path / 'data')

def test_requirements_and_scope(tmp_path):
    r = Registry()
    with pytest.raises(ValueError): r.check_inputs('multi_tracking', 'sort', [])
    r.check_inputs('multi_tracking', 'sort', ['detections'])
    with pytest.raises(ValueError): r.check_installation('localization', 'fast_lio', lambda _: tmp_path)
    with pytest.raises(ValueError): r.resolve(profile='base_sim')
    with pytest.raises(ValueError): r.resolve(overrides={'backends': {'exploration': dict(backend='fuel', parameters={})}})


def test_typed_mapping_priority_and_legacy_conflict():
    r = Registry()
    assert r.typed_parameters('multi_tracking', 'sort').max_lost_frames == 45
    config, sources = r.resolve_layers(profile_defaults={'backends': {'multi_tracking': {'parameters': {'min_hits': 4}}}},
        file_config={'backends': {'multi_tracking': {'parameters': {'min_hits': 5}}}},
        cli={'backends': {'multi_tracking': {'parameters': {'min_hits': 6}}}})
    assert config['backends']['multi_tracking']['parameters']['min_hits'] == 6
    assert sources['/backends/multi_tracking/parameters/min_hits'] == 'cli'
    with pytest.raises(ValueError):
        r.import_legacy({'tracking': {'min_hits': 4}}, {'backends': {'multi_tracking': {'backend': 'sort', 'parameters': {'min_hits': 5}}}})
    with pytest.raises(ValueError):
        r.resolve(overrides={'backends': {'multi_tracking': {'backend': 'sort', 'parameters': {'iou_threshold': float('nan')}}}})


def test_mismatched_navigation_backend_is_rejected():
    with pytest.raises(ValueError):
        Registry().resolve(strategy='gcopter', overrides={'backends': {'trajectory_planning': dict(backend='ego', parameters={})}})


def test_manifest_reference_cannot_escape_package(tmp_path):
    shutil.copytree(DATA, tmp_path / 'data')
    p = tmp_path / 'data/backends.json'; data = json.loads(p.read_text())
    data['backends'][0]['parameter_schema'] = 'schemas/../../outside.json'
    (tmp_path / 'outside.json').write_text('{}'); p.write_text(json.dumps(data))
    with pytest.raises(ValueError): Registry(tmp_path / 'data')


def test_actual_legacy_tracking_split_keeps_loader_fallbacks():
    r = Registry()
    patch = r.import_tracking_yaml({'tracking': {'tracker_backend': 'sort', 'input_topic': '/detections',
        'iou_threshold': .15, 'max_lost_frames': 45, 'target_classes': ['person'],
        'id_consolidation': {'enabled': True, 'reacquire_radius': 3., 'ttl': 10.}}})
    assert patch['backends']['multi_tracking']['parameters']['max_lost_frames'] == 45
    assert patch['platform']['multi_tracking']['input_topic'] == '/detections'
    assert r.import_tracking_yaml({'tracking': {}})['backends']['multi_tracking']['parameters']['max_lost_frames'] == 30
    with pytest.raises(Exception): r.import_tracking_yaml({'tracking': {'min_hitts': 3}})
