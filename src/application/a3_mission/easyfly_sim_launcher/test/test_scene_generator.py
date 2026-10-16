# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reproducibility, independent geometry contracts and fail-closed publication."""
import copy
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import jsonschema
import pytest
import yaml

from easyfly_sim_launcher import scene_generator as generator
from easyfly_px4_sitl.scene_catalog import SceneCatalog

ROOT = Path(__file__).resolve().parents[5]
WORLDS = ROOT / 'src/simulation/easyfly_gazebo_worlds'


@pytest.fixture
def config(monkeypatch):
    monkeypatch.setattr(generator, 'world_share', lambda: WORLDS)
    return json.loads((WORLDS / 'generators/primitives_sparse_seed42_v1.json').read_text())


def test_specified_random_stream():
    rng = generator.Random64(0)
    assert [rng.next() for _ in range(3)] == [
        0xe220a8397b1dcdaf, 0x6e789e6aa1b965f4, 0x06c45d188009454f]


def test_identical_inputs_reproduce_all_bytes_and_seed_changes_layout(config, tmp_path):
    a, b = tmp_path / 'one', tmp_path / 'two'
    first = generator.generate(config, a)
    assert first == generator.generate(config, b)
    generator.check_layout(config, a)
    different = copy.deepcopy(config)
    different['seed'] += 1
    assert generator.bundle(different) != first
    original = {p: (a / p).read_bytes() for p in first}
    with pytest.raises(FileExistsError):
        generator.generate(different, a)
    assert {p: (a / p).read_bytes() for p in first} == original


@pytest.mark.parametrize('kind', ['sparse', 'corridor'])
def test_native_geometry_contract_and_draft_catalog(config, tmp_path, kind):
    config['layout'] = kind
    files = generator.generate(config, tmp_path / 'bundle')
    root = tmp_path / 'bundle'
    world_path = next(p for p in files if p.endswith('.world'))
    world = ET.parse(root / world_path).getroot()
    models = world.findall('./world/model')
    assert len(models) == config['obstacles']['count'] + (7 if kind == 'corridor' else 5)
    assert not world.findall('.//uri') and not world.findall('.//plugin')
    for model in models:
        assert model.findtext('static') == 'true'
        collision, visual = model.find('./link/collision/geometry'), model.find('./link/visual/geometry')
        collision.tail = visual.tail = None
        assert ET.tostring(collision) == ET.tostring(visual)
    # Independently evaluate emitted geometry against the reserved rectangular
    # part of the corridor, not the generator's own clearance predicate.
    sx, sy, _ = config['spawn']
    ex, margin = sx + config['route']['length_m'], config['route']['width_m'] / 2
    for model in models:
        if model.get('name') == 'arena_floor':
            continue
        px, py, _, _, _, _ = map(float, model.findtext('pose').split())
        geometry = model.find('./link/collision/geometry')
        size = geometry.findtext('./box/size')
        if size:
            dx, dy, _ = map(float, size.split())
        else:
            dx = dy = 2 * float(geometry.findtext('./cylinder/radius'))
        # Cylinder AABBs are conservative; the centre must at least be outside
        # the rectangular route strip or beyond its end caps.
        assert px + dx / 2 <= sx or px - dx / 2 >= ex or py + dy / 2 <= sy - margin + 1e-6 or py - dy / 2 >= sy + margin - 1e-6
    shutil.copytree(WORLDS / 'schemas', root / 'schemas')
    report = SceneCatalog({'easyfly_gazebo_worlds': root,
                           'easyfly_gazebo_models': ROOT / 'src/simulation/easyfly_gazebo_models'}).validate(config['scene_id']).as_dict()
    assert report['scene_resources_complete'] and report['licenses_complete']
    assert not report['registration_complete'] and not report['runtime_verified']
    data = yaml.safe_load((root / f'scenes/{config["scene_id"]}.yaml').read_text())
    assert data['status'] == 'draft' and not data['supported_profiles']
    assert data['navigation']['world_to_navigation'] is None
    assert not data['verification']['navigation_tested']


def test_blocked_layout_never_publishes_a_scene(config, tmp_path):
    config['layout'] = 'blocked'
    output = tmp_path / 'rejected'
    with pytest.raises(ValueError, match='reference corridor blocked by rejected_barrier'):
        generator.generate(config, output)
    assert not output.exists()


@pytest.mark.parametrize('change', ['spawn', 'bounds', 'ranges', 'budget'])
def test_impossible_geometry_rejected_without_partial_scene(config, tmp_path, change, monkeypatch):
    if change == 'spawn':
        config['spawn'][0] = config['bounds']['min'][0]
    elif change == 'bounds':
        config['bounds']['max'][0] = config['bounds']['min'][0]
    elif change == 'ranges':
        config['obstacles']['min_size_m'] = 1.4
        config['obstacles']['max_size_m'] = .4
    else:
        config['obstacles'].update(count=1, min_size_m=.4, max_size_m=.4)
        # Every candidate remains inside the reserved route: exhaust the real
        # bounded placement loop, without returning a smaller obstacle count.
        monkeypatch.setattr(generator.Random64, 'between',
                            lambda self, low, high: 0. if low <= 0 <= high else low)
    with pytest.raises(ValueError, match='placement budget exhausted' if change == 'budget' else None):
        generator.generate(config, tmp_path / 'invalid')
    assert not (tmp_path / 'invalid').exists()


@pytest.mark.parametrize('text', [
    '{"seed":1,"seed":2}', '{"seed":NaN}', '{"seed":Infinity}',
    '{"seed":true}', '{"scene_id":"../escape"}', '{}',
])
def test_bad_parameter_documents(config, tmp_path, text):
    path = tmp_path / 'input.json'
    path.write_text(text)
    with pytest.raises((ValueError, jsonschema.ValidationError)):
        generator.load_config(path)


@pytest.mark.parametrize('field,value', [('seed', True), ('seed', -1), ('seed', 2**32),
                                        ('scene_id', '../escape'), ('layout', 'script.py'),
                                        ('unknown', 'value')])
def test_complete_document_schema_guards(config, tmp_path, field, value):
    config[field] = value
    path = tmp_path / 'complete.json'
    path.write_text(json.dumps(config))
    with pytest.raises((ValueError, jsonschema.ValidationError)):
        generator.load_config(path)
    with pytest.raises((ValueError, jsonschema.ValidationError)):
        generator.generate(config, tmp_path / 'rejected')
    assert not (tmp_path / 'rejected').exists()


@pytest.mark.parametrize('identifier', ['primitives_sparse_seed42_v1',
                                       'primitives_corridor_seed1042_v1',
                                       'primitives_corridor_seed1042_v2'])
def test_checked_in_sample_matches_generator(config, identifier):
    data = generator.load_config(WORLDS / f'generators/{identifier}.json')
    generator.check_layout(data, WORLDS)


def test_layout_check_detects_tampering_and_preserves_qualification(config, tmp_path):
    generator.generate(config, tmp_path / 'scene')
    root = tmp_path / 'scene'
    path = root / f'scenes/{config["scene_id"]}.yaml'
    scene = yaml.safe_load(path.read_text())
    scene['navigation']['world_to_navigation'] = {'translation': [0, 0, 0], 'yaw': 0}
    scene['verification']['world_loaded'] = True
    path.write_text(yaml.safe_dump(scene))
    generator.check_layout(config, root)  # Read-only; no certification is inferred.
    world = root / scene['world']['path']
    world.write_bytes(world.read_bytes() + b'<!-- changed -->')
    with pytest.raises(ValueError, match='generated bytes differ'):
        generator.check_layout(config, root)
