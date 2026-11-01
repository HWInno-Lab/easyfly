# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Template serialization and one-request persistence without devices or flight."""

import copy
import hashlib
from pathlib import Path
import sys
import uuid

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from easyfly_connection_manager import onboard_profiles as profiles  # noqa: E402
from easyfly_connection_manager import onboard_parameter_templates as templates  # noqa: E402
from easyfly_connection_manager.onboard_runtime import PlatformOwner  # noqa: E402
from easyfly_connection_manager.onboard_service import bootstrap_local  # noqa: E402
from easyfly_connection_manager.remote_profiles import private_json  # noqa: E402


class Observer:
    """Keep platform ownership idle; no ROS or hardware observation is created."""

    ever_connected = False

    def __init__(self, recipe):
        pass

    def snapshot(self, name):
        """Supply only the idle fixture evidence needed for configuration."""
        return dict(grounded=True, isolated=True, observer_alive=True, startup_abort_safe=True)

    def close(self):
        """Release no external resources."""
        pass


@pytest.fixture
def owner(tmp_path):
    server, _ = bootstrap_local(tmp_path / 'platform', 189)
    result = PlatformOwner(private_json(server), observer_factory=Observer)
    yield result
    result.close()


def request(owner, recipe, names, expected_hash=None):
    return dict(v=2, server_id=owner.config['server_id'], boot_id=owner.boot,
                id=uuid.uuid4().hex, client_id='c' * 32, op='configure',
                data=dict(recipe=recipe, templates=names,
                          expected_hash=expected_hash or profiles.digest(owner.recipe)))


def enabled(*names):
    result = profiles.template()
    for name in names:
        result['components'][name]['enabled'] = True
    return result


def test_catalog_does_not_mutate_persisted_defaults_or_share_mutable_arrays():
    before = profiles.template()
    catalog = templates.catalog()
    assert set(catalog) == set(profiles.CATALOG)
    assert catalog['fast_lio']['parameters']['mapping.extrinsic_T'] == []
    catalog['fast_lio']['parameters']['mapping.extrinsic_T'].append(12)
    assert templates.catalog()['fast_lio']['parameters']['mapping.extrinsic_T'] == []
    assert profiles.template() == before
    assert not catalog['mavlink']['accepts_file']
    assert not catalog['local_planner']['accepts_file']
    assert catalog['realsense']['format'] == 'flat'


def test_serialization_overlay_formats_and_content_identity(tmp_path):
    recipe = enabled('color_line', 'realsense')
    original = copy.deepcopy(recipe)
    recipe['components']['color_line']['parameters']['hue_min'] = 42
    result = templates.materialize(recipe, ['color_line', 'realsense'], tmp_path)
    assert recipe['components']['color_line']['parameter_file'] == ''
    for name in ['color_line', 'realsense']:
        entry = result['components'][name]
        path = Path(entry['parameter_file'])
        content = path.read_bytes()
        assert templates.is_template_file(path, name)
        assert path.name.startswith(hashlib.sha256(content).hexdigest()[:16])
        assert path.stat().st_mode & 0o777 == 0o600
        document = yaml.safe_load(content)
        values = document if name == 'realsense' else document['/**']['ros__parameters']
        assert values['use_sim_time'] is False
        assert {k: v for k, v in values.items() if k != 'use_sim_time'} == entry['parameters']
    assert result['components']['color_line']['parameters']['hue_min'] == 42
    # Repeated explicit save can reuse immutable bytes but never mutate another component.
    assert templates.materialize(recipe, ['color_line', 'realsense'], tmp_path) == result
    assert result['components']['ellipse'] == original['components']['ellipse']


def test_invalid_hardware_is_rejected_before_any_file_is_written(tmp_path):
    recipe = enabled('color_line', 'fast_lio')
    with pytest.raises(ValueError, match='真实'):
        templates.materialize(recipe, ['color_line', 'fast_lio'], tmp_path)
    assert not list(tmp_path.iterdir())
    fast = recipe['components']['fast_lio']['parameters']
    fast.update({'preprocess.lidar_type': 1, 'preprocess.scan_line': 4,
                 'preprocess.timestamp_unit': 3, 'preprocess.scan_rate': 10,
                 'mapping.extrinsic_T': [0., 0., 0.],
                 'mapping.extrinsic_R': [1., 0., 0., 0., 1., 0., 0., 0., 1.]})
    result = templates.materialize(recipe, ['fast_lio'], tmp_path)
    assert result['components']['fast_lio']['parameters']['mapping.extrinsic_T'] == [0., 0., 0.]
    fast['mapping.extrinsic_R'] = [1.] * 9
    with pytest.raises(ValueError, match='正交'):
        templates.materialize(recipe, ['fast_lio'], tmp_path)


def test_body_offset_clock_and_symlinks_do_not_bypass_template_validation(tmp_path):
    recipe = enabled('map_odom')
    with pytest.raises(ValueError, match='body_to_base_translation'):
        templates.materialize(recipe, ['map_odom'], tmp_path)
    recipe['components']['map_odom']['parameters']['body_to_base_translation'] = [0., 0., 0.]
    recipe['components']['map_odom']['parameters']['use_sim_time'] = True
    with pytest.raises(ValueError, match='模拟时间'):
        templates.materialize(recipe, ['map_odom'], tmp_path)
    recipe = enabled('ellipse')
    result = templates.materialize(recipe, ['ellipse'], tmp_path)
    path = Path(result['components']['ellipse']['parameter_file'])
    outside = tmp_path / 'outside.yaml'
    path.rename(outside)
    path.symlink_to(outside)
    with pytest.raises(ValueError, match='未覆盖'):
        templates.materialize(recipe, ['ellipse'], tmp_path)
    assert path.is_symlink()


def test_atomic_configure_keeps_hash_idle_and_task_guards_before_template_writes(owner):
    recipe = enabled('color_line')
    root = Path(owner.config['configuration_roots'][0])
    assert owner.status()['capabilities']['parameter_templates'] == 1
    assert not owner.mutate(request(owner, recipe, ['color_line'], 'f' * 64))['accepted']
    assert not list(root.iterdir())
    owner.state['phase'] = 'running'
    assert not owner.mutate(request(owner, recipe, ['color_line']))['accepted']
    assert not list(root.iterdir())
    owner.state['phase'] = 'stopped'
    owner.state['tasks']['pending'] = dict(execution_pending=True)
    assert not owner.mutate(request(owner, recipe, ['color_line']))['accepted']
    assert not list(root.iterdir())
    owner.state['tasks'].clear()
    operation = request(owner, recipe, ['color_line'])
    result = owner.mutate(operation)
    assert result['accepted']
    assert result['recipe'] == owner.recipe
    assert result['recipe_hash'] == profiles.digest(owner.recipe)
    assert owner.mutate(operation) == result
    assert len(list(root.iterdir())) == 1
    assert not owner.children


def test_invalid_second_template_keeps_original_recipe_and_files(owner):
    recipe = enabled('color_line', 'map_odom')
    before = copy.deepcopy(owner.recipe)
    result = owner.mutate(request(owner, recipe, ['color_line', 'map_odom']))
    assert not result['accepted']
    assert owner.recipe == before
    assert not list(Path(owner.config['configuration_roots'][0]).iterdir())


def test_real_tls_save_returns_canonical_recipe_and_replay_receipt(tmp_path):
    """Exercise authenticated template saving without starting an adapter or ROS."""
    import threading
    from easyfly_connection_manager.onboard_client import PlatformClient
    from easyfly_connection_manager.onboard_deployment import client_settings
    from easyfly_connection_manager.onboard_service import PlatformHandler
    from easyfly_connection_manager.remote_service import Server

    server_path, client_path = bootstrap_local(tmp_path / 'platform', 188)
    config = private_json(server_path)
    stop, ready = threading.Event(), threading.Event()
    errors, owners = [], []

    def serve():
        owner = service = None
        try:
            owner = PlatformOwner(config, observer_factory=Observer)
            owners.append(owner)
            service = Server(config, owner, PlatformHandler)
            service.timeout = 0.05
            ready.set()
            while not stop.is_set():
                service.handle_request()
        except Exception as error:
            errors.append(error)
            ready.set()
        finally:
            if service:
                service.server_close()
            if owner:
                owner.close()

    worker = threading.Thread(target=serve)
    worker.start()
    try:
        assert ready.wait(5) and not errors
        client = PlatformClient(client_settings(client_path))
        snapshot = client.status()
        assert snapshot['capabilities']['parameter_templates'] == 1
        recipe = enabled('color_line', 'flight_control', 'map_odom')
        recipe['components']['color_line']['parameters']['hue_min'] = 42
        recipe['components']['flight_control']['parameters']['control_rate'] = 25
        recipe['components']['map_odom']['parameters']['body_to_base_translation'] = [0, 0, 0.1]
        values = dict(recipe=recipe, expected_hash=snapshot['recipe_hash'],
                      templates=['color_line', 'flight_control', 'map_odom'])
        identity = uuid.uuid4().hex
        receipt = client.operation(snapshot, 'configure', values, request_id=identity)
        assert receipt['accepted'], receipt
        assert client.operation(snapshot, 'configure', values, request_id=identity) == receipt
        current = client.status()
        assert current['recipe'] == receipt['recipe']
        assert current['recipe_hash'] == receipt['recipe_hash']
        path = Path(receipt['recipe']['components']['color_line']['parameter_file'])
        assert templates.is_template_file(path, 'color_line')
        assert yaml.safe_load(path.read_bytes())['/**']['ros__parameters']['hue_min'] == 42
        controller = receipt['recipe']['components']['flight_control']
        assert type(controller['parameters']['control_rate']) is float
        offset = receipt['recipe']['components']['map_odom']
        assert all(type(value) is float for value in offset['parameters'][
            'body_to_base_translation'])
        assert len(list(Path(config['configuration_roots'][0]).iterdir())) == 3
        assert not client.operation(current, 'configure', values)['accepted']
        assert not owners[0].children
    finally:
        stop.set()
        worker.join(timeout=5)
        assert not worker.is_alive() and not errors


def test_qt_json_numeric_values_become_the_declared_ros_parameter_types(tmp_path):
    """Integral JSON numbers and mixed physical arrays retain ROS double types."""
    import json
    recipe = enabled('flight_control', 'map_odom', 'fast_lio')
    recipe['components']['flight_control']['parameters'] = json.loads('{"control_rate":25}')
    recipe['components']['map_odom']['parameters'] = json.loads(
        '{"body_to_base_translation":[0,0,0.1]}')
    recipe['components']['fast_lio']['parameters'] = json.loads(
        '{"preprocess.lidar_type":1,"preprocess.scan_line":4.0,'
        '"preprocess.timestamp_unit":3,"preprocess.scan_rate":10,'
        '"mapping.extrinsic_T":[0,0,0],"mapping.extrinsic_R":[1,0,0,0,1,0,0,0,1]}')
    original = copy.deepcopy(recipe)
    result = templates.materialize(recipe, ['flight_control', 'map_odom', 'fast_lio'], tmp_path)
    assert recipe == original
    assert type(result['components']['flight_control']['parameters']['control_rate']) is float
    fast = result['components']['fast_lio']['parameters']
    assert type(fast['preprocess.scan_line']) is int
    for name, key in [('map_odom', 'body_to_base_translation'),
                      ('fast_lio', 'mapping.extrinsic_T'), ('fast_lio', 'mapping.extrinsic_R')]:
        entry = result['components'][name]
        assert all(type(value) is float for value in entry['parameters'][key])
        document = yaml.safe_load(Path(entry['parameter_file']).read_bytes())
        values = document['/**']['ros__parameters']
        assert all(type(value) is float for value in values[key])
    assert templates.normalize_parameters(result, [
        'flight_control', 'map_odom', 'fast_lio']) == result


@pytest.mark.parametrize('name,key,value', [
    ('flight_control', 'control_rate', True), ('flight_control', 'control_rate', '25'),
    ('flight_control', 'control_rate', float('nan')),
    ('flight_control', 'control_rate', float('inf')),
    ('fast_lio', 'preprocess.scan_line', 4.5),
    ('fast_lio', 'preprocess.scan_line', True),
    ('fast_lio', 'mapping.extrinsic_R', [True] * 9),
    ('map_odom', 'body_to_base_translation', [0, '0', 0]),
    ('map_odom', 'body_to_base_translation', [0, float('inf'), 0]),
])
def test_bad_template_numeric_types_never_write_files(tmp_path, name, key, value):
    recipe = enabled(name)
    recipe['components'][name]['parameters'][key] = value
    with pytest.raises(ValueError):
        templates.materialize(recipe, [name], tmp_path)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('recipe,names', [
    (None, ['color_line']), ({}, ['color_line']), ({'components': []}, ['color_line']),
    ({'components': {'color_line': None}}, ['color_line']),
    ({'components': {'color_line': {'enabled': True, 'parameters': []}}}, ['color_line']),
    ({'components': {}}, [['color_line']]), ({'components': {}}, 'color_line'),
])
def test_malformed_template_recipes_are_rejected_before_writes(tmp_path, recipe, names):
    with pytest.raises(ValueError):
        templates.materialize(recipe, names, tmp_path)
    assert not list(tmp_path.iterdir())


def test_non_template_parameters_and_legacy_digests_are_unchanged():
    recipe = enabled('flight_control')
    recipe['components']['flight_control']['parameters']['control_rate'] = 25
    original = copy.deepcopy(recipe)
    assert templates.normalize_parameters(recipe, []) == recipe
    assert profiles.digest(templates.normalize_parameters(recipe, [])) == profiles.digest(recipe)
    result = templates.normalize_parameters(recipe, ['flight_control'])
    assert type(result['components']['flight_control']['parameters']['control_rate']) is float
    assert recipe == original


@pytest.mark.parametrize('value', [2**63, -(2**63) - 1, [2**63], [-(2**63) - 1]])
def test_extra_integer_scalar_and_array_overflow_never_write_files(tmp_path, value):
    """Unknown template parameter names still require ROS signed-64 integer values."""
    recipe = enabled('color_line')
    recipe['components']['color_line']['parameters']['extra_integer'] = value
    with pytest.raises(ValueError, match='64 位'):
        templates.materialize(recipe, ['color_line'], tmp_path)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('value', [-(2**63), 2**63 - 1, [-(2**63), 2**63 - 1], True, [True]])
def test_extra_integer_boundaries_and_boolean_types_remain_exact(tmp_path, value):
    """Representable integers and actual booleans preserve distinct YAML types."""
    recipe = enabled('color_line')
    recipe['components']['color_line']['parameters']['extra_integer'] = value
    result = templates.materialize(recipe, ['color_line'], tmp_path)
    path = Path(result['components']['color_line']['parameter_file'])
    loaded = yaml.safe_load(path.read_bytes())['/**']['ros__parameters']['extra_integer']
    assert loaded == value and type(loaded) is type(value)
    if isinstance(value, list):
        assert [type(item) for item in loaded] == [type(item) for item in value]
