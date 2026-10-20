# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Selection and override boundaries for the measured VIO simulation defaults."""
import ast
from pathlib import Path

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration, PythonExpression


ROOT = Path(__file__).resolve().parents[1]


def test_lockstep_is_selected_only_for_enabled_openvins():
    tree = ast.parse((ROOT/'launch/perception_sitl.launch.py').read_text())
    assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
                      isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'gzserver')
    value = next(k.value for k in assignment.value.keywords if k.arg == 'additional_env')
    environment = eval(compile(ast.Expression(value), '<launch environment>', 'eval'), {
        'LaunchConfiguration': LaunchConfiguration, 'PythonExpression': PythonExpression})
    assert set(environment) == {'EASYFLY_GAZEBO_LOCKSTEP'}
    for backend in ('fast_lio', 'openvins', 'disabled'):
        for slam in ('true', 'false'):
            context = LaunchContext()
            context.launch_configurations.update(localization_backend=backend, slam_enabled=slam)
            assert environment['EASYFLY_GAZEBO_LOCKSTEP'].perform(context) == (
                '1' if backend == 'openvins' and slam == 'true' else '0')


def test_preflight_selects_vio_policy_before_children(monkeypatch):
    import easyfly_px4_sitl.scene_inputs as module
    import easyfly_fastdds_compat.runtime as runtime
    calls = []
    monkeypatch.setenv('LD_LIBRARY_PATH', '/test/private')
    monkeypatch.setattr(module, 'simulation_preflight', lambda *args, **kwargs: [])
    monkeypatch.setattr(runtime, 'activate', lambda **kwargs: calls.append(kwargs) or
                        {'profile_environment': {'FASTRTPS_DEFAULT_PROFILES_FILE': '/test/policy'}})
    for scene, backend, slam, expected in [
            ('', 'fast_lio', 'true', None), ('fixture', 'fast_lio', 'true', False),
            ('', 'openvins', 'true', True), ('fixture', 'openvins', 'false', False)]:
        values = dict(scene=scene, localization_backend=backend, slam_enabled=slam)
        monkeypatch.setattr(module, '_value', lambda context, key: values.get(key, ''))
        calls.clear()
        module._managed_preflight(LaunchContext())
        assert calls == ([] if expected is None else [{'openvins': expected}])
