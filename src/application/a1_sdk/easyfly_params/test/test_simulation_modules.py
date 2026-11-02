# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Selection closure, legacy behavior, and inactive algorithm dependency isolation."""
import pytest

from easyfly_params.algorithms import fields
from easyfly_params.simulation import resolve_simulation_modules


def no_modules(**overrides):
    options = {field['key']: False for field in fields() if field['key'].startswith('start_')}
    return dict(options, **overrides)


def test_base_profile_does_not_imply_the_legacy_perception_pipeline():
    result = resolve_simulation_modules('base_sim')
    assert result['effective'] == []
    assert result['enabled_roles'] == []


def test_default_perception_preserves_existing_modules_without_adding_prediction():
    result = resolve_simulation_modules('navigation_dev')
    assert {'slam', 'navigation', 'object_detection', 'tracking', 'single_tracking',
            'pointcloud_filter', 'semantic_map', 'pixel_to_3d', 'target_tracking'} <= set(result['effective'])
    assert 'target_prediction' not in result['effective']


def test_multitracking_adds_detection_without_enabling_navigation_or_slam():
    result = resolve_simulation_modules('full_perception', no_modules(
        slam='disabled', start_tracking=True))
    assert set(result['effective']) == {'tracking', 'object_detection'}
    assert result['automatic'] == ['object_detection']
    assert result['reasons']['object_detection'] == ['tracking']
    assert result['launch_arguments']['start_detection'] == 'true'
    assert result['launch_arguments']['start_navigation'] == 'false'


def test_single_tracking_does_not_require_object_detection():
    result = resolve_simulation_modules('full_perception', no_modules(
        slam='disabled', start_single_tracking=True))
    assert result['effective'] == ['single_tracking']
    assert result['enabled_roles'] == ['single_tracking']


def test_fuel_adds_its_navigation_chain_without_loading_the_yolo_chain():
    result = resolve_simulation_modules('navigation_dev', no_modules(
        slam='fast_lio', strategy='gcopter', exploration='fuel'))
    assert set(result['effective']) == {'slam', 'navigation', 'exploration'}
    assert result['enabled_roles'] == []
    assert result['options']['start_navigation'] is True
    assert result['options']['start_detection'] is False
    assert 'semantic_map' not in result['effective']


@pytest.mark.parametrize('strategy', ['astar', 'ego', 'combined'])
def test_grid_navigation_adds_the_obstacle_map_chain(strategy):
    result = resolve_simulation_modules('navigation_dev', no_modules(
        slam='fast_lio', strategy=strategy, start_navigation=True))
    assert set(result['effective']) == {'slam', 'navigation', 'pointcloud_filter', 'semantic_map'}
    assert result['enabled_roles'] == ({'astar': ['global_grid'], 'combined': ['combined_grid']}.get(strategy, []))


def test_disabled_localization_does_not_get_silently_reenabled_by_old_defaults():
    result = resolve_simulation_modules('navigation_dev', dict(slam='disabled'))
    assert 'slam' not in result['effective']
    assert not {'navigation', 'semantic_map', 'pointcloud_filter', 'target_tracking'} & set(result['effective'])
    assert result['blocked']['navigation']


def test_vio_selection_keeps_incompatible_legacy_follow_disabled():
    result = resolve_simulation_modules('navigation_dev', dict(slam='openvins'))
    assert result['localization'] == 'openvins'
    assert 'target_tracking' not in result['effective']


def test_fuel_still_rejects_an_incompatible_terminal_selection():
    with pytest.raises(ValueError, match='FUEL'):
        resolve_simulation_modules('navigation_dev', dict(exploration='fuel', strategy='ego'))


def test_inactive_detection_and_tracking_do_not_read_their_installed_configuration(monkeypatch):
    from easyfly_params import backends
    def unexpected_package(name):
        raise AssertionError('Inactive backend attempted to resolve package: ' + name)
    monkeypatch.setattr(backends, 'get_package_share_directory', unexpected_package)
    result = backends.resolve_selection('full_perception', no_modules(slam='disabled'))
    assert result['enabled_roles'] == []
    assert result['legacy_files'] == {}
