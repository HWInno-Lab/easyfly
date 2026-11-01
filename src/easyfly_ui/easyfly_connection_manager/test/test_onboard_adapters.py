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
"""Physical adapter admission without launching devices or actions."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from easyfly_connection_manager import onboard_profiles as profiles  # noqa: E402
from easyfly_connection_manager import onboard_parameter_templates as templates  # noqa: E402


def test_follow_template_uses_native_custom_yaml(tmp_path):
    recipe = profiles.template()
    recipe['components']['target_tracking']['enabled'] = True
    frozen = templates.materialize(recipe, ['target_tracking'], tmp_path)
    entry = frozen['components']['target_tracking']
    assert not entry['parameter_file']
    import yaml

    document = yaml.safe_load(Path(entry['parameters']['config_file']).read_text())
    assert profiles.validate_tracking_configuration(document) == profiles.TRACK_DEFAULTS


@pytest.mark.parametrize(
    'change',
    [
        dict(max_altitude=0),
        dict(state_timeout=True),
        dict(input_topic='relative'),
        dict(unknown=1),
    ],
)
def test_follow_rejects_invalid_or_unknown_configuration(change):
    values = dict(profiles.TRACK_DEFAULTS, **change)
    with pytest.raises(ValueError):
        profiles.validate_tracking_configuration({'target_tracking': values})


def test_two_physical_camera_drivers_require_distinct_serials():
    recipe = profiles.template()
    for name in ('realsense', 'down_camera'):
        recipe['components'][name]['enabled'] = True
    recipe['components']['down_camera']['parameters']['serial_no'] = 'down'
    with pytest.raises(ValueError, match='serial_no'):
        profiles.validate(recipe)
    recipe['components']['realsense']['parameters']['serial_no'] = 'front'
    profiles.validate(recipe)


def test_gcopter_dependency_and_exclusive_motion_output():
    recipe = profiles.template()
    recipe['components']['navigation']['parameters']['strategy'] = 'gcopter'
    expanded = profiles.enable_dependencies(recipe, ['navigation'])['recipe']
    assert expanded['components']['fuel']['enabled']
    assert expanded['components']['gcopter']['enabled']
    expanded['components']['local_planner']['enabled'] = True
    with pytest.raises(ValueError, match='GCOPTER'):
        profiles.validate(expanded)


def test_gcopter_template_preserves_explicit_multi_node_scopes(tmp_path):
    import yaml

    recipe = profiles.template()
    entry = recipe['components']['gcopter']
    entry['enabled'] = True
    entry['parameters'] = dict(
        map_min=[-5.0, -5.0, 0.0],
        map_max=[5.0, 5.0, 4.0],
        resolution=0.2,
        inflation_radius=0.4,
        source_frame='odom',
        z_min=0.5,
        z_max=3.0,
    )
    result = templates.materialize(recipe, ['gcopter'], tmp_path)
    generated = result['components']['gcopter']
    assert generated['parameters'] == {}
    document = yaml.safe_load(Path(generated['parameter_file']).read_text())
    assert set(document) == {'gcopter_voxel_map', 'gcopter_navigation'}


def test_frozen_single_navigation_adapter_uses_completed_recipe_preflight():
    recipe = profiles.template()
    entry = recipe['components']['navigation']
    entry['enabled'] = True
    entry['parameters']['strategy'] = 'gcopter'
    with pytest.raises(ValueError, match='gcopter and fuel'):
        profiles.validate(recipe)
    isolated = dict(recipe, components={'navigation': entry})
    profiles.validate(isolated, require_files=True, standalone=True)
