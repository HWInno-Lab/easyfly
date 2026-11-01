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
"""Exercise the installation used by the application, without source-path shims."""

import json
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest


def installed_command(*arguments, request=None):
    """Load exactly the environment selected by the application launcher."""
    prefix = Path(get_package_prefix('easyfly_connection_manager'))
    setup = prefix / 'setup.bash'
    if not setup.is_file():
        setup = prefix.parent / 'setup.bash'
    assert setup.is_file(), 'Installed EasyFly setup.bash is missing'
    script = '''
set -e
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source "$1"
shift
exec "$@"
'''
    return subprocess.run(
        ['/bin/bash', '-c', script, 'easyfly-installed-scenes', str(setup), *arguments],
        input=json.dumps(request) + '\n' if request is not None else None,
        text=True, capture_output=True, timeout=30, check=False)


def test_installed_application_can_load_the_scene_catalog():
    """Catch missing installed scene modules even when unit tests use source paths."""
    prefix = Path(get_package_prefix('easyfly_connection_manager'))
    executable = prefix / 'lib/easyfly_connection_manager/easyfly_connection_backend'
    result = installed_command(str(executable), '--catalog', '--px4-directory', '')
    assert result.returncode == 0, result.stdout + result.stderr
    catalog = json.loads(result.stdout)
    registered = [scene for scene in catalog['scenes'] if scene['status'] == 'registered']
    assert any('base_sim' in scene['profiles'] for scene in registered)
    assert any('navigation_dev' in scene['profiles'] for scene in registered)


def test_installed_launch_helpers_are_importable_without_source_paths():
    """Verify helpers needed after scene selection are present in the installation."""
    result = installed_command('/usr/bin/python3', '-c', '''
import easyfly_launch.composition
import easyfly_fastdds_compat.runtime
import easyfly_precision_landing.simulation_setup
import easyfly_px4_sitl.imu_compatibility
import easyfly_px4_sitl.scene_environment
import easyfly_sim_launcher.scene_cli
''')
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize('scene', ['warehouse_aws_v1', 'office_willowgarage_v1'])
def test_installed_base_scene_passes_the_actual_preflight(scene):
    """Require real installed assets and plugins, without starting a simulator."""
    prefix = Path(get_package_prefix('easyfly_connection_manager'))
    executable = prefix / 'lib/easyfly_connection_manager/easyfly_connection_backend'
    request = dict(
        mode='sitl', simulation_time=True, selected_profile='base_sim',
        simulation=dict(scene=scene, px4_directory='',
                        gazebo_port=11495, gazebo_gui=False),
        fcu=dict(transport='udp', bind_host='127.0.0.1', bind_port=14540,
                 remote_host='127.0.0.1', remote_port=14580,
                 serial_device='/dev/ttyACM0', baud=921600, target_system=1),
        gcs=dict(enabled=False, host='127.0.0.1', port=14560, bind_port=14558))
    result = installed_command(str(executable), '--check', request=request)
    assert result.returncode == 0, result.stdout + result.stderr
    assert json.loads(result.stdout)['ok'] is True
