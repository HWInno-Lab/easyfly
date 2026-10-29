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
"""Validate administrator-owned presets; remote clients supply only preset identities."""

import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat

from ament_index_python.packages import get_package_prefix, get_package_share_directory


# A client can never request an arbitrary executable, environment or shell command.
CATALOG = {
    'color_line': ('easyfly_color_line_detection', 'color_line_detection_node', False),
    'ellipse': ('easyfly_ellipse_detection', 'ellipse_detection_node', False),
    'fast_lio': ('easyfly_fast_lio_integration', 'fastlio.launch.py', True),
    'openvins': ('easyfly_openvins_integration', 'openvins.launch.py', True),
    'fast_livo2': ('easyfly_fast_livo2_integration', 'fastlivo.launch.py', True),
    'orbslam3': ('easyfly_orbslam3_integration', 'orbslam3.launch.py', True),
    'gcopter': ('easyfly_trajectory_optimizer', 'gcopter.launch.py', True),
    'super': ('easyfly_super_planner', 'super.launch.py', True),
    'fuel': ('easyfly_fuel_exploration', 'fuel.launch.py', True),
}


def private_json(path):
    """Load a bounded owned configuration through a non-following file descriptor."""
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'r') as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or
                info.st_mode & 0o077 or info.st_size > 65536):
            raise ValueError('configuration must be an owned private file, at most 64 KiB')
        return json.load(stream)


def fingerprint(value):
    """Freeze preset content independently of object key ordering."""
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def validate(config):
    """Fail before binding when identity, credentials or preset fields are ambiguous."""
    required = {'server_id', 'bind', 'port', 'certificate', 'private_key', 'token_file',
                'state_directory', 'ros_domain_id', 'simulation_time', 'profiles'}
    if (not isinstance(config, dict) or set(config) != required or
            not re.fullmatch('[a-f0-9]{32}', config['server_id']) or
            type(config['port']) is not int or not 1 <= config['port'] <= 65535 or
            type(config['ros_domain_id']) is not int or not 0 <= config['ros_domain_id'] <= 232 or
            type(config['simulation_time']) is not bool or
            not isinstance(config['profiles'], dict) or not 1 <= len(config['profiles']) <= 16):
        raise ValueError('invalid remote server configuration')
    for key in ('certificate', 'private_key', 'token_file', 'state_directory'):
        path = Path(config[key])
        if not path.is_absolute() or path.resolve() != path:
            raise ValueError('remote server paths must be absolute without symlinks')
    for name, profile in config['profiles'].items():
        if (not re.fullmatch('[a-z][a-z0-9_]{0,47}', name) or
                set(profile) != {'algorithm', 'parameters', 'output_topic',
                                 'output_type', 'policy'} or
                profile['algorithm'] not in CATALOG or
                profile['policy'] not in ('grounded', 'isolated_observation') or
                not isinstance(profile['parameters'], dict) or len(profile['parameters']) > 64 or
                not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', profile['output_topic']) or
                not re.fullmatch(r'[a-z][a-z0-9_]*/msg/[A-Za-z][A-Za-z0-9]*',
                                 profile['output_type'])):
            raise ValueError('invalid algorithm preset: ' + name)
        if profile['policy'] == 'isolated_observation' and profile['algorithm'] not in (
                'color_line', 'ellipse'):
            raise ValueError('only independent image observations support isolated policy')
        if ('use_sim_time' in profile['parameters'] and
                profile['parameters']['use_sim_time'] is not config['simulation_time']):
            raise ValueError('preset and service must use the same ROS clock')
        if profile['algorithm'] in ('openvins', 'fast_livo2', 'orbslam3') and not (
                config['simulation_time']):
            raise ValueError('this installed localization launch requires calibrated simulation')
        if profile['algorithm'] in ('color_line', 'ellipse'):
            expected = ('ColorLine' if profile['algorithm'] == 'color_line' else 'Ellipse')
            if (profile['output_type'] != 'easyfly_msgs/msg/' + expected + 'ObservationArray' or
                    profile['parameters'].get('output_topic') != profile['output_topic']):
                raise ValueError('observation type and configured output must match the preset')
        for key, value in profile['parameters'].items():
            if (not re.fullmatch('[A-Za-z_][A-Za-z0-9_.]*', key) or
                    type(value) not in (str, int, float, bool) or
                    (isinstance(value, float) and not math.isfinite(value)) or
                    (isinstance(value, str) and (len(value) > 1024 or '\n' in value))):
                raise ValueError('invalid preset parameter')
    return config


def command(profile, directory, simulation_time):
    """Resolve only installed catalog binaries and freeze typed parameters locally."""
    package, name, launch = CATALOG[profile['algorithm']]
    parameters = dict(profile['parameters'])
    parameters.setdefault('use_sim_time', simulation_time)
    if profile['algorithm'] == 'orbslam3':
        runtime = directory / 'atlas'
        runtime.mkdir(mode=0o700)
        parameters['runtime_directory'] = str(runtime)
    if profile['algorithm'] == 'fuel':
        parameters['evidence_directory'] = str(directory / 'fuel')
    if launch:
        path = Path(get_package_share_directory(package)) / 'launch' / name
        if not path.is_file():
            raise ValueError('installed launch file is missing')
        return ['ros2', 'launch', package, name, *[
            key + ':=' + (str(value).lower() if type(value) is bool else str(value))
            for key, value in parameters.items()]]
    executable = Path(get_package_prefix(package)) / 'lib' / package / name
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise ValueError('installed algorithm executable is missing')
    # JSON is valid YAML and preserves numbers, booleans and strings without CLI coercion.
    settings = directory / 'parameters.yaml'
    settings.write_text(json.dumps({'/**': {'ros__parameters': parameters}}))
    settings.chmod(0o600)
    return [str(executable), '--ros-args', '--params-file', str(settings)]
