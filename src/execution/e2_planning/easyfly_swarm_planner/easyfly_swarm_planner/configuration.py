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

"""Validate bounded world-coordinate fleet tasks before reserving native flight actions."""

import json
import math
import os
from pathlib import Path
import stat


def read_json(path, limit=65536):
    """Read a bounded physical regular file without following its final symlink."""
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'r') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or not 0 < info.st_size <= limit:
            raise ValueError('expected bounded regular JSON input')
        return json.load(stream)


def point(value):
    """Reject nonfinite values, booleans and undeclared dimensions."""
    return (isinstance(value, list) and len(value) == 3 and
            all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 100
                for v in value))


def load(path, names):
    """Validate one complete immutable task with explicit limits and target assignment policy."""
    value = read_json(path)
    fields = {'version', 'lower', 'upper', 'grid_m', 'tracking_tube_m', 'separation_m',
              'vehicle_radius_m', 'obstacles', 'max_steps', 'speed_mps', 'acceleration_mps2',
              'takeoff_world_z', 'assignment', 'targets', 'land_on_complete'}
    if not isinstance(value, dict) or set(value) != fields or value['version'] != 1:
        raise ValueError('invalid swarm task fields')
    limits = {'grid_m': (.5, 2.), 'tracking_tube_m': (.15, .5), 'separation_m': (1.5, 5.),
              'vehicle_radius_m': (.25, .6), 'speed_mps': (.1, 1.),
              'acceleration_mps2': (.1, 1.), 'takeoff_world_z': (.8, 4.)}
    for key, (low, high) in limits.items():
        if (type(value[key]) not in (int, float) or not math.isfinite(value[key]) or
                not low <= value[key] <= high):
            raise ValueError('invalid swarm bound: ' + key)
    if not point(value['lower']) or not point(value['upper']):
        raise ValueError('invalid world bounds')
    sizes = [high - low for low, high in zip(value['lower'], value['upper'])]
    if (any(not value['grid_m'] * 2 <= size <= 30 for size in sizes) or
            math.prod(math.ceil(size / value['grid_m']) + 1 for size in sizes) > 12000 or
            value['lower'][2] < .2 or value['upper'][2] > 8. or
            not value['lower'][2] < value['takeoff_world_z'] < value['upper'][2]):
        raise ValueError('invalid grid size or altitude bounds')
    if (type(value['max_steps']) is not int or not 4 <= value['max_steps'] <= 80 or
            type(value['land_on_complete']) is not bool or
            value['assignment'] not in ('minimum_distance', 'explicit')):
        raise ValueError('invalid execution or assignment policy')
    targets = value['targets']
    if value['assignment'] == 'explicit':
        if not isinstance(targets, dict) or set(targets) != set(names):
            raise ValueError('explicit targets must identify every manifest vehicle exactly once')
        positions = list(targets.values())
    else:
        if not isinstance(targets, list) or len(targets) != len(names):
            raise ValueError('minimum-distance assignment needs one target per vehicle')
        positions = targets
    if not all(point(target) for target in positions):
        raise ValueError('invalid metric world target')
    if not isinstance(value['obstacles'], list) or len(value['obstacles']) > 32:
        raise ValueError('static obstacle input exceeds bound')
    for box in value['obstacles']:
        if (not isinstance(box, dict) or set(box) != {'lower', 'upper'} or
                not point(box['lower']) or not point(box['upper']) or
                any(a >= b for a, b in zip(box['lower'], box['upper']))):
            raise ValueError('invalid static axis-aligned obstacle')
    return value


def output_directory(path):
    """Create a fresh private evidence directory; never overwrite an earlier execution."""
    root = Path(path)
    if not root.is_absolute() or root.resolve() != root or not root.parent.is_dir():
        raise ValueError('output must be a new absolute physical directory')
    root.mkdir(mode=0o700)
    return root
