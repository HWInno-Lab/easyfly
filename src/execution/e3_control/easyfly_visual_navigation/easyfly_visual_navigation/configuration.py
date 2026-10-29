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

"""Freeze one small explicit visual navigation request."""

import json
import os
from pathlib import Path
import re
import stat

import numpy as np

from .geometry import finite_vector


def read(path):
    """Read one bounded regular configuration without following its final symlink."""
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or not 0 < info.st_size <= 16384:
            raise ValueError('task must be a small regular JSON file')
        with os.fdopen(descriptor, closefd=False) as stream:
            return json.loads(stream.read(16385))
    finally:
        os.close(descriptor)


def configuration(path):
    """Require explicit metric goal, corridor, source bindings and control budgets."""
    value = read(Path(path))
    fields = {'version', 'mode', 'use_sim_time', 'target', 'plane_z', 'corridor_min',
              'corridor_max', 'max_speed', 'max_acceleration', 'timeout_s', 'optical_frame',
              'observation_topic', 'depth_topic', 'camera_info_topic', 'use_predictions'}
    if (set(value) != fields or value['version'] != 1 or
            value['mode'] not in ('line', 'ellipse') or
            type(value['use_sim_time']) is not bool or
            type(value['use_predictions']) is not bool or
            not .1 <= value['max_speed'] <= .5 or
            not .1 <= value['max_acceleration'] <= .5 or
            not 10 <= value['timeout_s'] <= 180 or
            not -10 <= value['plane_z'] <= 10):
        raise ValueError('invalid bounded visual task configuration')
    goal = finite_vector(value['target'], 3)
    low, high = finite_vector(value['corridor_min'], 3), finite_vector(value['corridor_max'], 3)
    if (np.any(high <= low) or np.any(high - low > 20) or np.any(goal < low) or
            np.any(goal > high) or not .8 <= goal[2] <= 5):
        raise ValueError('invalid metric goal or local control corridor')
    for key in ['observation_topic', 'depth_topic', 'camera_info_topic']:
        if not re.fullmatch('/[A-Za-z_][A-Za-z0-9_/]{0,190}', value[key]):
            raise ValueError('invalid absolute source topic')
    if len({value[key] for key in ['observation_topic', 'depth_topic', 'camera_info_topic']}) != 3:
        raise ValueError('visual source topics must differ')
    if not re.fullmatch('[A-Za-z_][A-Za-z0-9_/]{0,110}', value['optical_frame']):
        raise ValueError('invalid optical frame')
    return value
