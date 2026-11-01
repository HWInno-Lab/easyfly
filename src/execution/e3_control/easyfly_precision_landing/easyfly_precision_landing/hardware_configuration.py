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
"""Admission for measured fixed-camera hardware configuration; sends no command."""
import copy
import math
from pathlib import Path
import yaml
from .configuration import validate_document as native_validate
from .geometry import rotation


def validate_document(document):
    """Cross-check all consumers, physical extrinsics and explicit hardware clock."""
    if not isinstance(document, dict):
        raise ValueError('hardware precision requires ROS parameter mapping')
    if set(document) - {
        'landing_transform',
        'apriltag_detection',
        'landing_detection',
        'flight_controller',
    }:
        raise ValueError(
            'hardware precision only accepts explicit consumer scopes; no wildcard or extra nodes'
        )
    required = (
        'landing_transform',
        'apriltag_detection',
        'landing_detection',
        'flight_controller',
    )
    try:
        parts = {key: document[key]['ros__parameters'] for key in required}
        transform, flight = parts['landing_transform'], parts['flight_controller']
        if transform.get('dynamic_camera') is not False:
            raise ValueError('hardware precision requires explicit fixed camera')
        translation = transform['camera_translation_flu']
        if len(translation) != 3 or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in translation
        ):
            raise ValueError('measured camera translation required')
        if rotation(transform['camera_quaternion_xyzw'])[2][2] >= 0:
            raise ValueError('measured optical axis must face downward in body FLU')
        for part in parts.values():
            if part.get('use_sim_time', False) is not False:
                raise ValueError('hardware cannot use simulated time')
        if (
            transform['plane_height_tolerance_m'] != flight['precision_plane_tolerance']
            or transform['plane_tilt_limit_rad'] != flight['precision_tilt_limit']
        ):
            raise ValueError('precision plane/control tolerances mismatch')
        if type(transform['selected_tag_id']) is not int or transform['selected_tag_id'] < 0:
            raise ValueError('explicit physical marker ID required')
        if (
            type(transform['tag_size_m']) not in (int, float)
            or not math.isfinite(transform['tag_size_m'])
            or transform['tag_size_m'] <= 0
        ):
            raise ValueError('measured tag size required')
        if not isinstance(transform['optical_frame'], str) or not transform['optical_frame']:
            raise ValueError('explicit optical frame required')
        if flight.get('precision_moving_target', False) is not False:
            raise ValueError('hardware adapter supports static landing marker only')
        positive = (
            'horizontal_speed',
            'descent_speed',
            'align_tolerance',
            'handoff_height',
            'descent_floor_height',
            'max_height',
            'max_radius',
            'plane_tolerance',
            'tilt_limit',
            'target_jump',
            'target_max_age',
            'acquire_timeout',
            'lost_timeout',
        )
        for key in positive:
            value = flight.get('precision_' + key)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('explicit positive precision control limit required: ' + key)
        limits = {
            'horizontal_speed': 1.0,
            'descent_speed': 0.5,
            'align_tolerance': 0.3,
            'max_height': 10.0,
            'max_radius': 5.0,
            'target_max_age': 1.0,
            'acquire_timeout': 30.0,
            'lost_timeout': 10.0,
        }
        if any(flight['precision_' + key] > limit for key, limit in limits.items()):
            raise ValueError('precision limit exceeds native controller bounds')
        settle = flight.get('precision_settle_observations')
        if (
            type(settle) is not int
            or not 2 <= settle <= 100
            or flight['precision_tilt_limit'] >= math.pi / 4
            or flight['precision_max_height'] <= flight['precision_handoff_height']
            or not math.isfinite(flight['precision_plane_z'])
        ):
            raise ValueError('invalid native precision policy configuration')
        # Reuse native identity/size/frame/height validation, without launching gateway.
        native = copy.deepcopy(document)
        native['landing_gateway'] = {
            'ros__parameters': {'selected_tag_id': transform['selected_tag_id']}
        }
        native_validate(native)
        return document
    except (KeyError, TypeError) as error:
        raise ValueError(
            'explicit hardware marker, extrinsic and control configuration required'
        ) from error


def validate_hardware_configuration(path):
    return validate_document(yaml.safe_load(Path(path).read_text()))
