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
"""Measured fixed-camera adapter validation without hardware or flight."""
from pathlib import Path

import pytest
import yaml

from easyfly_precision_landing.hardware_configuration import validate_document

PACKAGE = Path(__file__).resolve().parents[1]


def measured_fixture():
    # Numerical fixture only; not a hardware default.
    document = yaml.safe_load((PACKAGE / 'config/d5_simulation.yaml').read_text())
    document.pop('landing_gateway')
    document['landing_transform']['ros__parameters']['dynamic_camera'] = False
    return document


def test_complete_measured_fixture_and_no_gateway_requirement():
    document = measured_fixture()
    assert validate_document(document) is document


@pytest.mark.parametrize(
    'section,key,value',
    [
        ('landing_transform', 'dynamic_camera', True),
        ('landing_transform', 'camera_quaternion_xyzw', [0.0, 0.0, 0.0, 0.0]),
        ('landing_transform', 'camera_translation_flu', []),
        ('landing_transform', 'tag_size_m', -1.0),
        ('flight_controller', 'precision_plane_tolerance', 0.2),
        ('flight_controller', 'precision_horizontal_speed', 2.0),
        ('apriltag_detection', 'use_sim_time', True),
    ],
)
def test_inconsistent_physical_configuration_rejected(section, key, value):
    document = measured_fixture()
    document[section]['ros__parameters'][key] = value
    with pytest.raises(ValueError):
        validate_document(document)


def test_unfilled_hardware_template_fails_closed():
    document = yaml.safe_load((PACKAGE / 'config/hardware_template.yaml').read_text())
    with pytest.raises(ValueError):
        validate_document(document)
