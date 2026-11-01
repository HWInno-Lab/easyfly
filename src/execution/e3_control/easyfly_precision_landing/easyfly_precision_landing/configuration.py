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
"""Shared native precision configuration checks."""
from pathlib import Path
import math
import yaml


def validate_configuration(path):
    """Require consistent size/identity/frame/plane across all consumers."""
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return validate_document(document)


def validate_document(document):
    """Validate an already parsed native precision configuration."""
    keys = (
        "landing_transform",
        "apriltag_detection",
        "landing_detection",
        "landing_gateway",
        "flight_controller",
    )
    if not isinstance(document, dict):
        raise ValueError("expected explicit D5 ROS parameter mapping")
    parts = [document[key]["ros__parameters"] for key in keys]
    transform, tag, detection, gateway, flight = parts
    floor = flight.get("precision_descent_floor_height")
    handoff = flight.get("precision_handoff_height")
    if (
        not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            for value in (floor, handoff)
        )
        or not 0.15 <= floor < handoff <= 1.0
    ):
        raise ValueError("explicit valid descent floor/handoff band required")
    if transform.get("configured") is not True or flight.get("precision_enabled") is not True:
        raise ValueError("D5 must be explicitly configured/enabled")
    if (
        transform["selected_tag_id"] != detection["selected_tag_id"]
        or transform["selected_tag_id"] != gateway["selected_tag_id"]
        or transform["selected_tag_id"] != flight["precision_tag_id"]
        or transform["tag_size_m"] != tag["tag_size_m"]
        or transform["tag_size_m"] != flight["precision_tag_size_m"]
        or transform["optical_frame"] != tag["optical_frame"]
        or transform["optical_frame"] != detection["optical_frame"]
        or transform["optical_frame"] != flight["precision_optical_frame"]
        or transform["landing_plane_z_m"] != flight["precision_plane_z"]
    ):
        raise ValueError("D5 tag/frame/size/plane configuration mismatch")
    dynamic = transform.get('dynamic_camera', False)
    if type(dynamic) is not bool:
        raise ValueError('dynamic_camera must be explicit boolean')
    if dynamic:
        from easyfly_precision_landing.extrinsics import measured_mount

        mount = document.get('landing_camera_mount', {}).get('ros__parameters', {})
        if (
            mount.get('configured') is not True
            or not mount.get('joint_name')
            or mount.get('optical_frame') != transform['optical_frame']
        ):
            raise ValueError('explicit matching measured camera mount required')
        measured_mount(
            0.0,
            mount.get('pivot_translation_flu', []),
            mount.get('optical_offset_flu_at_zero', []),
            mount.get('joint_axis_flu', []),
            mount.get('camera_quaternion_xyzw_zero', []),
        )
    return document
