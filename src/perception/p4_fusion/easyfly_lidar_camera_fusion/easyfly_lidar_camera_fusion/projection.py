# Copyright 2026 EasyFly Project
#
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

"""ROS message decoding/encoding around the independent projection core."""
import math
import struct
from easyfly_projection_py.core import (
    ImageView,
    TransformView,
    camera_model as compute_camera_model,
    image_layout as compute_image_layout,
    rigid_transform as compute_rigid_transform,
    project as project_geometry,
)


def camera_model(info):
    return compute_camera_model(info.width, info.height, info.k, info.d, info.distortion_model)


def image_view(image):
    return ImageView(image.width, image.height, image.encoding, image.step, image.data)


def image_layout(image):
    return compute_image_layout(image_view(image))


def cloud_points(cloud, max_points):
    count = cloud.width * cloud.height
    if (
        count > max_points
        or cloud.point_step < 1
        or cloud.point_step > 256
        or len(cloud.data) > 8388608
    ):
        raise ValueError("cloud budget")
    if (
        cloud.row_step < cloud.width * cloud.point_step
        or len(cloud.data) != cloud.row_step * cloud.height
    ):
        raise ValueError("cloud layout")
    if len(cloud.fields) > 32:
        raise ValueError("cloud fields budget")
    fields = {}
    for field in cloud.fields:
        if field.name in ("x", "y", "z"):
            if field.name in fields or field.count != 1 or field.datatype not in (7, 8):
                raise ValueError("XYZ field type/duplicate")
            size = 4 if field.datatype == 7 else 8
            if field.offset < 0 or field.offset + size > cloud.point_step:
                raise ValueError("XYZ offset")
            fields[field.name] = (
                field.offset,
                size,
                "f" if size == 4 else "d",
            )
    if set(fields) != {"x", "y", "z"}:
        raise ValueError("XYZ missing")
    ordered = sorted((o, o + size) for o, size, _ in fields.values())
    if any(a[1] > b[0] for a, b in zip(ordered, ordered[1:])):
        raise ValueError("overlapping XYZ")
    endian = ">" if cloud.is_bigendian else "<"
    points = []
    for row in range(cloud.height):
        for column in range(cloud.width):
            start = row * cloud.row_step + column * cloud.point_step
            point = tuple(
                struct.unpack_from(endian + fields[key][2], cloud.data, start + fields[key][0])[0]
                for key in ("x", "y", "z")
            )
            if all(math.isfinite(v) and abs(v) <= 100000 for v in point):
                points.append(point)
    return points


def rigid_transform(transform):
    q, t = transform.rotation, transform.translation
    return compute_rigid_transform((q.x, q.y, q.z, q.w), (t.x, t.y, t.z))


def project(points, image, model, transform):
    q, t = transform.rotation, transform.translation
    numeric_transform = TransformView((q.x, q.y, q.z, q.w), (t.x, t.y, t.z))
    colored, depth = project_geometry(points, image_view(image), model, numeric_transform)
    colors = b"".join(
        struct.pack("<fffI", *xyz, (r << 16) | (g << 8) | b) for xyz, (r, g, b) in colored
    )
    return colors, struct.pack("<" + "f" * len(depth), *depth), len(colored)
