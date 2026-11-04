# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_lidar_camera_fusion/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

import math
import struct
import unittest
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Transform
from easyfly_lidar_camera_fusion.projection import (
    camera_model,
    image_layout,
    cloud_points,
    project,
)


def fixtures():
    image = Image(
        width=3,
        height=3,
        encoding='rgb8',
        step=9,
        data=bytes([10, 20, 30] * 9),
    )
    info = CameraInfo(
        width=3,
        height=3,
        k=[1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0],
        distortion_model='plumb_bob',
        d=[0.0] * 5,
    )
    transform = Transform()
    transform.rotation.w = 1.0
    return image, info, transform


class Projection(unittest.TestCase):
    def test_known_pixel_color_depth_and_occlusion(self):
        image, info, transform = fixtures()
        colored, depth, count = project(
            [
                (0.0, 0.0, 4.0),
                (0.0, 0.0, 2.0),
                (1.0, 0.0, 1.0),
                (0.0, 0.0, -1.0),
            ],
            image,
            camera_model(info),
            transform,
        )
        self.assertEqual(count, 2)
        x, y, z, rgb = struct.unpack_from('<fffI', colored)
        self.assertEqual((x, y, z), (0.0, 0.0, 2.0))
        self.assertEqual(rgb, 0x0A141E)
        values = struct.unpack('<9f', depth)
        self.assertEqual(values[4], 2.0)
        self.assertEqual(values[5], 1.0)
        self.assertTrue(math.isnan(values[0]))

    def test_rigid_translation_distortion_and_bgr(self):
        image, info, transform = fixtures()
        image.encoding = 'bgr8'
        transform.translation.x = 1.0
        colored, depth, count = project(
            [(0.0, 0.0, 1.0)], image, camera_model(info), transform
        )
        self.assertEqual(count, 1)
        self.assertEqual(struct.unpack_from('<fffI', colored)[3], 0x1E140A)
        self.assertEqual(struct.unpack('<9f', depth)[5], 1.0)
        info.d[0] = 0.5
        self.assertEqual(
            project([(0.0, 0.0, 1.0)], image, camera_model(info), transform)[
                2
            ],
            0,
        )

    def test_bad_camera_image_tf_and_layout(self):
        image, info, transform = fixtures()
        info.k[0] = float('nan')
        with self.assertRaises(ValueError):
            camera_model(info)
        image.data = bytes([1])
        with self.assertRaises(ValueError):
            image_layout(image)
        image, info, transform = fixtures()
        transform.rotation.w = 0.0
        with self.assertRaises(ValueError):
            project([(0.0, 0.0, 1.0)], image, camera_model(info), transform)

    def test_cloud_padded_rows_big_endian_invalid_fields_and_budget(self):
        cloud = PointCloud2(
            width=1,
            height=2,
            point_step=12,
            row_step=16,
            is_bigendian=True,
            fields=[
                PointField(name=n, offset=i * 4, datatype=7, count=1)
                for i, n in enumerate(('x', 'y', 'z'))
            ],
            data=struct.pack('>fffIfffI', 1.0, 2.0, 3.0, 0, 4.0, 5.0, 6.0, 0),
        )
        self.assertEqual(
            cloud_points(cloud, 2), [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        )
        with self.assertRaises(ValueError):
            cloud_points(cloud, 1)
        cloud.fields[1].offset = 2
        with self.assertRaises(ValueError):
            cloud_points(cloud, 2)


if __name__ == '__main__':
    unittest.main()
