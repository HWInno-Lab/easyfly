# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
import math
import pytest
from easyfly_projection_py.core import ImageView, TransformView, camera_model, project, rigid_transform


def fixtures(encoding='rgb8', step=9):
    data = bytes([10, 20, 30] * 3 + [0] * (step-9)) * 3
    return (ImageView(3, 3, encoding, step, data),
            camera_model(3, 3, (1., 0., 1., 0., 1., 1., 0., 0., 1.), (0.,)*5, 'plumb_bob'),
            TransformView((0., 0., 0., 1.), (0., 0., 0.)))


def test_nearest_depth_nan_and_source_xyz_are_independent_of_camera_transform():
    image, model, _ = fixtures()
    transform = TransformView((0., 0., 0., 1.), (1., 0., 0.))
    colored, depth = project([(-1., 0., 4.), (-1., 0., 2.), (0., 0., -1.)], image, model, transform)
    assert colored == [((-1., 0., 2.), (10, 20, 30))]
    assert depth[4] == 2 and math.isnan(depth[0])


def test_bgr_padded_rows_and_first_point_wins_equal_depth():
    image, model, transform = fixtures('bgr8', 12)
    colored, depth = project([(0., 0., 2.), (.1, 0., 2.)], image, model, transform)
    assert colored == [((0., 0., 2.), (30, 20, 10))]
    assert depth[4] == 2


def test_mono_distortion_and_outside_image():
    _, model, transform = fixtures()
    image = ImageView(3, 3, 'mono8', 3, bytes([17]*9))
    assert project([(0., 0., 1.)], image, model, transform)[0][0][1] == (17, 17, 17)
    distorted = camera_model(3, 3, model[2], (.5, 0., 0., 0., 0.), 'plumb_bob')
    assert project([(1., 0., 1.)], image, distorted, transform)[0] == []


def test_bad_camera_layout_and_quaternion():
    image, model, transform = fixtures()
    with pytest.raises(ValueError):
        camera_model(3, 3, (float('nan'),)+model[2][1:], model[3], 'plumb_bob')
    with pytest.raises(ValueError):
        camera_model(3, 3, model[2], model[3], 'equidistant')
    with pytest.raises(ValueError):
        project([(0., 0., 1.)], ImageView(3, 3, 'rgb8', 9, b''), model, transform)
    with pytest.raises(ValueError):
        rigid_transform((0., 0., 0., 0.), (0., 0., 0.))
