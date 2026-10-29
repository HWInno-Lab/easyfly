# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
import base64
import numpy as np
from bspline_metrics import polynomial_pieces, value, derivative_maximum, MapCertificate


def line():
    return dict(order=3, knots=[0., 0., 0., 0., 1., 1., 1., 1.],
                position_points=[[0., 0., 1.3], [1/3, 0., 1.3], [2/3, 0., 1.3], [1., 0., 1.3]])


def test_native_clamped_cubic_linear_identity_and_exact_derivatives():
    pieces = polynomial_pieces(line())
    for t in np.linspace(0., 1., 13):
        assert np.allclose(value(pieces, t), [t, 0., 1.3], atol=1e-12)
        assert np.allclose(value(pieces, t, 1), [1., 0., 0.], atol=1e-12)
    assert abs(derivative_maximum(pieces, 1)-1.) < 1e-12
    assert derivative_maximum(pieces, 2) < 1e-12


def test_unclamped_uniform_native_knots_preserve_multiple_span_time_origin():
    # Uniform cubic cardinal B-splines reproduce this affine function exactly.
    # Negative exterior knots and three interior spans match native EGO form.
    plan = dict(order=3, knots=list(np.arange(-3., 7.)),
                position_points=[[float(i), .25*float(i), 1.3] for i in range(6)])
    pieces = polynomial_pieces(plan)
    assert [p['start'] for p in pieces] == [0., 1., 2.]
    for t in np.linspace(0., 3., 25):
        assert np.allclose(value(pieces, t), [t+1., .25*(t+1.), 1.3], atol=1e-12)
        assert np.allclose(value(pieces, t, 1), [1., .25, 0.], atol=1e-12)
    assert abs(derivative_maximum(pieces, 1)-np.hypot(1., .25)) < 1e-12


def test_unknown_between_free_endpoints_cannot_pass_common_map_certificate():
    cells = np.zeros((18, 30, 30), dtype=np.int8)
    message = dict(valid=True, frame='odom', inflation=.65, size=[30, 30, 18],
                   origin=[-2., -3., 0.], resolution=.2, data_b64=base64.b64encode(cells).decode())
    assert MapCertificate(message).trajectory(polynomial_pieces(line()))
    cells[6, 15, 12] = -1
    message['data_b64'] = base64.b64encode(cells).decode()
    assert not MapCertificate(message).trajectory(polynomial_pieces(line()))


def test_conversion_roundoff_cannot_allow_contact_with_an_unknown_cell():
    plan = line()
    for point in plan['position_points']:
        point[2] = 1.2
    cells = np.zeros((18, 30, 30), dtype=np.int8)
    cells[6, 15, 12] = -1
    message = dict(valid=True, frame='odom', inflation=.65, size=[30, 30, 18],
                   origin=[-2., -3., 0.], resolution=.2, data_b64=base64.b64encode(cells).decode())
    assert not MapCertificate(message).trajectory(polynomial_pieces(plan))
