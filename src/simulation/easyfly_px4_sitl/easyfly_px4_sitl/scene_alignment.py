# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Compare measured world-to-navigation transforms over an explicit domain.

This is qualification geometry, not an estimator, controller or timestamp source.
The default preserves the historical world-origin translation comparison.
"""
import itertools
import math


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('alignment requires finite numbers')
    return value


def _point(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('alignment requires three-dimensional points')
    return [_number(v) for v in value]


def _transform(value):
    if not isinstance(value, dict) or set(value) != {'translation', 'yaw'}:
        raise ValueError('alignment transform requires translation and yaw')
    return _point(value['translation']), _number(value['yaw'])


def _corners(region):
    if not isinstance(region, dict) or set(region) != {'min', 'max'}:
        raise ValueError('recommended-region alignment requires explicit region bounds')
    low, high = _point(region['min']), _point(region['max'])
    if any(a >= b for a, b in zip(low, high)):
        raise ValueError('invalid recommended region bounds')
    return list(itertools.product(*zip(low, high)))


def validate_targets(region, targets):
    """Require every reviewed airborne mission target inside the checked domain."""
    _corners(region)
    if not targets:
        raise ValueError('region alignment requires explicit mission targets')
    for target in targets:
        if any(v < a or v > b for v, a, b in zip(_point(target), region['min'], region['max'])):
            raise ValueError('reference mission target outside recommended alignment region')


def compare_alignment(reference, measured, *, policy='world_origin', region=None):
    """Return position/yaw errors without choosing or changing admission limits.

    For recommended_region, transform each fixed reference-navigation corner
    back into world, then project that same world point with the measurement.
    The maximum corner norm also bounds this affine error throughout the box.
    Changing the world coordinate origin does not change those physical errors.
    """
    rt, ry = _transform(reference)
    mt, my = _transform(measured)
    samples = []
    if policy == 'world_origin':
        samples.append(dict(world_point=[0., 0., 0.], reference_navigation=rt,
                            measured_navigation=mt, error_m=math.dist(rt, mt)))
    elif policy == 'recommended_region':
        for corner in _corners(region):
            x, y, z = [v-t for v, t in zip(corner, rt)]
            c, s = math.cos(ry), math.sin(ry)
            world = [c*x+s*y, -s*x+c*y, z]
            c, s = math.cos(my), math.sin(my)
            projected = [c*world[0]-s*world[1]+mt[0], s*world[0]+c*world[1]+mt[1], world[2]+mt[2]]
            samples.append(dict(world_point=world, reference_navigation=list(corner),
                                measured_navigation=projected, error_m=math.dist(corner, projected)))
    else:
        raise ValueError('unknown alignment comparison policy')
    return dict(policy=policy, maximum_position_error_m=max(v['error_m'] for v in samples),
                yaw_error_rad=abs(math.atan2(math.sin(ry-my), math.cos(ry-my))),
                translation_component_error_m=math.dist(rt, mt), samples=samples)
