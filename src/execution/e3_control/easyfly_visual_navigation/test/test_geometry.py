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

"""Independent pixel/metric and collision checks, including conservative negative cases."""

from types import SimpleNamespace as S

from easyfly_visual_navigation.geometry import advance, blocked, project, visual_target
import numpy as np
import pytest


def time_stamp(seconds):
    """Build a plain source-time record independent of ROS transport."""
    return S(sec=seconds, nanosec=0)


def test_metric_pixels_use_actual_depth_and_transform():
    """Recover physical line/ellipse targets, reject holes and discontinuous depths."""
    depth = np.full((240, 320), 2., np.float32)
    matrix = np.array([[200., 0., 160.], [0., 200., 120.], [0., 0., 1.]])
    rotation = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
    translation = np.array([1., 2., 2.])
    assert np.allclose(project([180., 80.], depth, matrix, rotation, translation), [1.4, 1.8, 0])
    line = S(start_x=160., start_y=200., end_x=160., end_y=40.)
    desired, center, direction = visual_target('line', [line], depth, matrix, rotation,
                                               translation, np.array([1., 2.2, 2.]),
                                               np.array([2., 2., 2.]), 0.)
    assert np.allclose(desired, [1.55, 2., 2.]) and np.allclose(direction, [1., 0.])
    ellipse = S(center_x=180., center_y=80., major_axis_px=40., minor_axis_px=30.)
    desired, _, _ = visual_target('ellipse', [ellipse], depth, matrix, rotation, translation,
                                  np.array([1., 2., 2.]), np.array([1.4, 1.8, 2.]), 0.)
    assert np.allclose(desired, [1.4, 1.8, 2.])
    depth[78:83, 178:183] = np.nan
    with pytest.raises(ValueError, match='depth support'):
        project([180., 80.], depth, matrix, rotation, translation)


def obstacle(center, kind='unknown', associated=False):
    """Create independent finite geometry and its optional source-time association."""
    geometry = S(obstacle_id=3, header=S(frame_id='odom', stamp=time_stamp(10)),
                 center_x=center[0], center_y=center[1], center_z=center[2],
                 size_x=.2, size_y=.2, size_z=.2, is_dynamic=False,
                 velocity_x=0., velocity_y=0., velocity_z=0., obstacle_type=kind)
    return S(obstacle=geometry, association_valid=associated, confidence=.9, target_id=4,
             observed_at=time_stamp(10), expires_at=time_stamp(11))


def test_semantics_and_prediction_only_enlarge_obstacles():
    """Person margins and fresh matched predictions stop motion without deleting raw geometry."""
    start, end = np.array([0., 0., 1.5]), np.array([1., 0., 1.5])
    row = obstacle([.5, .65, 1.5])
    classified = S(geometry_valid=True, obstacles=[row])
    assert not blocked(start, end, classified, [], 10 * 10**9)
    row.association_valid, row.obstacle.obstacle_type = True, 'person'
    assert blocked(start, end, classified, [], 10 * 10**9)[0]['semantic_margin_m'] == .7
    row.obstacle.obstacle_type = 'vehicle'
    prediction = S(valid=True, track_id=4, class_name='vehicle', confidence=.9,
                   header=S(frame_id='odom', stamp=time_stamp(10)),
                   observation_header=S(frame_id='odom', stamp=time_stamp(10)),
                   position_x=.5, position_y=0., position_z=1.5,
                   velocity_x=0., velocity_y=-.65, velocity_z=0.,
                   variance_x=.01, variance_y=.01, variance_z=.01,
                   observation_age_seconds=0., horizon_seconds=1.)
    assert blocked(start, end, classified, [prediction], 10 * 10**9)[0]['prediction_used']
    prediction.observation_header.stamp = time_stamp(9)
    assert not blocked(start, end, classified, [prediction], 10 * 10**9)
    row.obstacle.center_y = 0.
    assert blocked(start, end, classified, [prediction], 10 * 10**9)
    with pytest.raises(ValueError, match='stale'):
        blocked(start, end, classified, [], 11 * 10**9)


def test_velocity_acceleration_and_braking_are_bounded():
    """Bound reference dynamics over acceleration and obstacle braking, not just one step."""
    position, velocity = np.zeros(3), np.zeros(3)
    target = np.array([3., 0., 0.])
    for _ in range(200):
        position, velocity, acceleration = advance(position, velocity, target, .04, .3, .3)
        assert np.linalg.norm(velocity) <= .300001 and np.linalg.norm(acceleration) <= .300001
    before = position.copy()
    for _ in range(40):
        position, velocity, acceleration = advance(position, velocity, target, .04, .3, .3, True)
    assert np.linalg.norm(velocity) < 1e-8 and np.linalg.norm(position - before) < .16
