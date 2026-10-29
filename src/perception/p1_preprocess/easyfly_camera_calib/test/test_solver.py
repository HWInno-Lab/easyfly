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

"""Check recovered metric camera geometry against an independent pinhole projection fixture."""

import cv2
from easyfly_camera_calib.solver import object_points, solve
import numpy as np
import pytest


def measured_corners(parallel=False, narrow_distance=False):
    """Project independent noisy corner measurements through a declared two-camera rig."""
    matrix = np.array([[450., 0., 320.], [0., 455., 240.], [0., 0., 1.]])
    distortion = np.array([-.08, .02, .001, -.0005, .005])
    rotation = cv2.Rodrigues(np.array([.005, -.01, .003]))[0]
    translation = np.array([[-.12], [.002], [.001]])
    objects = object_points(7, 5, .04)
    random = np.random.default_rng(412)
    samples = []
    for index in range(25):
        x, y = (index % 5 - 2) * .085, (index // 5 - 2) * .045
        vector = np.array([.16 * (index % 5 - 2), .17 * (index // 5 - 2),
                           .12 * ((index * 3) % 5 - 2)]) if not parallel else np.zeros(3)
        board_rotation = cv2.Rodrigues(vector)[0]
        distance = .75 if narrow_distance else .62 + .065 * (index % 4)
        position = np.array([[-.12 + x], [-.08 + y], [distance]])
        left = cv2.projectPoints(objects, vector, position, matrix, distortion)[0].reshape(-1, 2)
        right = cv2.projectPoints(objects, cv2.Rodrigues(rotation @ board_rotation)[0],
                                  rotation @ position + translation, matrix, distortion)[0]
        right = right.reshape(-1, 2)
        samples.append(dict(left=left + random.normal(0, .04, left.shape),
                            right=right + random.normal(0, .04, right.shape)))
    return samples, matrix, rotation, translation


def test_intrinsics_stereo_baseline_and_withheld_geometry():
    """Recover focal scale and baseline without exposing true intrinsics to the solver."""
    samples, matrix, rotation, translation = measured_corners()
    candidate = solve(samples, (640, 480), (7, 5, .04), True)
    assert len(candidate['validation_indices']) == 5
    assert not set(candidate['training_indices']) & set(candidate['validation_indices'])
    for camera in candidate['cameras'].values():
        actual = np.asarray(camera['matrix'])
        assert np.max(abs(np.diag(actual)[:2] / np.diag(matrix)[:2] - 1.)) < .02
        assert camera['validation_rms_px'] < .2
    stereo = candidate['stereo']
    assert abs(stereo['baseline_m'] / np.linalg.norm(translation) - 1.) < .02
    assert np.linalg.norm(np.asarray(stereo['right_from_left_rotation']) - rotation) < .01
    assert stereo['validation_p95_px'] < .3


def test_parallel_and_insufficient_views_do_not_produce_calibration():
    """A low training residual alone does not make degenerate camera poses observable."""
    samples, _, _, _ = measured_corners(parallel=True)
    with pytest.raises(ValueError):
        solve(samples, (640, 480), (7, 5, .04), True)
    with pytest.raises(ValueError, match='20 to 40'):
        solve(samples[:10], (640, 480), (7, 5, .04), False)


def test_wrong_pairing_fails_independent_stereo_validation():
    """Unrelated right-camera board poses cannot become a plausible rig transform."""
    samples, _, _, _ = measured_corners()
    for index in range(len(samples)):
        samples[index]['right'] = samples[(index + 7) % len(samples)]['right'].copy()
    with pytest.raises(ValueError):
        solve(samples, (640, 480), (7, 5, .04), True)


def test_single_distance_sweep_is_rejected_and_mono_is_independent():
    """Require depth excitation and allow independent monocular input."""
    samples, _, _, _ = measured_corners(narrow_distance=True)
    with pytest.raises(ValueError, match='distance range'):
        solve(samples, (640, 480), (7, 5, .04), True)
    samples, matrix, _, _ = measured_corners()
    candidate = solve([dict(left=value['left']) for value in samples],
                      (640, 480), (7, 5, .04), False)
    assert 'stereo' not in candidate and set(candidate['cameras']) == {'left'}
    actual = np.array(candidate['cameras']['left']['matrix'])
    assert np.max(abs(np.diag(actual)[:2] / np.diag(matrix)[:2] - 1.)) < .02
