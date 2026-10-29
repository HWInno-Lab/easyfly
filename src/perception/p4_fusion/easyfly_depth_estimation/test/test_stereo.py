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

"""Measure depth from independent image disparities; never feed target depth to the matcher."""

import copy

from easyfly_depth_estimation.stereo import calibration, StereoDepth
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo


def camera(width=384, height=192, baseline=0.):
    """Create known rectified calibration for projected-image tests."""
    info = CameraInfo(width=width, height=height, distortion_model='plumb_bob')
    info.k = [240., 0., width / 2, 0., 240., height / 2, 0., 0., 1.]
    info.p = [240., 0., width / 2, -240. * baseline,
              0., 240., height / 2, 0., 0., 0., 1., 0.]
    info.r = np.eye(3).reshape(-1).tolist()
    info.d = [0.] * 5
    return info


def texture_pair(disparity):
    """Render two independent pinhole image planes with a known integer parallax."""
    texture = np.random.default_rng(713).integers(0, 256, (192, 384 + disparity), dtype=np.uint8)
    return texture[:, :384].copy(), texture[:, disparity:].copy()


@pytest.mark.parametrize('disparity', [6, 12, 24])
def test_metric_scale_and_variance_follow_stereo_geometry(disparity):
    """Across 2, 4 and 8 metres, match textured correspondences and retain actual metric scale."""
    left, right = texture_pair(disparity)
    fx, baseline = calibration(camera(), camera(baseline=.2), left.shape)
    depth, pixels, variance = StereoDepth(disparities=64).compute(left, right, fx, baseline)
    central = np.zeros_like(depth, dtype=bool)
    central[12:-12, 80:-80] = True
    valid = central & np.isfinite(depth)
    expected = fx * baseline / disparity
    assert valid.sum() / central.sum() > .95
    assert np.percentile(np.abs(depth[valid] - expected), 95) < .03
    assert np.median(pixels[valid]) == pytest.approx(disparity, abs=.1)
    assert np.median(variance[valid]) == pytest.approx(
        (fx * baseline * .5 / disparity**2)**2, rel=.03)


def test_uniform_texture_and_occlusion_remain_unknown():
    """Matching does not synthesize valid depth on blank images or unsupported occlusion."""
    matcher = StereoDepth(disparities=64)
    blank = np.full((192, 384), 127, dtype=np.uint8)
    depth, _, _ = matcher.compute(blank, blank, 240., .2)
    assert not np.isfinite(depth).any()
    left, right = texture_pair(12)
    right[:, 140:220] = 127
    depth, _, _ = matcher.compute(left, right, 240., .2)
    assert np.isfinite(depth[15:-15, 158:222]).mean() < .05


def test_calibration_rejects_negative_baseline_uncorrected_optics_and_crop():
    """Wrong geometry is rejected before a plausible-looking depth scale can be published."""
    left, right = camera(), camera(baseline=.2)
    for changed in ['baseline', 'distortion', 'rotation', 'binning', 'principal_point', 'nan']:
        value = copy.deepcopy(right)
        if changed == 'baseline':
            value.p[3] = 48.
        elif changed == 'distortion':
            value.d[0] = .1
        elif changed == 'rotation':
            value.r[0] = .99
        elif changed == 'binning':
            value.binning_x = 2
        elif changed == 'principal_point':
            value.p[2] += 3.
        else:
            value.p[0] = float('nan')
        with pytest.raises(ValueError):
            calibration(left, value, (192, 384))
