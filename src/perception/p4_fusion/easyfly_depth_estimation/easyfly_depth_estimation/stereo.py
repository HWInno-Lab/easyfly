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

"""Estimate disparity from real image correspondence and preserve invalid/uncertain regions."""

import cv2
import numpy as np


def calibration(left, right, shape):
    """Require already rectified horizontal pinhole cameras with metric positive baseline."""
    height, width = shape
    for camera in [left, right]:
        if (camera.width != width or camera.height != height or
                camera.binning_x not in (0, 1) or camera.binning_y not in (0, 1) or
                camera.roi.x_offset or camera.roi.y_offset or
                camera.roi.width not in (0, width) or camera.roi.height not in (0, height)):
            raise ValueError('calibration dimensions, crop or binning disagree with images')
        values = np.array([*camera.k, *camera.r, *camera.p, *camera.d])
        if not np.isfinite(values).all() or np.max(np.abs(values)) > 1e5:
            raise ValueError('nonfinite or unbounded camera calibration')
        if (not np.allclose(camera.r, np.eye(3).reshape(-1), atol=1e-8) or
                not np.allclose(camera.d, 0., atol=1e-8) or
                camera.distortion_model not in ('', 'plumb_bob') or camera.roi.do_rectify):
            raise ValueError('inputs must already be rectified with zero residual distortion')
        p = np.asarray(camera.p).reshape(3, 4)
        if (not np.allclose(p[2], [0., 0., 1., 0.], atol=1e-8) or
                not 50 <= p[0, 0] <= 5000 or not 50 <= p[1, 1] <= 5000 or
                not 0 <= p[0, 2] < width or not 0 <= p[1, 2] < height or
                abs(p[0, 1]) > 1e-8 or abs(p[1, 0]) > 1e-8 or abs(p[1, 3]) > 1e-8):
            raise ValueError('invalid rectified pinhole projection')
        if not np.allclose(camera.k, p[:, :3].reshape(-1), rtol=1e-6, atol=1e-6):
            raise ValueError('rectified K and P disagree')
    lp, rp = np.asarray(left.p).reshape(3, 4), np.asarray(right.p).reshape(3, 4)
    if abs(lp[0, 3]) > 1e-8 or not np.allclose(lp[:, :3], rp[:, :3], atol=1e-6):
        raise ValueError('stereo projections require a shared rectified principal point and scale')
    baseline = -rp[0, 3] / rp[0, 0]
    if not .02 <= baseline <= 1.:
        raise ValueError('stereo baseline must be positive and between 0.02 and 1 metre')
    return float(lp[0, 0]), float(baseline)


def grayscale(message):
    """Read valid raw image layouts, including row padding, with bounded dimensions."""
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3}.get(message.encoding, 0)
    if (not channels or not 160 <= message.width <= 1280 or not 96 <= message.height <= 720 or
            not message.width * channels <= message.step <= message.width * channels + 256 or
            len(message.data) != message.step * message.height):
        raise ValueError('unsupported or malformed bounded stereo image')
    pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
    pixels = pixels[:, :message.width * channels]
    if channels == 1:
        return np.ascontiguousarray(pixels)
    pixels = pixels.reshape(message.height, message.width, 3)
    conversion = cv2.COLOR_RGB2GRAY if message.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(pixels, conversion)


class StereoDepth:
    """Use SGBM, bilateral correspondence and texture/uncertainty gates; never invent holes."""

    def __init__(self, disparities=96, minimum_m=.3, maximum_m=20.):
        if (type(disparities) is not int or not 16 <= disparities <= 192 or disparities % 16 or
                not .1 <= minimum_m < maximum_m <= 100.):
            raise ValueError('invalid bounded stereo matcher configuration')
        self.disparities, self.minimum, self.maximum = disparities, minimum_m, maximum_m
        cv2.setNumThreads(2)
        options = dict(numDisparities=disparities, blockSize=5, P1=8 * 25, P2=32 * 25,
                       disp12MaxDiff=1, uniquenessRatio=12, speckleWindowSize=60,
                       speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.left = cv2.StereoSGBM_create(minDisparity=0, **options)
        self.right = cv2.StereoSGBM_create(minDisparity=-disparities, **options)

    def compute(self, left, right, focal, baseline):
        """Return optical-axis metres, disparity pixels and variance in square metres."""
        if (left.shape != right.shape or left.ndim != 2 or left.dtype != np.uint8 or
                right.dtype != np.uint8 or not 96 <= left.shape[0] <= 720 or
                not self.disparities + 64 <= left.shape[1] <= 1280 or
                not 50 <= focal <= 5000 or not .02 <= baseline <= 1.):
            raise ValueError('invalid stereo pair shape, scalar calibration or disparity range')
        dleft = self.left.compute(left, right).astype(np.float32) / 16.
        dright = self.right.compute(right, left).astype(np.float32) / 16.
        yy, xx = np.indices(left.shape)
        rx = np.rint(xx - dleft).astype(np.int32)
        sampled = dright[yy, np.clip(rx, 0, left.shape[1] - 1)]
        floating = left.astype(np.float32)
        mean = cv2.boxFilter(floating, -1, (9, 9))
        texture = np.sqrt(np.maximum(0., cv2.boxFilter(floating**2, -1, (9, 9)) - mean**2))
        floating_right = right.astype(np.float32)
        mean_right = cv2.boxFilter(floating_right, -1, (9, 9))
        texture_right = np.sqrt(np.maximum(0., cv2.boxFilter(
            floating_right**2, -1, (9, 9)) - mean_right**2))
        matched_texture = texture_right[yy, np.clip(rx, 0, left.shape[1] - 1)]
        valid = ((dleft > .5) & (dleft < self.disparities - 1) & (sampled < -.5) &
                 (sampled >= -self.disparities) & (rx >= 0) & (rx < left.shape[1]) &
                 (np.abs(dleft + sampled) <= 1.) & (texture >= 4.) & (matched_texture >= 4.))
        safe_disparity = np.maximum(dleft, .5)
        metric = focal * baseline / safe_disparity
        sigma = focal * baseline * .5 / safe_disparity**2
        valid &= (metric >= self.minimum) & (metric <= self.maximum) & (sigma / metric <= .15)
        depth = np.where(valid, metric, np.nan).astype('<f4')
        variance = np.where(valid, sigma**2, np.nan).astype('<f4')
        disparity = np.where(valid, dleft, np.nan).astype('<f4')
        return depth, disparity, variance
