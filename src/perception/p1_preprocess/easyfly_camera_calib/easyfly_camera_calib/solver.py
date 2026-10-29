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

"""Solve camera intrinsics and stereo extrinsics with withheld-view and observability gates."""

import cv2
import numpy as np


def object_points(columns, rows, square_m):
    """Use explicit physical square spacing, in metres, for the inner chessboard corners."""
    if not (type(columns) is int and type(rows) is int and
            4 <= columns <= 12 and 4 <= rows <= 10 and .005 <= square_m <= .5):
        raise ValueError('invalid checkerboard geometry')
    points = np.zeros((rows * columns, 3), np.float32)
    points[:, :2] = np.mgrid[:columns, :rows].T.reshape(-1, 2) * square_m
    return points


def detect(gray, columns, rows):
    """Find real subpixel image corners rather than accepting supplied simulator truth."""
    if (gray.dtype != np.uint8 or gray.ndim != 2 or
            not 320 <= gray.shape[1] <= 1280 or not 240 <= gray.shape[0] <= 960):
        raise ValueError('unsupported calibration image shape')
    found, corners = cv2.findChessboardCornersSB(
        gray, (columns, rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    corners = corners.reshape(-1, 2).astype(np.float32)
    if corners[0].sum() > corners[-1].sum():
        corners = corners[::-1].copy()
    if (not np.isfinite(corners).all() or np.min(corners) < 8 or
            np.max(corners[:, 0]) > gray.shape[1] - 8 or
            np.max(corners[:, 1]) > gray.shape[0] - 8):
        return None
    area = cv2.contourArea(cv2.convexHull(corners)) / (gray.shape[0] * gray.shape[1])
    if not .025 <= area <= .75:
        return None
    return corners


def view_descriptor(corners, size):
    """Describe pose diversity in image space, without assuming unknown intrinsic parameters."""
    scale = np.asarray(size, np.float32)
    normalized = corners / scale
    center = np.mean(normalized, axis=0)
    extent = np.ptp(normalized, axis=0)
    edge = normalized[-1] - normalized[0]
    return np.r_[center, extent, edge]


def quality(camera, distortion, stddev, size, rms):
    """Reject ill-conditioned or implausible pinhole solutions even when residuals are small."""
    width, height = size
    values = np.r_[camera.reshape(-1), distortion.reshape(-1), stddev.reshape(-1), rms]
    if not np.isfinite(values).all() or rms > .7:
        raise ValueError('nonfinite or excessive calibration reprojection error')
    if (not .25 * width <= camera[0, 0] <= 4 * width or
            not .25 * height <= camera[1, 1] <= 4 * width or
            not .2 * width <= camera[0, 2] <= .8 * width or
            not .2 * height <= camera[1, 2] <= .8 * height or
            max(abs(distortion.reshape(-1))) > 3.):
        raise ValueError('calibrated pinhole parameters outside supported geometry')
    sigma = stddev.reshape(-1)
    if (sigma[0] > .05 * camera[0, 0] or sigma[1] > .05 * camera[1, 1] or
            sigma[2] > .03 * width or sigma[3] > .03 * height):
        raise ValueError('intrinsic uncertainty indicates insufficient pose diversity')


def reprojection(objects, corners, matrix, distortion):
    """Fit only each held-out board pose and evaluate all measured corner residuals."""
    ok, rotation, translation = cv2.solvePnP(objects, corners, matrix, distortion)
    if not ok or translation[2, 0] <= 0:
        raise ValueError('held-out board pose is not observable in front of the camera')
    pixels, _ = cv2.projectPoints(objects, rotation, translation, matrix, distortion)
    residuals = np.linalg.norm(pixels.reshape(-1, 2) - corners, axis=1)
    return residuals, rotation, translation


def intrinsics(objects, samples, size, training, validation):
    """Keep validation views out of intrinsic estimation and reject nearly parallel boards."""
    views = [samples[index].astype(np.float32).reshape(-1, 1, 2) for index in training]
    output = cv2.calibrateCameraExtended(
        [objects] * len(views), views, size, None, None,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-8))
    rms, matrix, distortion, rotations, translations, stddev, _, per_view = output
    quality(matrix, distortion, stddev, size, rms)
    normals = np.array([cv2.Rodrigues(rotation)[0][:, 2] for rotation in rotations])
    spread = float(np.max(np.arccos(np.clip(normals @ normals.T, -1., 1.))))
    centers = np.array([np.mean(samples[index], axis=0) for index in training])
    if (spread < np.deg2rad(20.) or
            any(np.min(centers, axis=0) > np.asarray(size) * .45) or
            any(np.max(centers, axis=0) < np.asarray(size) * .55)):
        raise ValueError('checkerboard centers must cover both halves of each image axis')
    distances = np.array([float(translation[2, 0]) for translation in translations])
    if np.min(distances) <= 0 or np.max(distances) / np.min(distances) < 1.3:
        raise ValueError('checkerboard distance range must span a factor of at least 1.3')
    residuals = np.concatenate([reprojection(objects, samples[index], matrix, distortion)[0]
                                for index in validation])
    withheld_rms = float(np.sqrt(np.mean(residuals**2)))
    if withheld_rms > .8 or np.percentile(residuals, 95) > 1.5:
        raise ValueError('held-out image reprojection check failed')
    return dict(matrix=matrix.tolist(), distortion=distortion.reshape(-1).tolist(),
                training_rms_px=float(rms), validation_rms_px=withheld_rms,
                validation_p95_px=float(np.percentile(residuals, 95)),
                intrinsic_stddev=stddev.reshape(-1).tolist(),
                board_normal_spread_deg=float(np.rad2deg(spread)),
                per_training_view_rms_px=per_view.reshape(-1).tolist())


def solve(samples, size, board, stereo):
    """Return a reviewable calibrated candidate; no existing file or active TF is changed."""
    if not 20 <= len(samples) <= 40:
        raise ValueError('calibration requires 20 to 40 diverse measured views')
    objects = object_points(*board)
    cameras = ['left', 'right'] if stereo else ['left']
    # A fixed-seed split avoids aliasing a regular board-sweep capture order.
    # Select indices before fitting and never choose them using residuals.
    validation = sorted(np.random.default_rng(2026).choice(
        len(samples), max(4, len(samples) // 5), replace=False).tolist())
    training = [index for index in range(len(samples)) if index not in validation]
    if len(validation) < 4:
        raise ValueError('four independently withheld board views are required')
    for sample in samples:
        for camera in cameras:
            points = np.asarray(sample[camera])
            if points.shape != (len(objects), 2) or not np.isfinite(points).all():
                raise ValueError('invalid measured image corner array')
    result = dict(version=1, units='metre', width=size[0], height=size[1],
                  distortion_model='plumb_bob', board=dict(
                      columns=board[0], rows=board[1], square_m=board[2]),
                  training_indices=training, validation_indices=validation, cameras={})
    for camera in cameras:
        result['cameras'][camera] = intrinsics(
            objects, [np.asarray(sample[camera], np.float32) for sample in samples],
            size, training, validation)
    if not stereo:
        return result
    left, right = result['cameras']['left'], result['cameras']['right']
    k1, d1 = np.array(left['matrix']), np.array(left['distortion'])
    k2, d2 = np.array(right['matrix']), np.array(right['distortion'])
    output = cv2.stereoCalibrate(
        [objects] * len(training),
        [np.asarray(samples[i]['left'], np.float32).reshape(-1, 1, 2) for i in training],
        [np.asarray(samples[i]['right'], np.float32).reshape(-1, 1, 2) for i in training],
        k1, d1, k2, d2, size, flags=cv2.CALIB_USE_INTRINSIC_GUESS,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-8))
    rms, k1, d1, k2, d2, rotation, translation, _, _ = output
    # Jointly fit both independent intrinsics and their shared rigid transform.
    # No simulator geometry, equal-focal constraint, or held-out pixels enter this fit.
    for camera, matrix, distortion in [('left', k1, d1), ('right', k2, d2)]:
        initial = result['cameras'][camera]
        previous = np.array(initial['matrix'])
        if (np.max(abs(np.diag(matrix)[:2] / np.diag(previous)[:2] - 1.)) > .05 or
                np.any(abs(matrix[:2, 2] - previous[:2, 2]) > np.array(size) * .03)):
            raise ValueError('joint stereo fit diverged from observable monocular initialization')
        quality(matrix, distortion, np.array(initial['intrinsic_stddev']), size, rms)
        residuals = np.concatenate([reprojection(
            objects, np.asarray(samples[index][camera], np.float32), matrix, distortion)[0]
            for index in validation])
        heldout_rms = float(np.sqrt(np.mean(residuals**2)))
        if heldout_rms > .8 or np.percentile(residuals, 95) > 1.5:
            raise ValueError('joint intrinsic held-out reprojection check failed')
        # OpenCV exposes covariance estimates for the monocular initialization,
        # not for stereoCalibrate. Retain them under their actual provenance.
        result['cameras'][camera] = dict(
            matrix=matrix.tolist(), distortion=distortion.reshape(-1).tolist(),
            validation_rms_px=heldout_rms,
            validation_p95_px=float(np.percentile(residuals, 95)),
            joint_intrinsic_covariance_available=False,
            monocular_initialization=initial)
    baseline = float(np.linalg.norm(translation))
    if not np.isfinite(rms) or rms > .8 or not .02 <= baseline <= 1.:
        raise ValueError('stereo calibration residual or metric baseline invalid')
    errors = []
    for index in validation:
        _, board_rotation, board_translation = reprojection(
            objects, np.asarray(samples[index]['left'], np.float32), k1, d1)
        transform = rotation @ cv2.Rodrigues(board_rotation)[0]
        vector = rotation @ board_translation + translation
        projected, _ = cv2.projectPoints(objects, cv2.Rodrigues(transform)[0], vector, k2, d2)
        errors.extend(np.linalg.norm(projected.reshape(-1, 2) - samples[index]['right'], axis=1))
    withheld_rms = float(np.sqrt(np.mean(np.asarray(errors)**2)))
    if withheld_rms > 1. or np.percentile(errors, 95) > 2.:
        raise ValueError('held-out cross-camera geometry check failed')
    r1, r2, p1, p2, q, roi1, roi2 = cv2.stereoRectify(
        k1, d1, k2, d2, size, rotation, translation, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.)
    if abs(p2[1, 3]) > 1e-6 or not -.02 >= p2[0, 3] / p2[0, 0] >= -1.:
        raise ValueError('only horizontal left-to-right stereo rigs are supported')
    result['stereo'] = dict(method='joint_intrinsics_and_rigid_extrinsics',
                            right_from_left_rotation=rotation.tolist(),
                            right_from_left_translation_m=translation.reshape(-1).tolist(),
                            baseline_m=baseline, training_rms_px=float(rms),
                            validation_rms_px=withheld_rms,
                            validation_p95_px=float(np.percentile(errors, 95)),
                            left_rectification=r1.tolist(), right_rectification=r2.tolist(),
                            left_projection=p1.tolist(), right_projection=p2.tolist(),
                            disparity_to_depth=q.tolist(), left_valid_roi=list(roi1),
                            right_valid_roi=list(roi2))
    return result
