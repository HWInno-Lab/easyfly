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

"""Metric image geometry and conservative semantic/prediction control envelopes."""

import math

import numpy as np


def finite_vector(value, length):
    """Require an exact finite numeric vector."""
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError('invalid finite vector')
    return result


def project(pixel, depth, matrix, rotation, translation):
    """Use observed optical Z from a supported local depth patch, never a nominal ground range."""
    x, y = finite_vector(pixel, 2)
    height, width = depth.shape
    if not 3 <= x < width - 3 or not 3 <= y < height - 3:
        raise ValueError('visual point too close to image boundary')
    col, row = int(round(x)), int(round(y))
    patch = depth[row - 2:row + 3, col - 2:col + 3]
    values = patch[np.isfinite(patch) & (patch >= .3) & (patch <= 15.)]
    if len(values) < 20:
        raise ValueError('visual point lacks measured depth support')
    z = float(np.median(values))
    if np.percentile(values, 90) - np.percentile(values, 10) > .08:
        raise ValueError('visual depth crosses a surface discontinuity')
    ray = np.array([(x - matrix[0, 2]) / matrix[0, 0],
                    (y - matrix[1, 2]) / matrix[1, 1], 1.])
    return rotation @ (ray * z) + translation


def visual_target(mode, observations, depth, matrix, rotation, translation, position, target,
                  plane_z):
    """Choose one unambiguous measured line/ellipse tied to the requested bounded route."""
    if not 1 <= len(observations) <= 32:
        raise ValueError('visual target missing or observation budget exceeded')
    candidates = []
    for observation in observations:
        try:
            if mode == 'line':
                first = finite_vector([observation.start_x, observation.start_y], 2)
                last = finite_vector([observation.end_x, observation.end_y], 2)
                # A line may leave the image; use measured interior depth, not border pixels.
                start = project(.9 * first + .1 * last, depth, matrix, rotation, translation)
                end = project(.1 * first + .9 * last, depth, matrix, rotation, translation)
                if max(abs(start[2] - plane_z), abs(end[2] - plane_z)) > .15:
                    continue
                delta = end[:2] - start[:2]
                length = np.linalg.norm(delta)
                if not .2 <= length <= 10.:
                    continue
                direction = delta / length
                if direction @ (target[:2] - position[:2]) < 0:
                    direction *= -1
                foot = start[:2] + direction * ((position[:2] - start[:2]) @ direction)
                target_foot = start[:2] + direction * ((target[:2] - start[:2]) @ direction)
                if np.linalg.norm(target_foot - target[:2]) > .3:
                    continue
                lookahead = min(.55, np.linalg.norm(target[:2] - foot))
                desired = np.r_[foot + direction * lookahead, target[2]]
                candidates.append((float(np.linalg.norm(position[:2] - foot)), desired,
                                   np.r_[foot, plane_z], direction))
            else:
                # Require the complete target plus a small image margin before moving.
                radius = max(observation.major_axis_px, observation.minor_axis_px) / 2.
                if (not np.isfinite(radius) or radius < 6. or
                        min(observation.center_x - radius,
                            observation.center_y - radius) < 24. or
                        observation.center_x + radius >= depth.shape[1] - 24. or
                        observation.center_y + radius >= depth.shape[0] - 24.):
                    continue
                center = project([observation.center_x, observation.center_y], depth,
                                 matrix, rotation, translation)
                if abs(center[2] - plane_z) > .15:
                    continue
                distance = float(np.linalg.norm(center[:2] - target[:2]))
                if distance <= .3:
                    candidates.append((distance, np.r_[center[:2], target[2]], center, None))
        except (ValueError, TypeError, OverflowError):
            continue
    if not candidates:
        raise ValueError('no measured visual geometry matches this route')
    candidates.sort(key=lambda row: row[0])
    best = candidates[0]
    for row in candidates[1:]:
        if row[0] - best[0] < .2 and np.linalg.norm(row[1] - best[1]) > .15:
            raise ValueError('ambiguous competing visual routes')
    return best[1], best[2], best[3]


def segment_box(start, end, lower, upper):
    """Intersect the complete closed segment with an axis-aligned obstacle envelope."""
    low, high = 0., 1.
    for axis in range(3):
        delta = end[axis] - start[axis]
        if abs(delta) < 1e-10:
            if start[axis] < lower[axis] or start[axis] > upper[axis]:
                return False
            continue
        first, last = sorted(((lower[axis] - start[axis]) / delta,
                              (upper[axis] - start[axis]) / delta))
        low, high = max(low, first), min(high, last)
        if low > high:
            return False
    return True


def stamp(value):
    """Read a canonical nonnegative source time."""
    if value.sec < 0 or not 0 <= value.nanosec < 10**9:
        raise ValueError('invalid source stamp')
    return value.sec * 10**9 + value.nanosec


def blocked(start, end, classified, predictions, clock_ns):
    """Keep all geometry and enlarge envelopes for fresh semantics and linked predictions."""
    if not classified.geometry_valid or len(classified.obstacles) > 128:
        raise ValueError('semantic geometry invalid or oversized')
    if len(predictions) > 128:
        raise ValueError('prediction budget exceeded')
    seen, reasons = set(), []
    for row in classified.obstacles:
        obstacle = row.obstacle
        if obstacle.obstacle_id in seen or obstacle.header.frame_id != 'odom':
            raise ValueError('duplicate or foreign obstacle geometry')
        seen.add(obstacle.obstacle_id)
        if not 0 <= (clock_ns - stamp(obstacle.header.stamp)) / 1e9 <= .5:
            raise ValueError('stale obstacle geometry')
        center = finite_vector([obstacle.center_x, obstacle.center_y, obstacle.center_z], 3)
        size = finite_vector([obstacle.size_x, obstacle.size_y, obstacle.size_z], 3)
        velocity = finite_vector([obstacle.velocity_x, obstacle.velocity_y,
                                  obstacle.velocity_z], 3)
        if np.any(size <= 0.) or np.any(size > 20.) or np.linalg.norm(velocity) > 8.:
            raise ValueError('unsupported obstacle geometry or motion')
        associated = (row.association_valid and .7 <= row.confidence <= 1. and
                      stamp(row.observed_at) <= clock_ns < stamp(row.expires_at) and
                      0 <= (clock_ns - stamp(row.observed_at)) / 1e9 <= .5)
        margin = .7 if associated and obstacle.obstacle_type == 'person' else .35
        lower, upper = center - size / 2 - margin, center + size / 2 + margin
        # Existing geometry is never cleared or shrunk by a class or a prediction.
        if obstacle.is_dynamic:
            displacement = velocity * 1.5
            lower, upper = np.minimum(lower, lower + displacement), np.maximum(
                upper, upper + displacement)
        used_prediction = False
        for predicted in predictions:
            if not associated or predicted.track_id != row.target_id:
                continue
            values = finite_vector([predicted.position_x, predicted.position_y,
                                    predicted.position_z, predicted.velocity_x,
                                    predicted.velocity_y, predicted.velocity_z,
                                    predicted.variance_x, predicted.variance_y,
                                    predicted.variance_z], 9)
            if (not predicted.valid or predicted.header.frame_id != 'odom' or
                    predicted.observation_header.frame_id != 'odom' or
                    stamp(predicted.observation_header.stamp) != stamp(row.observed_at) or
                    predicted.class_name != obstacle.obstacle_type or
                    not .7 <= predicted.confidence <= 1. or
                    not 0 <= (clock_ns - stamp(predicted.header.stamp)) / 1e9 <= .5 or
                    not 0 <= predicted.observation_age_seconds <= .5 or
                    not 0 <= predicted.horizon_seconds <= 1.5 or
                    np.any(values[6:] < 0) or np.any(values[6:] > 1.) or
                    np.linalg.norm(values[3:6]) > 8.):
                continue
            future = values[:3]
            radius = size / 2 + margin + 2 * np.sqrt(values[6:])
            lower, upper = np.minimum(lower, future - radius), np.maximum(upper, future + radius)
            used_prediction = True
        if segment_box(start, end, lower, upper):
            reasons.append(dict(obstacle_id=obstacle.obstacle_id, semantic_margin_m=margin,
                                prediction_used=used_prediction))
    return reasons


def advance(position, velocity, desired, dt, max_speed, max_acceleration, stop=False):
    """Advance a low-speed reference with explicit velocity and acceleration bounds."""
    if not 0 < dt <= .1:
        raise ValueError('control source-time step invalid')
    delta = desired - position
    speed = min(max_speed, math.sqrt(2 * max_acceleration * np.linalg.norm(delta)),
                np.linalg.norm(delta) * 1.2)
    wanted = np.zeros(3) if stop or np.linalg.norm(delta) < 1e-8 else (
        delta / np.linalg.norm(delta) * speed)
    change = wanted - velocity
    limit = max_acceleration * dt
    if np.linalg.norm(change) > limit:
        change *= limit / np.linalg.norm(change)
    following = velocity + change
    return position + (velocity + following) * dt / 2., following, change / dt
