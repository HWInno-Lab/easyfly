# Copyright 2026 EasyFly Project
#
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

import copy
import math
from easyfly_obstacle_association.core import Obstacle, Target, associate
from builtin_interfaces.msg import Time
from easyfly_msgs.msg import ClassifiedObstacle, ClassifiedObstacleArray


def ns(header):
    t = header.stamp
    return (
        t.sec * 1000000000 + t.nanosec
        if t.sec >= 0 and t.nanosec < 1000000000
        else 0
    )


def header_valid(header, now, max_age):
    value = ns(header)
    return (
        header.frame_id == 'odom'
        and value > 0
        and -0.2 <= (now - value) / 1e9 <= max_age
    )


def classify(
    geometry,
    targets,
    now,
    max_age=0.5,
    ttl=0.5,
    margin=0.3,
    minimum_confidence=0.7,
    tolerance=0.15,
):
    output = ClassifiedObstacleArray(
        header=copy.deepcopy(geometry.header), geometry_valid=False
    )
    if len(geometry.obstacles) > 256 or not header_valid(
        geometry.header, now, max_age
    ):
        output.reason = 'invalid/stale geometry header/budget'
        return output
    for obstacle in geometry.obstacles:
        values = (
            obstacle.center_x,
            obstacle.center_y,
            obstacle.center_z,
            obstacle.size_x,
            obstacle.size_y,
            obstacle.size_z,
            obstacle.velocity_x,
            obstacle.velocity_y,
            obstacle.velocity_z,
        )
        if (
            not header_valid(obstacle.header, now, max_age)
            or ns(obstacle.header) != ns(geometry.header)
            or not all(math.isfinite(v) for v in values)
            or min(values[3:6]) < 0
        ):
            output.reason = 'invalid geometry item'
            return output
    output.geometry_valid = True
    output.reason = 'geometry_valid; no fresh unambiguous class association'
    for obstacle in geometry.obstacles:
        item = ClassifiedObstacle(
            obstacle=copy.deepcopy(obstacle), target_id=-1
        )
        item.obstacle.obstacle_type = 'unknown'
        output.obstacles.append(item)
    if len({o.obstacle_id for o in geometry.obstacles}) != len(
        geometry.obstacles
    ):
        output.reason = (
            'duplicate geometry IDs: semantics unavailable, geometry preserved'
        )
        return output
    if (
        targets is None
        or len(targets.targets) > 256
        or not header_valid(targets.header, now, min(max_age, ttl))
    ):
        return output
    if abs(ns(targets.header) - ns(geometry.header)) / 1e9 > tolerance:
        output.reason = 'geometry/target time mismatch; geometry preserved'
        return output
    ids = set()
    for target in targets.targets:
        if (
            target.track_id < 0
            or target.track_id in ids
            or not target.position_valid
            or not header_valid(target.header, now, min(max_age, ttl))
            or ns(target.header) != ns(targets.header)
            or not all(
                math.isfinite(v)
                for v in (
                    target.position_x,
                    target.position_y,
                    target.position_z,
                    target.confidence,
                )
            )
            or not 0 <= target.confidence <= 1
            or len(target.class_name) > 64
        ):
            output.reason = 'invalid target snapshot; geometry preserved'
            return output
        ids.add(target.track_id)
    associations = associate(
        [Obstacle((o.center_x, o.center_y, o.center_z), (o.size_x, o.size_y, o.size_z))
         for o in geometry.obstacles],
        [Target(t.track_id, t.class_name, t.confidence,
                (t.position_x, t.position_y, t.position_z)) for t in targets.targets],
        margin, minimum_confidence,
    )
    for association in associations:
        index = association.obstacle_index
        target = targets.targets[association.target_index]
        item = output.obstacles[index]
        item.obstacle.obstacle_type = target.class_name
        item.target_id, item.confidence, item.association_valid = (
            target.track_id,
            target.confidence,
            True,
        )
        item.observed_at = copy.deepcopy(target.header.stamp)
        expiry = ns(target.header) + round(ttl * 1e9)
        item.expires_at = Time(
            sec=expiry // 1000000000, nanosec=expiry % 1000000000
        )
    if any(item.association_valid for item in output.obstacles):
        output.reason = (
            'fresh one-to-one semantic association; geometry preserved'
        )
    return output
