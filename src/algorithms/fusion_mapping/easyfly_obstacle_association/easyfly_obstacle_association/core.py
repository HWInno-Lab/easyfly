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

"""Pure expanded-AABB matching; caller admits snapshots and owns all timestamps/TTL."""
from collections import Counter
from dataclasses import dataclass


@dataclass(frozen=True)
class Obstacle:
    center: tuple
    size: tuple


@dataclass(frozen=True)
class Target:
    track_id: int
    class_name: str
    confidence: float
    position: tuple


@dataclass(frozen=True)
class Association:
    obstacle_index: int
    target_index: int
    target_id: int
    class_name: str
    confidence: float


def associate(obstacles, targets, margin=0.3, minimum_confidence=0.7):
    matches = {}
    results = []
    uses = Counter()
    for index, obstacle in enumerate(obstacles):
        candidates = []
        for target_index, target in enumerate(targets):
            if (
                target.confidence < minimum_confidence
                or not target.class_name
                or target.class_name == 'unknown'
            ):
                continue
            if all(
                abs(position - center) <= size * 0.5 + margin
                for position, center, size in zip(
                    target.position,
                    obstacle.center,
                    obstacle.size,
                )
            ):
                candidates.append((target_index, target))
                uses[target.track_id] += 1
        if len(candidates) == 1:
            matches[index] = candidates[0]
    for index, (target_index, target) in matches.items():
        # A candidate must be unique in BOTH directions; never arbitrarily pick.
        if uses[target.track_id] != 1:
            continue
        results.append(Association(index, target_index, target.track_id, target.class_name, target.confidence))
    return results
