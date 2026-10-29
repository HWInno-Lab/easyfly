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

"""Prioritized space-time A* with conservative asynchronous segment reservations."""

import heapq
import itertools
import math
import time


def subtract(a, b):
    """Subtract finite three-dimensional vectors."""
    return tuple(x - y for x, y in zip(a, b))


def dot(a, b):
    """Compute a Euclidean inner product."""
    return sum(x * y for x, y in zip(a, b))


def point_segment_distance(point, start, end):
    """Return exact distance to the closed segment, including a stationary vehicle."""
    edge = subtract(end, start)
    length = dot(edge, edge)
    t = max(0., min(1., dot(subtract(point, start), edge) / length)) if length else 0.
    return math.dist(point, tuple(x + t * d for x, d in zip(start, edge)))


def segment_distance(a, b, c, d):
    """Bound arbitrary relative progress, rather than assuming synchronized endpoint arrival."""
    result = min(point_segment_distance(p, s, e)
                 for p, s, e in [(a, c, d), (b, c, d), (c, a, b), (d, a, b)])
    u, v, w = subtract(b, a), subtract(d, c), subtract(a, c)
    uu, vv, uv, uw, vw = dot(u, u), dot(v, v), dot(u, v), dot(u, w), dot(v, w)
    denominator = uu * vv - uv * uv
    if denominator > 1e-12:
        s, t = (uv * vw - vv * uw) / denominator, (uu * vw - uv * uw) / denominator
        if 0 <= s <= 1 and 0 <= t <= 1:
            result = min(result, math.dist(tuple(x + s * y for x, y in zip(a, u)),
                                           tuple(x + t * y for x, y in zip(c, v))))
    return result


def intersects_box(start, end, lower, upper):
    """Use closed slab intersection, treating contact with an inflated obstacle as blocked."""
    enter, leave = 0., 1.
    for a, b, low, high in zip(start, end, lower, upper):
        delta = b - a
        if abs(delta) < 1e-12:
            if not low <= a <= high:
                return False
        else:
            left, right = sorted(((low - a) / delta, (high - a) / delta))
            enter, leave = max(enter, left), min(leave, right)
            if enter > leave:
                return False
    return True


def assign(starts, targets):
    """Compute exact minimum Euclidean assignment for at most four interchangeable targets."""
    names = sorted(starts)
    if len(names) != len(targets) or not 2 <= len(names) <= 4:
        raise ValueError('assignment requires two to four vehicles and the same number of targets')
    best = min(itertools.permutations(range(len(targets))),
               key=lambda order: (sum(math.dist(starts[n], targets[j])
                                      for n, j in zip(names, order)), order))
    return {name: tuple(targets[index]) for name, index in zip(names, best)}


class Planner:
    """Reserve swept segments; execution awaits every vehicle at each phase boundary."""

    def __init__(self, config):
        self.config = config
        self.lower, self.upper = config['lower'], config['upper']
        self.step = config['grid_m']
        self.tube = config['tracking_tube_m']
        self.clearance = config['separation_m'] + 2 * self.tube
        self.obstacles = config['obstacles']
        self.max_steps = config['max_steps']
        self.expanded = 0
        self.deadline = 0.

    def point(self, cell):
        """Map the declared grid to world metres."""
        return tuple(low + index * self.step for low, index in zip(self.lower, cell))

    def cell(self, point):
        """Snap only a short initial/final connection, which is independently collision checked."""
        return tuple(round((v - low) / self.step) for v, low in zip(point, self.lower))

    def edge_free(self, start, end):
        """Keep each vehicle tracking tube within bounds and outside inflated boxes."""
        if any(not low + self.tube <= x <= high - self.tube
               for p in [start, end] for x, low, high in zip(p, self.lower, self.upper)):
            return False
        inflate = self.config['vehicle_radius_m'] + self.tube
        return not any(intersects_box(start, end, [v - inflate for v in box['lower']],
                                      [v + inflate for v in box['upper']])
                       for box in self.obstacles)

    @staticmethod
    def at(path, phase):
        """Keep a completed path occupying its final position."""
        return path[min(phase, len(path) - 1)]

    def available(self, start, end, phase, reservations):
        """Reject vertex conflicts and asynchronously overlapping swept segments."""
        return self.edge_free(start, end) and all(
            segment_distance(start, end, self.at(path, phase), self.at(path, phase + 1)) >=
            self.clearance - 1e-9 for path in reservations.values())

    def route(self, start, goal, reservations):
        """Find a bounded path with reserved phases and exact endpoint connections."""
        first, last = self.cell(start), self.cell(goal)
        first_point = self.point(first)
        if not self.available(start, first_point, 0, reservations):
            return None
        frontier = [(sum(abs(a - b) for a, b in zip(first, last)), 1, first)]
        previous = {(1, first): None}
        while frontier:
            self.expanded += 1
            if self.expanded > 100000 or time.monotonic() > self.deadline:
                raise TimeoutError('swarm planning budget exhausted; no flight commands issued')
            _, phase, cell = heapq.heappop(frontier)
            position = self.point(cell)
            if cell == last and self.available(position, goal, phase, reservations) and all(
                    self.available(goal, goal, t, reservations)
                    for t in range(phase + 1, self.max_steps + 3)):
                path, state = [goal], (phase, cell)
                while state is not None:
                    path.append(self.point(state[1]))
                    state = previous[state]
                return [tuple(start), *reversed(path)]
            if phase >= self.max_steps:
                continue
            moves = [(0, 0, 0), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                     (0, 0, 1), (0, 0, -1)]
            for move in moves:
                following = tuple(a + b for a, b in zip(cell, move))
                state = (phase + 1, following)
                if state in previous or not self.available(
                        position, self.point(following), phase, reservations):
                    continue
                previous[state] = (phase, cell)
                heuristic = sum(abs(a - b) for a, b in zip(following, last))
                heapq.heappush(frontier, (phase + 1 + heuristic, phase + 1, following))
        return None

    def plan(self, starts, goals):
        """Try deterministic priority orders and return only a completely verified joint plan."""
        if set(starts) != set(goals) or not 2 <= len(starts) <= 4:
            raise ValueError('joint plan requires matching vehicle identities')
        for positions in [starts, goals]:
            if any(len(p) != 3 or not all(math.isfinite(v) for v in p)
                   for p in positions.values()):
                raise ValueError('nonfinite world positions')
            if any(math.dist(a, b) < self.clearance for a, b in
                   itertools.combinations(positions.values(), 2)):
                raise ValueError('start or target separation does not include tracking margins')
        self.deadline = time.monotonic() + 8.
        self.expanded = 0
        for order in itertools.permutations(sorted(starts)):
            paths = {}
            for name in order:
                path = self.route(starts[name], goals[name], paths)
                if path is None:
                    break
                paths[name] = path
            if len(paths) == len(starts):
                certificate = self.verify(paths)
                return dict(paths=paths, order=order, expanded=self.expanded,
                            minimum_segment_separation=certificate,
                            required_separation=self.clearance)
        raise ValueError('no collision-free prioritized plan within configured bounds')

    def verify(self, paths):
        """Audit all paths together independently of search order before execution."""
        minimum = float('inf')
        for phase in range(max(map(len, paths.values())) - 1):
            edges = [(self.at(path, phase), self.at(path, phase + 1))
                     for path in paths.values()]
            if not all(self.edge_free(*edge) for edge in edges):
                raise ValueError('joint plan violates bounds or obstacles')
            for first, second in itertools.combinations(edges, 2):
                separation = segment_distance(*first, *second)
                minimum = min(minimum, separation)
                if separation < self.clearance - 1e-9:
                    raise ValueError('joint plan violates swept-segment separation')
        return minimum


def interpolate(start, end, elapsed, duration):
    """Quintic rest-to-rest position, velocity and acceleration along the certified segment."""
    fraction = max(0., min(1., elapsed / duration))
    s = 10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5
    speed = (30 * fraction**2 - 60 * fraction**3 + 30 * fraction**4) / duration
    acceleration = (60 * fraction - 180 * fraction**2 + 120 * fraction**3) / duration**2
    delta = subtract(end, start)
    return (tuple(a + s * d for a, d in zip(start, delta)),
            tuple(speed * d for d in delta), tuple(acceleration * d for d in delta))


def phase_duration(edges, speed, acceleration):
    """Respect the exact quintic maximum velocity and acceleration for all fleet members."""
    length = max(math.dist(a, b) for a, b in edges)
    return max(.5, 1.875 * length / speed, math.sqrt(5.774 * length / acceleration))
