#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Physical occluded rectangle route, for a genuine old-place loop query."""
import hashlib
import importlib.util
import math
from pathlib import Path

spec = importlib.util.spec_from_file_location('orb_corridor_fixture', Path(__file__).with_name('capture_orbslam3_loop.py'))
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


class Corridor(base.Capture):
    def __init__(self, args):
        super().__init__(args)
        data = Path(__file__).read_bytes()
        (self.directory / 'corridor-source.py').write_bytes(data)
        self.report['corridor_source_sha256'] = hashlib.sha256(data).hexdigest()

    def wall_layout(self):
        return [
            (7., 0., 2., 0., 8., 4.8), (2.5, 4., 2., math.pi/2, 9., 4.8),
            (-2., 0., 2., math.pi, 8., 4.8), (2.5, -4., 2., 3*math.pi/2, 9., 4.8),
            # Four opaque collision panels form the central occlusion box.
            (1.2, 0., 1.6, 0., 2.6, 3.2), (2.5, 1.3, 1.6, 3*math.pi/2, 2.6, 3.2),
            (3.8, 0., 1.6, math.pi, 2.6, 3.2), (2.5, -1.3, 1.6, math.pi/2, 2.6, 3.2),
        ]

    def route_waypoints(self, origin, yaw0):
        # The registered lane has zero native yaw. Keep every target outside
        # the central collision box with >=1.1 m center clearance to its faces.
        x0, y0, z0 = origin
        current = [x0, y0, z0]
        heading = yaw0
        targets = [([x0, y0+2.5, z0], math.pi/2),
                   ([x0+5., y0+2.5, z0], 0.),
                   ([x0+5., y0-2.5, z0], -math.pi/2),
                   ([x0, y0-2.5, z0], -math.pi),
                   ([x0, y0, z0], -3*math.pi/2)]
        number = 0
        for end, direction in targets:
            delta = math.atan2(math.sin(direction-heading), math.cos(direction-heading))
            turns = max(1, math.ceil(abs(delta)/math.radians(15)))
            for i in range(1, turns+1):
                angle = heading+delta*i/turns
                number += 1
                yield f'turn-{number}', list(current), math.atan2(math.sin(angle), math.cos(angle))
            heading += delta
            steps = max(1, math.ceil(math.dist(current, end)/.5))
            start = list(current)
            for i in range(1, steps+1):
                current = [a+(b-a)*i/steps for a, b in zip(start, end)]
                number += 1
                yield f'leg-{number}', list(current), math.atan2(math.sin(heading), math.cos(heading))
        delta = math.atan2(math.sin(yaw0-heading), math.cos(yaw0-heading))
        for i in range(1, 7):
            angle = heading+delta*i/6
            yield f'close-view-{i}', list(current), math.atan2(math.sin(angle), math.cos(angle))


if __name__ == '__main__':
    base.orb.fixture.Probe = Corridor
    raise SystemExit(base.orb.fixture.main())
