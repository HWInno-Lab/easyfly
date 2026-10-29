#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Original native RGB-D/IMU capture with uninterrupted, physically flown legs."""
import hashlib
import importlib.util
import math
from pathlib import Path

spec = importlib.util.spec_from_file_location('orb_inertial_corridor_base', Path(__file__).with_name('capture_orbslam3_corridor.py'))
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


class InertialCorridor(base.Corridor):
    def __init__(self, args):
        super().__init__(args)
        data = Path(__file__).read_bytes()
        (self.directory / 'inertial-corridor-source.py').write_bytes(data)
        self.report['inertial_corridor_source_sha256'] = hashlib.sha256(data).hexdigest()
        self.report['motion_fixture'] = dict(
            straight_legs='single FCU waypoint per existing free corridor segment',
            reason='avoid half-meter stop/hover intervals that reset upstream inertial initialization',
            acquisition_speed_mps=.3, original_native_measurements=True,
            extra_old_place_visit=[0., 2.5], truth_injected=False)

    def route_waypoints(self, origin, yaw0):
        # Coalesce collinear half-meter goals, retaining all original turn poses
        # and the same free physical segments, speed and collision clearance.
        pending = None
        for item in super().route_waypoints(origin, yaw0):
            if item[0].startswith('leg-'):
                pending = item
            else:
                if pending:
                    yield pending
                    pending = None
                yield item
        if pending:
            yield pending
        # Revisit the first north/east corner as well as the launch position:
        # an actual new inertial map may have started there after static resets.
        x, y, z = origin
        north = [x, y+2.5, z]
        for i in range(1, 7):
            yield f'revisit-north-turn-{i}', list(origin), yaw0+(math.pi/2-yaw0)*i/6
        yield 'revisit-north-leg', north, math.pi/2
        for i in range(1, 7):
            yield f'revisit-east-turn-{i}', north, math.pi/2*(1-i/6)
        for i in range(3):
            yield f'old-view-hold-{i}', north, 0.
        yield 'return-launch-position', list(origin), 0.


if __name__ == '__main__':
    base.base.orb.fixture.Probe = InertialCorridor
    raise SystemExit(base.base.orb.fixture.main())
