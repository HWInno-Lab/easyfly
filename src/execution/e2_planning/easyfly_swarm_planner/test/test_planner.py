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

"""Independent geometric and task-allocation contracts for bounded fleet planning."""

import copy
import itertools
import json
import math
from pathlib import Path

from easyfly_swarm_planner.configuration import load
from easyfly_swarm_planner.planner import assign, interpolate, phase_duration, Planner
from easyfly_swarm_planner.planner import segment_distance
import pytest


def configuration():
    """Load the declared demonstration configuration without depending on installation paths."""
    return json.loads((Path(__file__).parents[1] / 'config/swap.example.json').read_text())


def test_crossing_and_asynchronous_edge_conflicts():
    """Crossing and head-on segments collide even when their endpoints are separated."""
    assert segment_distance((-2, 0, 0), (2, 0, 0), (0, -2, 0), (0, 2, 0)) == 0.
    assert segment_distance((0, 0, 0), (3, 0, 0), (3, 0, 0), (0, 0, 0)) == 0.
    assert segment_distance((0, 0, 0), (3, 0, 0), (0, 3, 0), (3, 3, 0)) == 3.
    assert segment_distance((0, 0, 0), (0, 0, 0), (1, 0, 0), (1, 0, 0)) == 1.


def test_swap_has_true_swept_volume_clearance():
    """Sample both vehicles' independent progress through every returned phase."""
    starts = {'alpha': (0., 0., 1.5), 'bravo': (5., 0., 1.5)}
    planner = Planner(configuration())
    result = planner.plan(starts, {'alpha': starts['bravo'], 'bravo': starts['alpha']})
    paths = list(result['paths'].values())
    for phase in range(max(map(len, paths)) - 1):
        for s, t in itertools.product([0., .25, .5, .75, 1.], repeat=2):
            points = []
            for path, fraction in zip(paths, [s, t]):
                a, b = planner.at(path, phase), planner.at(path, phase + 1)
                points.append(tuple(x + fraction * (y - x) for x, y in zip(a, b)))
            assert math.dist(*points) >= planner.clearance - 1e-9
    assert result['paths']['alpha'][-1] == starts['bravo']
    assert result['paths']['bravo'][-1] == starts['alpha']


def test_assignment_and_obstacle_detour():
    """Allocate the minimum-cost target permutation and route around an inflated obstacle."""
    starts = {'alpha': (0., 0., 1.5), 'bravo': (5., 0., 1.5)}
    assigned = assign(starts, [(5., 3., 1.5), (0., 3., 1.5)])
    assert assigned == {'alpha': (0., 3., 1.5), 'bravo': (5., 3., 1.5)}
    config = configuration()
    config['obstacles'] = [{'lower': [-.4, 1.1, .5], 'upper': [.4, 1.9, 3.5]}]
    planner = Planner(config)
    result = planner.plan(starts, assigned)
    assert any(abs(point[0]) >= 2 for point in result['paths']['alpha'])
    assert planner.verify(result['paths']) >= planner.clearance


def test_impossible_endpoint_and_invalid_bounds_fail_without_path():
    """Neither conflicting goals nor an occupied destination can produce an executable plan."""
    starts = {'alpha': (0., 0., 1.5), 'bravo': (5., 0., 1.5)}
    with pytest.raises(ValueError, match='separation'):
        Planner(configuration()).plan(starts, {'alpha': (1., 1., 1.5), 'bravo': (2., 1., 1.5)})
    config = configuration()
    config['obstacles'] = [{'lower': [-1., -1., .5], 'upper': [1., 1., 3.5]}]
    with pytest.raises(ValueError, match='no collision-free'):
        Planner(config).plan(starts, starts)


def test_quintic_speed_and_acceleration_limits():
    """Numerically sample exact interpolation bounds and rest-to-rest endpoints."""
    a, b = (0., 0., 1.), (2., 2., 3.)
    duration = phase_duration([(a, b)], .5, .4)
    for i in range(101):
        _, velocity, acceleration = interpolate(a, b, duration * i / 100, duration)
        assert math.sqrt(sum(v * v for v in velocity)) <= .5 + 1e-9
        assert math.sqrt(sum(v * v for v in acceleration)) <= .4 + 1e-9
    assert interpolate(a, b, 0, duration) == (a, (0., 0., 0.), (0., 0., 0.))
    assert interpolate(a, b, duration, duration) == (b, (0., 0., 0.), (0., 0., 0.))


def test_configuration_rejects_unbounded_or_aliased_tasks(tmp_path):
    """Validate input budgets and exact fleet target identity before native action admission."""
    config = configuration()
    path = tmp_path / 'mission.json'
    path.write_text(json.dumps(config))
    assert load(path, ['alpha', 'bravo']) == config
    for key, value in [('speed_mps', float('nan')), ('grid_m', .01), ('max_steps', 100000),
                       ('targets', {'alpha': [0, 0, 1], 'charlie': [5, 0, 1]})]:
        changed = copy.deepcopy(config)
        changed[key] = value
        path.write_text(json.dumps(changed))
        with pytest.raises(ValueError):
            load(path, ['alpha', 'bravo'])


def test_four_vehicle_assignment_and_joint_separation():
    """Find and jointly validate a four-vehicle exchange without assuming two-agent geometry."""
    starts = {'alpha': (0., 0., 1.5), 'bravo': (5., 0., 1.5),
              'charlie': (0., 5., 1.5), 'delta': (5., 5., 1.5)}
    goals = dict(zip(starts, reversed(list(starts.values()))))
    planner = Planner(configuration())
    result = planner.plan(starts, goals)
    assert planner.verify(result['paths']) >= planner.clearance
    assert all(result['paths'][name][-1] == goals[name] for name in starts)
