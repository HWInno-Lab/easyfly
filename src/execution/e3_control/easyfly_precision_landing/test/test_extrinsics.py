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

"""Independent analytical rotations and original-time failure boundaries."""

import math

import pytest

from easyfly_precision_landing.extrinsics import Mount, MountHistory
from easyfly_precision_landing.geometry import apply, rotation


def sample(stamp=1000000000, receipt=1., angle=0.):
    return Mount(stamp, receipt, (0., 0., -.08),
                 (0., math.sin(angle / 2), 0., math.cos(angle / 2)))


def admit(history, value, now=None, gid='one'):
    return history.receive(value, 'base_link', 'optical', gid,
                           value.stamp_ns, value.receipt if now is None else now)


def test_original_time_interpolation_matches_analytic_rotating_camera():
    history = MountHistory('optical')
    admit(history, sample())
    admit(history, sample(1100000000, 1.1, .1))
    position, q = history.at(1050000000, 1100000000, 1.15)
    assert position == (0., 0., -.08)
    actual = apply(rotation(q), (0., 0., 1.))
    assert actual == pytest.approx((math.sin(.05), 0., math.cos(.05)), abs=1e-12)
    assert history.at(1100000000, 1100000000, 1.15)[1] == sample(angle=.1).quaternion
    with pytest.raises(ValueError):
        history.at(1110000000, 1110000000, 1.15)
    with pytest.raises(ValueError):
        history.at(990000000, 1100000000, 1.15)


def test_duplicates_and_delayed_queries_cannot_renew_mount_lifetime():
    history = MountHistory('optical')
    assert admit(history, sample())
    assert not admit(history, sample(receipt=1.2))
    assert history.samples[0].receipt == 1.
    with pytest.raises(ValueError):
        history.at(1000000000, 1000000000, 1.31)
    assert not admit(history, sample(1000000000, 1.), now=1.31)


@pytest.mark.parametrize('fault', ['identity', 'conflict', 'rollback', 'jump', 'frame'])
def test_discontinuity_requires_new_explicit_mount_session(fault):
    history = MountHistory('optical')
    admit(history, sample())
    with pytest.raises(ValueError):
        if fault == 'identity':
            admit(history, sample(1100000000, 1.1), gid='another')
        elif fault == 'conflict':
            admit(history, sample(angle=.01))
        elif fault == 'rollback':
            admit(history, sample(990000000, 1.1))
        elif fault == 'jump':
            admit(history, sample(1100000000, 1.1, .3))
        else:
            history.receive(sample(), 'world', 'optical', 'one', 1000000000, 1.)
    with pytest.raises(ValueError):
        admit(history, sample(1200000000, 1.2))
    assert not history.samples


def test_gap_and_stale_or_future_samples_never_supply_an_extrapolated_mount():
    history = MountHistory('optical')
    admit(history, sample())
    admit(history, sample(1200000000, 1.2))
    assert len(history.samples) == 1
    with pytest.raises(ValueError):
        history.at(1100000000, 1200000000, 1.2)
    future = sample(1300000000, 1.3)
    assert not history.receive(future, 'base_link', 'optical', 'one', 1250000000, 1.3)
    assert len(history.samples) == 1


def test_measured_encoder_uses_calibrated_axis_and_rotating_optical_lever():
    from easyfly_precision_landing.mount_node import measured_mount
    from easyfly_precision_landing.geometry import apply, rotation
    angle = .2
    position, quaternion = measured_mount(angle, (0., 0., -.08), (.03, 0., 0.),
                                          (0., 1., 0.), (0., 0., 0., 1.))
    assert position == pytest.approx((.03*math.cos(angle), 0., -.08-.03*math.sin(angle)))
    assert apply(rotation(quaternion), (0, 0, 1)) == pytest.approx(
        (math.sin(angle), 0., math.cos(angle)))
    with pytest.raises(ValueError):
        measured_mount(.36, (0., 0., 0.), (0., 0., 0.), (0., 1., 0.), (0., 0., 0., 1.))
