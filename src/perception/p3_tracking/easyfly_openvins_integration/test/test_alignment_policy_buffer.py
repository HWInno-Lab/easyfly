# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Policy-service scheduling must not manufacture gaps in healthy sensor streams."""
import time
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

from easyfly_openvins_integration.alignment import Alignment
from easyfly_openvins_integration.alignment_node import OpenVinsAlignment
from easyfly_openvins_integration.reference import ReferencePairer


def fixture():
    engine = Alignment([0., 0., 0.], [0., 0., 0., 1.])
    engine.update_clock(2_000_000_000)
    node = SimpleNamespace(engine=engine, pairer=ReferencePairer(engine),
        clock_ever_verified=True, clock_verified_at=time.monotonic(), health_fields_valid=True,
        p={'reference_pose_frame': 'map', 'reference_attitude_frame': 'base_link'},
        update_clock=lambda: None)
    for name in ['clock_policy_ready', 'publish_outputs', 'on_reference', 'on_attitude']:
        setattr(node, name, getattr(OpenVinsAlignment, name).__get__(node))
    return node


def feed(node, ns):
    p, a = PoseStamped(), Imu()
    p.header.frame_id, a.header.frame_id = 'map', 'base_link'
    p.header.stamp.sec, p.header.stamp.nanosec = divmod(ns, 10**9)
    a.header.stamp = p.header.stamp
    a.orientation.w = 1.
    node.on_attitude(a)
    node.on_reference(p)


def test_delayed_policy_reply_retains_sources_but_cannot_authorize_output():
    node = fixture()
    feed(node, 1_600_000_000)
    feed(node, 1_640_000_000)
    before = len(node.engine.references)
    node.clock_verified_at = time.monotonic()-1.
    for ns in range(1_680_000_000, 1_880_000_001, 40_000_000):
        feed(node, ns)
    assert node.engine.failure is None
    assert len(node.engine.references) == before
    assert not node.engine.health_valid
    node.clock_verified_at = time.monotonic()
    feed(node, 1_920_000_000)
    assert node.engine.failure is None
    assert len(node.engine.references) > before
    assert node.engine.health_valid
    # Genuine missing source samples are still rejected under the same limit.
    node.engine.update_clock(2_200_000_000)
    feed(node, 2_160_000_000)
    assert node.engine.failure is not None
    assert 'source_gap' in node.engine.reason


def test_unverified_clock_does_not_enter_reference_history():
    node = fixture()
    node.clock_ever_verified = False
    feed(node, 1_900_000_000)
    assert not node.pairer.positions and not node.pairer.attitudes
