# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_perception_contracts/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
import pytest
from std_msgs.msg import Header
from easyfly_perception_contracts import Producer, SourceGate, Context, context_valid


def test_restart_epoch_and_duplicate_observation_are_distinct():
    one, two, gate = Producer(), Producer(), SourceGate()
    h = Header(frame_id='camera_color_optical_frame'); h.stamp.sec = 10
    a = one.next(h, 'front', 64, 48)
    assert gate.observe(a, b'first') and not gate.changed
    assert not gate.observe(a, b'first')
    one.reset(); h.stamp.sec = 5
    b = one.next(h, 'front', 64, 48)
    assert gate.observe(b, b'first') and gate.changed
    assert not gate.observe(a, b'first')
    h.stamp.sec = 11
    assert gate.observe(two.next(h, 'front', 64, 48), b'second') and gate.changed
    b.sequence = 100; b.header.stamp.sec = 20
    assert not gate.observe(b, b'first')


def test_missing_image_evidence_cannot_be_valid_empty_result():
    p = Producer(); h = Header(frame_id='camera_color_optical_frame'); h.stamp.sec = 1
    assert not context_valid(p.next(h, 'front', 0, 0))
    waiting = p.next(h, 'front', 0, 0, Context.NOT_READY, 'waiting for source')
    assert context_valid(waiting)


def test_velocity_units_and_observed_boxes_are_not_lost_in_display_view():
    from easyfly_msgs.msg import ImageTrackArrayV2, ImageTrackV2, PixelBoxV2, TrackKeyV2
    from easyfly_perception_contracts import reference, track_snapshot, tracks_valid
    producer, detector = Producer(), Producer()
    h = Header(frame_id='camera_color_optical_frame'); h.stamp.sec = 1
    c = producer.next(h, 'front', 100, 50)
    t = ImageTrackV2(key=TrackKeyV2(tracker_session_id=c.producer_session_id, raw_id=2**40,
        legacy_id_valid=True, legacy_raw_id=3, legacy_stable_id=8), filtered_box_valid=True,
        filtered_box=PixelBoxV2(x1=0., y1=0., x2=20., y2=20.), observed_box_valid=True,
        observed_box=PixelBoxV2(x1=20., y1=10., x2=40., y2=30.), observed_this_frame=True,
        image_velocity_valid=True, image_velocity_unit=2, image_velocity_x=120.)
    m = ImageTrackArrayV2(context=c, upstream=reference(detector.next(h, 'front', 100, 50)), tracks=[t])
    assert tracks_valid(m)
    view = track_snapshot(m, observed_only=True)
    assert view.tracks[0].bbox_center_x == pytest.approx(.3)
    assert view.tracks[0].velocity_x == 0., 'pixels/second must not become ambiguous legacy velocity'
    assert m.tracks[0].key.raw_id == 2**40
    m.tracks[0].observed_this_frame = False
    assert not track_snapshot(m, observed_only=True).tracks


def test_world_alias_collision_and_exact_transform_validation():
    from easyfly_msgs.msg import WorldTargetArrayV2, WorldTargetV2, TrackKeyV2
    from easyfly_perception_contracts import reference, world_valid
    tracker, producer = Producer(), Producer()
    h = Header(frame_id='camera_color_optical_frame'); h.stamp.sec = 1
    tracked = tracker.next(h, 'front', 64, 48)
    h.frame_id = 'odom'
    context = producer.next(h, 'front', 64, 48)
    key = TrackKeyV2(tracker_session_id=tracked.producer_session_id, raw_id=2**40,
                    legacy_id_valid=True, legacy_raw_id=0, legacy_stable_id=0)
    target = WorldTargetV2(key=key, observation=reference(tracked), position_valid=True, optical_depth_m=2.)
    m = WorldTargetArrayV2(context=context, upstream=reference(tracked), depth_stamp=h.stamp,
                           transform_stamp=h.stamp, transform_exact=True, targets=[target])
    assert world_valid(m)
    import copy
    other = copy.deepcopy(target); other.key.raw_id += 1
    m.targets.append(other)
    assert not world_valid(m), 'distinct full keys cannot share a compatibility alias'
    m.targets.pop(); m.transform_exact = False
    assert not world_valid(m)
