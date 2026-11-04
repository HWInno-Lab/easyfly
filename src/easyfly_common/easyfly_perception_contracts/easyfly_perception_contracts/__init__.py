# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_perception_contracts/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""ROS boundary contracts. Conversions never manufacture V2 evidence from V1."""
import copy
import math
import re
import uuid

from easyfly_msgs.msg import PerceptionContextV2 as Context, ObservationRefV2
from easyfly_msgs.msg import Target3D, Target3DArray, TrackInfo, TrackInfoArray


def stamp(header):
    t = header.stamp
    return t.sec * 10**9 + t.nanosec if t.sec >= 0 and 0 <= t.nanosec < 10**9 else -1


def uuid_valid(value):
    return len(value.uuid) == 16 and any(value.uuid)


def identity(key):
    return (bytes(key.tracker_session_id.uuid), key.tracker_epoch, key.raw_id)


def reference(context):
    return ObservationRefV2(header=copy.deepcopy(context.header),
                            producer_session_id=copy.deepcopy(context.producer_session_id),
                            epoch=context.epoch, sequence=context.sequence)


def ref_valid(ref):
    return uuid_valid(ref.producer_session_id) and ref.sequence > 0 and stamp(ref.header) > 0 and 0 < len(ref.header.frame_id.encode()) <= 256


def key_valid(key):
    return (uuid_valid(key.tracker_session_id) and key.raw_id >= 0 and
            (not key.stable_id_valid or key.stable_id >= 0) and
            (not key.legacy_id_valid or min(key.legacy_raw_id, key.legacy_stable_id) >= 0))


def context_valid(c):
    return (bool(re.fullmatch('[A-Za-z0-9_]{1,32}', c.camera_id)) and
            uuid_valid(c.producer_session_id) and c.sequence > 0 and
            0 <= c.status <= Context.STALE and len(c.reason.encode()) <= 256 and
            len(c.header.frame_id.encode()) <= 256 and
            (c.status != Context.VALID or (stamp(c.header) > 0 and bool(c.header.frame_id) and
             c.image_width > 0 and c.image_height > 0 and c.image_width * c.image_height <= 2073600)))


def box_valid(box):
    return all(math.isfinite(v) for v in [box.x1, box.y1, box.x2, box.y2]) and box.x2 > box.x1 and box.y2 > box.y1 and max(abs(v) for v in [box.x1, box.y1, box.x2, box.y2]) <= (2**31-1)/4


def score(valid, value):
    return not valid or math.isfinite(value) and 0 <= value <= 1


def detections_valid(m):
    return (context_valid(m.context) and m.context.status == Context.VALID and len(m.detections) <= 100 and
            all(box_valid(t.box) and t.class_id >= -1 and len(t.class_name.encode()) <= 128 and
                score(t.confidence_valid, t.confidence) for t in m.detections))


def tracks_valid(m):
    if not context_valid(m.context) or m.context.status != Context.VALID or not ref_valid(m.upstream) or len(m.tracks) > 100:
        return False
    keys = set()
    for t in m.tracks:
        key = identity(t.key)
        if (not key_valid(t.key) or key in keys or
                t.key.tracker_session_id != m.context.producer_session_id or t.key.tracker_epoch != m.context.epoch or
                not t.filtered_box_valid or not box_valid(t.filtered_box) or
                t.observed_box_valid and not box_valid(t.observed_box) or
                t.observed_this_frame and (not t.observed_box_valid or t.missed_updates != 0) or
                len(t.class_name.encode()) > 128 or not score(t.confidence_valid, t.confidence) or
                t.image_velocity_unit > 2 or t.image_velocity_valid and
                (t.image_velocity_unit == 0 or not all(math.isfinite(v) for v in [t.image_velocity_x, t.image_velocity_y]))):
            return False
        keys.add(key)
    return True


def world_valid(m):
    if (not context_valid(m.context) or m.context.status != Context.VALID or
            m.context.header.frame_id != 'odom' or not ref_valid(m.upstream) or
            not m.transform_exact or m.context.header.stamp != m.upstream.header.stamp or m.transform_stamp != m.upstream.header.stamp or
            m.depth_stamp.sec * 10**9 + m.depth_stamp.nanosec <= 0 or len(m.targets) > 100):
        return False
    keys, aliases = set(), set()
    for t in m.targets:
        key = identity(t.key)
        if (not key_valid(t.key) or key in keys or not ref_valid(t.observation) or t.observation.header != m.upstream.header or
                t.key.tracker_session_id != m.upstream.producer_session_id or
                t.key.tracker_epoch != m.upstream.epoch or
                t.key.legacy_id_valid and t.key.legacy_raw_id in aliases or
                len(t.class_name.encode()) > 128 or not score(t.confidence_valid, t.confidence) or
                t.position_valid and (not all(math.isfinite(v) for v in [
                    t.position_x, t.position_y, t.position_z, t.optical_depth_m]) or t.optical_depth_m <= 0)):
            return False
        keys.add(key)
        if t.key.legacy_id_valid:
            aliases.add(t.key.legacy_raw_id)
    return True


def writer(node, topic, info):
    sources = node.get_publishers_info_by_topic(node.resolve_topic_name(topic))
    if len(sources) != 1:
        return None
    gid = bytes(sources[0].endpoint_gid)
    return gid if info is not None and gid == bytes(info.publisher_gid) else None


class Producer:
    def __init__(self):
        from unique_identifier_msgs.msg import UUID
        self.session = UUID(uuid=list(uuid.uuid4().bytes))
        self.epoch = self.sequence = self.last_stamp = 0

    def reset(self):
        if self.epoch == 2**64 - 1:
            raise OverflowError('perception epoch exhausted')
        self.epoch += 1
        self.sequence = self.last_stamp = 0

    def next(self, header, camera, width, height, status=Context.VALID, reason=''):
        source = stamp(header)
        if 0 < source < self.last_stamp or self.sequence == 2**64 - 1:
            self.reset()
        self.sequence += 1
        self.last_stamp = max(0, source)
        return Context(header=copy.deepcopy(header), camera_id=camera, image_width=width,
                       image_height=height, producer_session_id=copy.deepcopy(self.session),
                       epoch=self.epoch, sequence=self.sequence, status=status, reason=reason)


class SourceGate:
    def __init__(self):
        self.current = None
        self.retired = set()
        self.epoch = self.sequence = self.last_stamp = 0
        self.writer = b''
        self.changed = self.exhausted = False

    def observe(self, c, source=b''):
        self.changed = False
        if not context_valid(c) or self.exhausted:
            return False
        key = bytes(c.producer_session_id.uuid)
        if key in self.retired or (key == self.current and (c.epoch < self.epoch or source != self.writer and self.writer)):
            return False
        transition = self.current is not None and (key != self.current or c.epoch != self.epoch or source != self.writer)
        if key != self.current and self.current is not None:
            if len(self.retired) >= 256:
                self.exhausted = True
                return False
            self.retired.add(self.current)
        previous_sequence, previous_stamp = (0, 0) if transition or self.current is None else (self.sequence, self.last_stamp)
        if c.sequence <= previous_sequence or c.status == Context.VALID and stamp(c.header) <= previous_stamp:
            return False
        self.current, self.epoch, self.sequence, self.writer = key, c.epoch, c.sequence, source
        self.last_stamp = stamp(c.header) if c.status == Context.VALID else previous_stamp
        self.changed = transition
        return True


def world_snapshot(m):
    if not world_valid(m) or any(not t.key.legacy_id_valid or any(abs(v) > 3.402823466e38 for v in [t.position_x, t.position_y, t.position_z, t.optical_depth_m]) for t in m.targets):
        raise ValueError('invalid native V2 world envelope or missing compatibility alias')
    return Target3DArray(header=copy.deepcopy(m.context.header), targets=[Target3D(
        header=copy.deepcopy(m.context.header), track_id=t.key.legacy_raw_id,
        class_name=t.class_name, confidence=t.confidence if t.confidence_valid else 0.,
        position_x=float(t.position_x), position_y=float(t.position_y), position_z=float(t.position_z),
        position_valid=t.position_valid, distance=float(t.optical_depth_m), pixel_u=t.pixel_u, pixel_v=t.pixel_v)
        for t in m.targets])


def track_snapshot(m, observed_only=False, stable=False):
    if not tracks_valid(m):
        raise ValueError('invalid native V2 track envelope')
    out = TrackInfoArray(header=copy.deepcopy(m.context.header))
    for t in m.tracks:
        if observed_only and not t.observed_this_frame:
            continue
        if not t.key.legacy_id_valid:
            raise ValueError('missing producer alias')
        b = t.observed_box if observed_only else t.filtered_box
        out.tracks.append(TrackInfo(header=copy.deepcopy(out.header),
            track_id=t.key.legacy_stable_id if stable else t.key.legacy_raw_id,
            class_name=t.class_name, confidence=t.confidence if t.confidence_valid else 0.,
            bbox_center_x=(b.x1+b.x2)/2/m.context.image_width,
            bbox_center_y=(b.y1+b.y2)/2/m.context.image_height,
            bbox_width=(b.x2-b.x1)/m.context.image_width,
            bbox_height=(b.y2-b.y1)/m.context.image_height))
    return out


class WorldInput:
    """Native source admission shared by query, semantics and task consumers."""
    def __init__(self, node, accept, invalidate, topic='/perception/v2/world_targets'):
        from easyfly_msgs.msg import WorldTargetArrayV2
        from rclpy.qos import qos_profile_sensor_data
        self.gate, self.scope = SourceGate(), None
        self.aliases = {}
        from rcl_interfaces.msg import ParameterDescriptor
        camera = node.get_parameter('camera_id').value if node.has_parameter('camera_id') else node.declare_parameter('camera_id', 'front', ParameterDescriptor(read_only=True)).value
        self.native = None
        def receive(message, info):
            source = writer(node, topic, info)
            if source is None:
                self.native = None
                invalidate('ambiguous_perception_writer')
                return
            if not self.gate.observe(message.context, source):
                return
            scope = bytes(message.upstream.producer_session_id.uuid), message.upstream.epoch
            if self.gate.changed or self.scope is not None and scope != self.scope:
                self.native = None
                self.aliases.clear()
                invalidate('perception_source_changed')
            self.scope = scope
            age = node.get_clock().now().nanoseconds - stamp(message.context.header)
            if not world_valid(message) or message.context.camera_id != camera or not -200000000 <= age <= 500000000:
                self.native = None
                invalidate('invalid_perception_evidence')
                return
            for t in message.targets:
                key = identity(t.key)
                if self.aliases.get(t.key.legacy_raw_id, key) != key or len(self.aliases) >= 65536:
                    self.native = None
                    invalidate('compatibility_alias_rebound_or_exhausted')
                    return
                self.aliases[t.key.legacy_raw_id] = key
            try:
                numeric = world_snapshot(message)
            except (ValueError, OverflowError, AssertionError):
                self.native = None
                invalidate('unrepresentable_perception_view')
                return
            self.native = message
            accept(message, numeric)
        self.subscription = NativeSubscription(node, WorldTargetArrayV2, topic, receive)


def perception_version(node):
    from rcl_interfaces.msg import ParameterDescriptor
    value = node.declare_parameter('perception_version', 1, ParameterDescriptor(read_only=True)).value
    if type(value) is not int or value not in (1, 2):
        raise ValueError('perception_version must be 1 or 2')
    return value


def prediction_snapshot(m):
    from easyfly_msgs.msg import TargetPrediction, TargetPredictionArray
    if (not context_valid(m.context) or m.context.status != Context.VALID or
            m.context.header.frame_id != 'odom' or not ref_valid(m.upstream) or len(m.predictions) > 100):
        raise ValueError('invalid native prediction envelope')
    out = TargetPredictionArray(header=copy.deepcopy(m.context.header))
    seen = set()
    for t in m.predictions:
        key = identity(t.key)
        if (not key_valid(t.key) or key in seen or not t.key.legacy_id_valid or
                not ref_valid(t.observation) or t.header != m.context.header or
                t.observation_header.frame_id != 'odom' or stamp(t.observation_header) <= 0 or
                len(t.class_name.encode()) > 128 or len(t.reason.encode()) > 256 or
                not score(t.confidence_valid, t.confidence) or
                not math.isfinite(t.horizon_seconds) or not 0 <= t.horizon_seconds <= 10 or
                not math.isfinite(t.observation_age_seconds) or t.observation_age_seconds < 0 or
                t.valid and (not all(math.isfinite(v) for v in [t.position_x, t.position_y, t.position_z,
                    t.velocity_x, t.velocity_y, t.velocity_z, t.variance_x, t.variance_y, t.variance_z]) or
                    min(t.variance_x, t.variance_y, t.variance_z) < 0)):
            raise ValueError('invalid native prediction item')
        seen.add(key)
        value = TargetPrediction(track_id=t.key.legacy_raw_id)
        for field in value.get_fields_and_field_types():
            if field != 'track_id':
                setattr(value, field, copy.deepcopy(getattr(t, field)))
        out.predictions.append(value)
    return out


class NativeSubscription:
    """Bounded C++/CDR intake preserving actual writer evidence on ROS Humble."""
    def __init__(self, node, kind, topic, callback):
        from types import SimpleNamespace
        from rclpy.serialization import deserialize_message
        from rclpy.clock import Clock, ClockType
        from ._intake import Intake
        self.intake = Intake(node.resolve_topic_name(topic), kind.__name__)
        def poll():
            for _ in range(4):
                item = self.intake.take()
                if item is None: break
                payload, gid = item
                if not payload or not gid:
                    callback(kind(), None)
                    continue
                callback(deserialize_message(payload, kind), SimpleNamespace(publisher_gid=gid))
        self.timer = node.create_timer(.01, poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if not hasattr(node, '_native_intakes'):
            node._native_intakes = []
            destroy = node.destroy_node
            def destroy_with_intakes():
                for intake in node._native_intakes:
                    intake.close()
                node._native_intakes.clear()
                return destroy()
            node.destroy_node = destroy_with_intakes
        node._native_intakes.append(self.intake)
