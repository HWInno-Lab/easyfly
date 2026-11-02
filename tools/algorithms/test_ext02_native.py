# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Isolated native V2 node fixtures. Run with an explicit rebuilt overlay; never flight."""
import copy
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from easyfly_msgs.msg import (DetectionArrayV2, DetectionV2, ImageTrackArrayV2, ImageTrackV2,
    WorldTargetArrayV2, WorldTargetV2, TargetPredictionArrayV2, PixelBoxV2, TrackKeyV2,
    PerceptionContextV2, SingleTrackStateV2, UAVState, GimbalState, Target3DArray)
from easyfly_msgs.srv import GetPerceptionV2, ManageSingleTrackV2, SetGimbalTracking
from easyfly_perception_contracts import Producer, reference, identity, tracks_valid, world_valid
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
import yaml

ROOT = Path(__file__).resolve().parents[2]


class Fixture:
    def __init__(self, directory):
        self.directory = directory
        rclpy.init()
        self.node = rclpy.create_node('ext02_native_fixture')
        self.processes, self.logs = [], []

    def run(self, package, executable, *args, version=2):
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        log = (self.directory / (executable + '.log')).open('w')
        self.logs.append(log)
        process = subprocess.Popen([str(path), '--ros-args', '-p', f'perception_version:={version}', *args],
                                   stdout=log, stderr=log, start_new_session=True)
        self.processes.append(process)
        return process

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.005)

    def until(self, condition, timeout=10):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            assert all(p.poll() is None for p in self.processes), 'node exited: ' + str(self.directory)
            rclpy.spin_once(self.node, timeout_sec=.01)
        assert condition(), 'native fixture timed out: ' + str(self.directory)

    def close(self):
        for p in self.processes:
            if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
        for p in self.processes:
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL); p.wait(timeout=3)
        for stream in self.logs: stream.close()
        self.node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def f(tmp_path):
    value = Fixture(tmp_path)
    try: yield value
    finally: value.close()


def ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def camera(f, optical='camera_color_optical_frame'):
    info_pub = f.node.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
    depth_pub = f.node.create_publisher(Image, '/camera/aligned_depth_to_color/depth/image_raw', 10)
    def send():
        h = Header(frame_id=optical, stamp=f.node.get_clock().now().to_msg())
        info = CameraInfo(header=h, width=64, height=48,
                          k=[40., 0., 32., 0., 40., 24., 0., 0., 1.])
        info_pub.publish(info)
        depth_pub.publish(Image(header=h, width=64, height=48, encoding='32FC1', step=256,
                                data=struct.pack('<3072f', *([2.] * 3072))))
        f.spin(.035)
        return h
    return send


def test_real_sort_projection_prediction_query_and_epoch_isolation(f, tmp_path):
    config = tmp_path / 'tracking.yaml'
    config.write_text(yaml.safe_dump({'tracking': dict(tracker_backend='sort', publish_debug_image=False,
        image_width=64, image_height=48, iou_threshold=.15, max_lost_frames=2, min_hits=1,
        max_tracks=100, target_classes=['person'], id_consolidation=dict(enabled=True, reacquire_radius=3., ttl=10.))}))
    raw, display, worlds, predictions = {}, {}, {}, []
    for topic, kind, sink in [('/perception/v2/tracks_raw', ImageTrackArrayV2, raw),
            ('/perception/v2/tracks', ImageTrackArrayV2, display),
            ('/perception/v2/world_targets', WorldTargetArrayV2, worlds)]:
        f.node.create_subscription(kind, topic, lambda m, target=sink: target.__setitem__(ns(m.context.header), m), 10)
    f.node.create_subscription(TargetPredictionArrayV2, '/perception/v2/predictions', predictions.append, 10)
    detection_pub = f.node.create_publisher(DetectionArrayV2, '/perception/v2/detections', 10)
    send_camera = camera(f)
    reset = f.node.create_client(Trigger, '/easyfly_multi_object_tracking/reset')
    query = f.node.create_client(GetPerceptionV2, '/platform/perception/v2/query')
    broadcaster = StaticTransformBroadcaster(f.node)
    transform = TransformStamped(); transform.header.frame_id = 'odom'
    transform.child_frame_id = 'camera_color_optical_frame'; transform.transform.rotation.w = 1.
    transform.transform.translation.x = 8.; transform.transform.translation.y = 2.
    broadcaster.sendTransform(transform)
    f.run('easyfly_multi_object_tracking', 'tracking_node', '-p', 'config_file:=' + str(config))
    f.run('easyfly_pixel_to_3d', 'pixel_to_3d_node')
    f.run('easyfly_target_prediction', 'target_prediction_node')
    f.run('easyfly_perception_srvs', 'perception_services')
    f.until(lambda: detection_pub.get_subscription_count() and reset.service_is_ready() and query.service_is_ready())
    f.spin(.4); broadcaster.sendTransform(transform)
    producer = Producer()
    def send(present=True):
        h = send_camera()
        m = DetectionArrayV2(context=producer.next(h, 'front', 64, 48), backend_id='onnx', active_provider='fixture')
        if present:
            m.detections = [DetectionV2(box=PixelBoxV2(x1=24., y1=12., x2=40., y2=36.), class_id=0,
                                        class_name='person', confidence_valid=True, confidence=.9)]
        detection_pub.publish(m); key = ns(h)
        f.until(lambda: key in worlds and key in raw and key in display)
        assert worlds[key].context.status == 0, worlds[key].context.reason
        return m, raw[key], worlds[key], display[key]
    first, tracks, world, _ = send()
    assert tracks_valid(tracks) and world_valid(world)
    assert len(world.targets) == 1
    target = world.targets[0]
    assert target.key == tracks.tracks[0].key
    assert world.upstream == reference(tracks.context) and target.observation == tracks.upstream
    assert [target.position_x, target.position_y, target.position_z] == pytest.approx([8., 2., 2.])
    send()
    f.until(lambda: any(p.valid and p.key == target.key for a in predictions for p in a.predictions))
    request = query.call_async(GetPerceptionV2.Request(kind='targets'))
    f.until(request.done)
    assert request.result().status == request.result().VALID
    assert request.result().targets.targets[0].key == target.key
    _, gone, empty_world, predicted = send(False)
    assert not gone.tracks and not empty_world.targets
    assert predicted.tracks and not predicted.tracks[0].observed_this_frame
    future = reset.call_async(Trigger.Request()); f.until(future.done); assert future.result().success
    _, next_tracks, _, _ = send()
    assert identity(next_tracks.tracks[0].key) != identity(target.key)
    assert next_tracks.context.epoch > tracks.context.epoch
    old = copy.deepcopy(first); old.context.sequence += 100
    old.context.header.stamp = f.node.get_clock().now().to_msg()
    detection_pub.publish(old)
    # Same old session/epoch is legitimate for the detector after tracker reset;
    # replacing the detector retires it, then a late sample must be rejected.
    producer = Producer()
    _, replaced, _, _ = send()
    replaced_key = identity(replaced.tracks[0].key)
    old.context.header.stamp = f.node.get_clock().now().to_msg(); detection_pub.publish(old); f.spin(.1)
    assert identity(display[ns(replaced.context.header)].tracks[0].key) == replaced_key
    assert ns(old.context.header) not in display


def test_exact_source_tf_is_required_and_int64_key_survives(f, tmp_path):
    cfg = tmp_path / 'pixel.yaml'
    cfg.write_text(yaml.safe_dump({'pixel_to_3d': dict(camera_optical_frame='missing_optical_frame',
        target_frame='odom', publish_markers=False)}))
    seen = []
    f.node.create_subscription(WorldTargetArrayV2, '/perception/v2/world_targets', seen.append, 10)
    publisher = f.node.create_publisher(ImageTrackArrayV2, '/perception/v2/tracks_raw', 10)
    send_camera = camera(f, 'missing_optical_frame')
    f.run('easyfly_pixel_to_3d', 'pixel_to_3d_node', '-p', 'config_file:=' + str(cfg))
    f.until(lambda: publisher.get_subscription_count()); f.spin(.2)
    detector, tracker = Producer(), Producer()
    h = send_camera(); context = tracker.next(h, 'front', 64, 48)
    key = TrackKeyV2(tracker_session_id=context.producer_session_id, tracker_epoch=context.epoch,
                     raw_id=2**40, legacy_id_valid=True, legacy_raw_id=7, legacy_stable_id=7)
    t = ImageTrackV2(key=key, filtered_box=PixelBoxV2(x1=2., y1=2., x2=10., y2=10.),
                    observed_box=PixelBoxV2(x1=24., y1=12., x2=40., y2=36.), filtered_box_valid=True,
                    observed_box_valid=True, observed_this_frame=True, confidence_valid=True, confidence=.9)
    publisher.publish(ImageTrackArrayV2(context=context, upstream=reference(detector.next(h, 'front', 64, 48)), tracks=[t]))
    f.until(lambda: seen)
    assert seen[-1].context.status != 0 and seen[-1].context.reason == 'source_time_transform_unavailable'
    # A static transform can answer an exact historical query. No latest-time fallback is involved.
    broadcaster = StaticTransformBroadcaster(f.node)
    tf = TransformStamped(); tf.header.frame_id = 'odom'; tf.child_frame_id = 'missing_optical_frame'
    tf.transform.rotation.w = 1.; broadcaster.sendTransform(tf); f.spin(.15)
    h = send_camera(); context = tracker.next(h, 'front', 64, 48)
    publisher.publish(ImageTrackArrayV2(context=context, upstream=reference(detector.next(h, 'front', 64, 48)), tracks=[t]))
    f.until(lambda: seen[-1].context.status == 0)
    assert seen[-1].targets[0].key.raw_id == 2**40
    assert seen[-1].targets[0].position_x == pytest.approx(0.)  # observed box, not filtered box


def test_native_detector_matches_real_fixed_frame_baseline(f):
    seen = []
    f.node.create_subscription(DetectionArrayV2, '/perception/v2/detections', seen.append, 10)
    publisher = f.node.create_publisher(Image, '/camera/color/image_raw', 10)
    f.run('easyfly_object_detection', 'detection_node', '-p', 'use_gpu_override:="false"',
          '-p', 'publish_debug_image_override:="false"')
    f.until(lambda: publisher.get_subscription_count(), 20)
    image = Image(width=640, height=480, step=1920, encoding='rgb8', data=(ROOT /
        'src/perception/p2_detection/easyfly_object_detection/test/m2_frame.bin').read_bytes())
    image.header.frame_id = 'camera_color_optical_frame'; image.header.stamp = f.node.get_clock().now().to_msg()
    publisher.publish(image); f.until(lambda: any(x.context.status == 0 for x in seen))
    actual = next(x for x in seen if x.context.status == 0)
    baseline = json.loads((ROOT / 'src/perception/p2_detection/easyfly_object_detection/test/alg5_native_baseline.json').read_text())
    expected = next(x['result'] for x in baseline['runs'] if x['result']['provider'] == 'CPUExecutionProvider')
    assert len(actual.detections) == len(expected['detections']) == 5
    assert actual.active_provider == 'CPUExecutionProvider'
    for a, b in zip(actual.detections, expected['detections']):
        assert a.class_name == b['class_name'] and a.confidence_valid
        assert a.confidence == pytest.approx(b['confidence'], abs=1e-5)
        assert [a.box.x1, a.box.y1, a.box.x2, a.box.y2] == pytest.approx(b['box'], abs=1e-3)


def native_world(f, tracking, world, raw=2**40, alias=7):
    optical = Header(frame_id='camera_color_optical_frame', stamp=f.node.get_clock().now().to_msg())
    tc = tracking.next(optical, 'front', 64, 48)
    header = copy.deepcopy(optical); header.frame_id = 'odom'
    c = world.next(header, 'front', 64, 48)
    key = TrackKeyV2(tracker_session_id=tc.producer_session_id, tracker_epoch=tc.epoch, raw_id=raw,
                     legacy_id_valid=True, legacy_raw_id=alias, legacy_stable_id=alias)
    return WorldTargetArrayV2(context=c, upstream=reference(tc), depth_stamp=optical.stamp,
        transform_stamp=optical.stamp, transform_exact=True, targets=[WorldTargetV2(key=key,
            observation=reference(tc), class_name='person', confidence_valid=True, confidence=.9,
            position_valid=True, position_x=2., optical_depth_m=2.)])


def test_gimbal_pins_full_identity_and_stops_on_source_change(f):
    seen = []
    f.node.create_subscription(GimbalState, '/gimbal/state', seen.append, 10)
    publisher = f.node.create_publisher(WorldTargetArrayV2, '/perception/v2/world_targets', 10)
    state_pub = f.node.create_publisher(UAVState, '/uav_state', 10)
    request = f.node.create_client(SetGimbalTracking, '/gimbal/set_tracking')
    f.run('easyfly_gimbal_control', 'gimbal_control_node')
    f.until(lambda: publisher.get_subscription_count() and request.service_is_ready()); f.spin(.2)
    tracking, world = Producer(), Producer()
    def send():
        m = native_world(f, tracking, world)
        state_pub.publish(UAVState(header=m.context.header, connected=True, pose_valid=True, attitude_valid=True))
        publisher.publish(m); f.spin(.03)
    send()
    future = request.call_async(SetGimbalTracking.Request(enabled=True, target_id=7)); f.until(future.done)
    assert future.result().success, future.result().message
    f.until(lambda: seen and seen[-1].is_tracking)
    world.reset(); send()
    f.until(lambda: not seen[-1].is_tracking)
    for _ in range(3): send()
    assert not seen[-1].is_tracking, 'same numeric alias silently resumed after source change'


def test_single_native_service_real_csrt_and_legacy_control_isolated(f):
    from easyfly_msgs.srv import ManageSingleTrack
    seen = []
    f.node.create_subscription(SingleTrackStateV2, '/perception/v2/single_track/state', seen.append, 10)
    publisher = f.node.create_publisher(Image, '/camera/image_raw', 10)
    request = f.node.create_client(ManageSingleTrackV2, '/single_track/v2/manage')
    legacy = f.node.create_client(ManageSingleTrack, '/single_track/manage')
    f.run('easyfly_single_object_tracking', 'single_object_tracking_node')
    f.until(lambda: publisher.get_subscription_count() and request.service_is_ready() and legacy.service_is_ready())
    f.spin(.15)
    data = bytearray([30]) * (200 * 160)
    for y in range(40):
        for x in range(40): data[(50+y)*200+60+x] = ((x*127+y*53) ^ (x*y*13)) % 256
    image = Image(width=200, height=160, step=200, encoding='mono8', data=data,
        header=Header(frame_id='camera_color_optical_frame', stamp=f.node.get_clock().now().to_msg()))
    publisher.publish(image); f.spin(.04)
    future = legacy.call_async(ManageSingleTrack.Request(operation='reset')); f.until(future.done)
    assert not future.result().accepted
    future = request.call_async(ManageSingleTrackV2.Request(operation='initialize', image_header=image.header,
        roi_x=.3, roi_y=.3125, roi_width=.2, roi_height=.25, class_name='manual'))
    f.until(future.done); assert future.result().accepted, future.result().message
    f.until(lambda: any(s.valid for s in seen))
    state = next(s for s in seen if s.valid)
    assert state.context.image_width == 200 and state.context.image_height == 160
    assert state.track.key.tracker_session_id == state.context.producer_session_id
    assert not state.track.confidence_valid and state.track.observed_this_frame
    future = request.call_async(ManageSingleTrackV2.Request(operation='reset')); f.until(future.done)
    assert future.result().accepted and not future.result().state.valid
    assert future.result().state.context.epoch > state.context.epoch


def test_follow_aborts_and_drains_child_when_native_source_changes(f):
    import threading
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from easyfly_msgs.action import TrackTarget, GoToWaypoint
    from action_msgs.msg import GoalStatus
    fake = rclpy.create_node('ext02_waypoint_fixture')
    group = ReentrantCallbackGroup()
    children, finished = [], []
    shutdown, change = threading.Event(), threading.Event()
    def execute(handle):
        children.append(handle)
        end = time.monotonic() + 15
        while not handle.is_cancel_requested and not shutdown.is_set() and time.monotonic() < end:
            time.sleep(.01)
        if handle.is_cancel_requested:
            handle.canceled(); finished.append('canceled')
        else:
            handle.abort(); finished.append('aborted')
        return GoToWaypoint.Result(success=False, message='software fixture terminal')
    server = ActionServer(fake, GoToWaypoint, '/goto_waypoint', execute_callback=execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
    publisher = fake.create_publisher(WorldTargetArrayV2, '/perception/v2/world_targets', 10)
    state_pub = fake.create_publisher(UAVState, '/uav_state', 10)
    tracker, world = Producer(), Producer()
    def publish():
        if change.is_set(): world.reset(); change.clear()
        m = native_world(f, tracker, world)
        state_pub.publish(UAVState(header=m.context.header, connected=True, pose_valid=True, position_z=1.5))
        publisher.publish(m)
    fake.create_timer(.05, publish, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=2); executor.add_node(fake)
    thread = threading.Thread(target=executor.spin); thread.start()
    try:
        client = ActionClient(f.node, TrackTarget, '/track_target')
        f.run('easyfly_target_tracking', 'target_tracking_node')
        f.until(lambda: client.server_is_ready() and publisher.get_subscription_count()); f.spin(.2)
        request = client.send_goal_async(TrackTarget.Goal(target_id=7, follow_distance=1., follow_height=1.5))
        f.until(request.done); assert request.result().accepted
        result = request.result().get_result_async()
        f.until(lambda: children)
        change.set()
        f.until(result.done)
        assert result.result().status == GoalStatus.STATUS_ABORTED
        assert 'perception_source_changed_or_invalid' in result.result().result.message
        assert finished == ['canceled'], 'parent became terminal before draining its child'
    finally:
        shutdown.set(); executor.shutdown(timeout_sec=3); thread.join(timeout=3)
        server.destroy(); fake.destroy_node()


def test_reid_real_appearance_preserves_native_key_and_observed_crop(f):
    from ament_index_python.packages import get_package_share_directory
    from easyfly_msgs.msg import VehicleIdentityArrayV2
    from easyfly_vehicle_reid.node import Reidentification
    from easyfly_vehicle_reid.appearance import AppearanceModel
    camera_id = 'front'
    config = dict(version=1, use_sim_time=False, threshold=.42, margin=.08, perception_version=2,
        cameras=[dict(id=camera_id, image='/ext02/reid_image', observed_tracks='/ext02/reid_tracks',
                      frame='camera_color_optical_frame')])
    model = AppearanceModel(Path(get_package_share_directory('easyfly_vehicle_reid')) /
                            'models/osnet_ain_x1_0_vehicle_reid.onnx')
    node = Reidentification(config, model, f.node.context)
    from rclpy.executors import SingleThreadedExecutor
    reid_executor = SingleThreadedExecutor(context=f.node.context); reid_executor.add_node(node)
    received = []
    original_receive = node.receive
    def record_receive(camera, kind, message):
        received.append(dict(kind=kind, stamp=ns(message.header), frame=message.header.frame_id))
        return original_receive(camera, kind, message)
    node.receive = record_receive
    try:
        images = f.node.create_publisher(Image, '/ext02/reid_image', 10)
        tracks = f.node.create_publisher(ImageTrackArrayV2, '/ext02/reid_tracks', 10)
        seen = []
        f.node.create_subscription(VehicleIdentityArrayV2, '/perception/v2/vehicle_identities', seen.append, 10)
        def drive(condition):
            reid_executor.spin_once(timeout_sec=.005)
            return condition()
        f.until(lambda: drive(lambda: images.get_subscription_count() and tracks.get_subscription_count() and
                node.inputs[camera_id]['gids'] and all(node.inputs[camera_id]['gids'])))
        h = Header(frame_id='camera_color_optical_frame', stamp=f.node.get_clock().now().to_msg())
        producer, detector = Producer(), Producer()
        c = producer.next(h, camera_id, 96, 96)
        key = TrackKeyV2(tracker_session_id=c.producer_session_id, raw_id=2**40,
                        legacy_id_valid=True, legacy_raw_id=7, legacy_stable_id=7)
        observed = PixelBoxV2(x1=16., y1=16., x2=80., y2=80.)
        t = ImageTrackV2(key=key, filtered_box=PixelBoxV2(x1=2., y1=2., x2=12., y2=12.),
            observed_box=observed, filtered_box_valid=True, observed_box_valid=True,
            observed_this_frame=True, class_name='car', confidence_valid=True, confidence=.9)
        expected_refs = []
        pixels = bytes((x*13+x//47) % 256 for x in range(96*96*3))
        for _ in range(6):
            h = Header(frame_id='camera_color_optical_frame', stamp=f.node.get_clock().now().to_msg())
            c = producer.next(h, camera_id, 96, 96); expected_refs.append(reference(c))
            images.publish(Image(header=h, width=96, height=96, encoding='rgb8', step=288, data=pixels))
            tracks.publish(ImageTrackArrayV2(context=c, upstream=reference(detector.next(h, camera_id, 96, 96)), tracks=[t]))
            end = time.monotonic() + .07
            while time.monotonic() < end:
                reid_executor.spin_once(timeout_sec=.005); f.spin(.005)
            if any(x.valid for x in seen): break
        try:
            f.until(lambda: drive(lambda: any(x.valid for x in seen)))
        finally:
            row = node.inputs[camera_id]
            (f.directory / 'reid-state.json').write_text(json.dumps(dict(
                last=row['last'], reason=row['reason'], valid=row['valid'],
                images=list(row['images']), tracks=list(row['tracks']), native=list(row['native']),
                seen=[dict(valid=m.valid, reason=m.reason) for m in seen], gids=row['gids'], received=received,
                ros_now=node.get_clock().now().nanoseconds, clock_ns=node.clock_ns), indent=2))
        result = next(x for x in seen if x.valid)
        assert result.upstream in expected_refs
        assert len(result.identities) == 1 and result.identities[0].key == key
        assert result.identities[0].observed_box == observed
        assert result.identities[0].state != 'invalid_crop'
    finally:
        reid_executor.shutdown()
        node.destroy_node()


def test_onboard_native_observer_uses_verified_writer_and_rejects_bad_envelope(f):
    from easyfly_connection_manager.onboard_observer import PlatformObserver
    topic = '/perception/v2/detections'
    recipe = dict(components=dict(detection=dict(enabled=True,
        parameters=dict(perception_version=2, camera_id='front'), output_topic=topic, remappings={})))
    observer = PlatformObserver(recipe)
    try:
        publisher = f.node.create_publisher(DetectionArrayV2, topic, 10)
        producer = Producer()
        f.until(lambda: publisher.get_subscription_count()); f.spin(.2)
        last = None
        for _ in range(5):
            h = Header(frame_id='camera_color_optical_frame', stamp=f.node.get_clock().now().to_msg())
            last = DetectionArrayV2(context=producer.next(h, 'front', 64, 48), detections=[
                DetectionV2(box=PixelBoxV2(x1=1., y1=1., x2=10., y2=10.), class_id=3,
                            class_name='car', confidence_valid=True, confidence=.9)])
            publisher.publish(last); f.spin(.05)
        f.until(lambda: 'detection' in observer.native_outputs and observer.output_stamps.get('detection', (0,0))[1] >= 2)
        assert observer.native_outputs['detection'].detections[0].class_id == 3
        h = Header(frame_id='camera_color_optical_frame', stamp=f.node.get_clock().now().to_msg())
        last.context = producer.next(h, 'front', 0, 0)
        publisher.publish(last)
        f.until(lambda: 'detection' not in observer.native_outputs and 'detection' not in observer.output_stamps)
    finally:
        observer.close()


@pytest.mark.parametrize('version', [1, 2])
def test_legacy_observations_never_manufacture_native_query_evidence(f, version):
    from easyfly_msgs.msg import Target3D
    from easyfly_msgs.srv import GetPerception
    publisher = f.node.create_publisher(Target3DArray, '/target_positions', 10)
    legacy = f.node.create_client(GetPerception, '/platform/perception/query')
    native = f.node.create_client(GetPerceptionV2, '/platform/perception/v2/query')
    f.run('easyfly_perception_srvs', 'perception_services', version=version)
    f.until(lambda: legacy.service_is_ready() and native.service_is_ready()); f.spin(.2)
    h = Header(frame_id='odom', stamp=f.node.get_clock().now().to_msg())
    publisher.publish(Target3DArray(header=h, targets=[Target3D(header=h, track_id=7,
        class_name='person', confidence=.9, position_valid=True, position_x=1., position_z=1., distance=2.)]))
    f.spin(.08)
    old = legacy.call_async(GetPerception.Request(kind='targets')); f.until(old.done)
    assert old.result().status == (GetPerception.Response.VALID if version == 1 else GetPerception.Response.NOT_READY)
    new = native.call_async(GetPerceptionV2.Request(kind='targets')); f.until(new.done)
    assert new.result().status == GetPerceptionV2.Response.NOT_READY
    assert not any(new.result().targets.context.producer_session_id.uuid)
    assert not new.result().targets.targets
