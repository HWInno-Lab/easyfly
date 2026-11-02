# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""EXT-04 real node selection/parameter routes, isolated from flight services."""
from pathlib import Path
import json
import resource
import signal
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest
import yaml

from test_ext02_native import Fixture, Producer, DetectionArrayV2, DetectionV2, PixelBoxV2, ImageTrackArrayV2


@pytest.mark.parametrize('package,executable,parameters,reason', [
    ('easyfly_multi_object_tracking', 'tracking_node', ['tracker_backend:=fixture_missing'], 'fixture_missing'),
    ('easyfly_multi_object_tracking', 'tracking_node', ['backend_parameters:={min_hits: true}'], 'integer required'),
    ('easyfly_global_planner', 'global_planner_node', ['planner_backend:=fixture_missing'], 'unknown backend'),
    ('easyfly_global_planner', 'navigation_coordinator_node', ['combined_planner_backend:=fixture_missing'], 'unknown backend'),
    ('easyfly_global_planner', 'navigation_coordinator_node', ['combined_planner_parameters:={max_cells: 0}'], 'invalid backend parameter'),
    ('easyfly_object_detection', 'detection_node', ['backend:=fixture_missing'], 'fixture_missing'),
    ('easyfly_object_detection', 'detection_node', ['use_gpu_override:=false', 'backend_parameters:={use_gpu: true}'], 'conflicting CLI use_gpu'),
    ('easyfly_object_detection', 'detection_node', ['backend_parameters:={vendor: 1}'], 'unknown or duplicate backend parameter'),
    ('easyfly_single_object_tracking', 'single_object_tracking_node', ['tracker_backend:=fixture_missing'], 'no C++ parameter schema'),
    ('easyfly_single_object_tracking', 'single_object_tracking_node', ['backend_parameters:={threshold: 1}'], 'unknown or duplicate backend parameter'),
])
def test_reject_configuration_before_worker_or_control(package, executable, parameters, reason, tmp_path):
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    arguments = [str(path), '--ros-args']
    for parameter in parameters:
        key, value = parameter.split(':=', 1)
        arguments += ['-p', key + ':=' + (json.dumps(value) if value.startswith('{') or key == 'use_gpu_override' else value)]
    with (tmp_path / 'startup.log').open('w+') as log:
        process = subprocess.Popen(arguments, stdout=log, stderr=log, start_new_session=True,
            preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_CORE, (0, 0)))
        try:
            rc = process.wait(timeout=15)
            log.seek(0)
            text = log.read()
            assert rc != 0 and reason in text, text
        finally:
            if process.poll() is None:
                import os
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)


def test_tracking_cli_backend_parameters_reach_real_sort(tmp_path):
    config = tmp_path / 'tracking.yaml'
    config.write_text(yaml.safe_dump({'tracking': dict(tracker_backend='sort', publish_debug_image=False,
        image_width=64, image_height=48, min_hits=3, max_tracks=100)}))
    fixture = Fixture(tmp_path)
    try:
        seen = []
        fixture.node.create_subscription(ImageTrackArrayV2, '/perception/v2/tracks', seen.append, 10)
        publisher = fixture.node.create_publisher(DetectionArrayV2, '/perception/v2/detections', 10)
        fixture.run('easyfly_multi_object_tracking', 'tracking_node', '-p', 'config_file:=' + str(config),
                    '-p', 'backend_parameters:=' + json.dumps('{min_hits: 1, max_tracks: 1}'))
        fixture.until(lambda: publisher.get_subscription_count() > 0)
        fixture.spin(.4)
        producer = Producer()
        for _ in range(3):
            from std_msgs.msg import Header
            header = Header(frame_id='camera_color_optical_frame', stamp=fixture.node.get_clock().now().to_msg())
            message = DetectionArrayV2(context=producer.next(header, 'front', 64, 48), backend_id='onnx', active_provider='fixture')
            message.detections = [DetectionV2(box=PixelBoxV2(x1=x, y1=8., x2=x+8., y2=20.),
                class_id=0, class_name='person', confidence_valid=True, confidence=.9) for x in (2., 44.)]
            publisher.publish(message)
            fixture.spin(.06)
        fixture.until(lambda: any(m.context.status == 0 and m.tracks for m in seen))
        valid = [m for m in seen if m.context.status == 0 and m.tracks]
        # Factory max_tracks and min_hits change live native output, not only logs.
        assert all(len(m.tracks) == 1 and m.tracks[0].confirmed for m in valid)
        assert all(m.backend_id == 'sort' for m in valid)
    finally:
        fixture.close()


def test_tracking_file_legacy_and_native_conflict_is_rejected(tmp_path):
    config = tmp_path / 'conflict.yaml'
    config.write_text(yaml.safe_dump({'tracking': dict(min_hits=3, backend_parameters=dict(min_hits=1))}))
    test_reject_configuration_before_worker_or_control('easyfly_multi_object_tracking', 'tracking_node',
        ['config_file:=' + str(config)], 'legacy/new tracking parameter conflict', tmp_path)
