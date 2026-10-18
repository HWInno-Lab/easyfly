# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Installed real detector: fixed RGB input, model resolution and CPU/CUDA output."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from easyfly_msgs.msg import DetectionInfoArray
import pytest
import rclpy
from sensor_msgs.msg import Image


@pytest.fixture
def ros_node():
    rclpy.init()
    node = rclpy.create_node('alg5_fixed_frame_detector')
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('gpu', [False, True])
def test_fixed_frame_installed_detector(gpu, tmp_path, ros_node):
    node = ros_node
    publisher = node.create_publisher(Image, '/camera/color/image_raw', 10)
    seen = []
    node.create_subscription(DetectionInfoArray, '/detections', seen.append, 10)
    package = Path(__file__).resolve().parents[1]
    baseline = json.loads(Path(os.environ.get(
        'ALG5_NATIVE_BASELINE', package / 'test/alg5_native_baseline.json')).read_text())
    model = Path(get_package_share_directory('easyfly_object_detection')) / 'models/yolo11n.onnx'
    assert model.is_file()
    executable = Path(get_package_prefix('easyfly_object_detection')) / (
        'lib/easyfly_object_detection/detection_node')
    log_path = tmp_path / 'detection.log'
    log = log_path.open('w')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'use_gpu_override:="' + str(gpu).lower() + '"',
        '-p', 'publish_debug_image_override:="false"'], stdout=log, stderr=log,
        start_new_session=True)
    try:
        end = time.monotonic() + 15
        while publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=.02)
            assert process.poll() is None, log_path.read_text()
            assert time.monotonic() < end, log_path.read_text()
        image = Image()
        image.header.stamp = node.get_clock().now().to_msg()
        image.header.frame_id = 'camera_color_optical_frame'
        image.width, image.height, image.step, image.encoding = 640, 480, 1920, 'rgb8'
        image.data = (package / 'test/m2_frame.bin').read_bytes()
        end = time.monotonic() + 10
        while not seen:
            publisher.publish(image)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=.02)
            assert time.monotonic() < end, log_path.read_text()
        output = seen[-1]
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            code = process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
            code = -9
        log.close()
        assert code == 0, log_path.read_text()
    diagnostic = log_path.read_text()
    actual_provider = ('CUDAExecutionProvider' if 'EP=CUDAExecutionProvider' in diagnostic
                       else 'CPUExecutionProvider')
    assert 'EP=' + actual_provider in diagnostic
    if not gpu:
        assert actual_provider == 'CPUExecutionProvider'
    expected = next(r['result'] for r in baseline['runs']
                    if r['result']['provider'] == actual_provider)
    assert output.header == image.header
    assert len(output.detections) == len(expected['detections']) == 5
    for actual, native in zip(output.detections, expected['detections']):
        assert actual.header == image.header
        assert actual.track_id == -1
        assert actual.class_name == native['class_name']
        assert actual.confidence == pytest.approx(native['confidence'], abs=1e-5)
        box = [(actual.bbox_center_x - actual.bbox_width / 2) * 640,
               (actual.bbox_center_y - actual.bbox_height / 2) * 480,
               (actual.bbox_center_x + actual.bbox_width / 2) * 640,
               (actual.bbox_center_y + actual.bbox_height / 2) * 480]
        assert box == pytest.approx(native['box'], abs=1e-3)
