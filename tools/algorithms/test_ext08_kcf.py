# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Real KCF extension checks: selection, installed launch and ROS image/ROI flow.

Only synthetic camera frames and read-only tracking services; no flight actions.
Run through verify_extension_migration.py with an explicit KCF overlay.
"""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from easyfly_backend_registry import Registry, fields
from easyfly_params.backends import resolve_selection, launch_values
from easyfly_perception_contracts.backend_status import BackendMonitor
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from easyfly_msgs.msg import SingleTrackStateV2
from easyfly_msgs.srv import ManageSingleTrack, ManageSingleTrackV2
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from test_ext02_native import Fixture


def test_kcf_registry_ui_config_and_leaf_launch_preserve_csrt_default():
    registry = Registry()
    assert registry.ids('single_tracking') == ['csrt', 'kcf']
    assert registry.parameters('single_tracking', 'kcf') == {'detect_thresh': .5}
    assert resolve_selection('full_perception')['selection']['backends']['single_tracking']['backend'] == 'csrt'
    field = next(f for f in fields() if f['key'] == 'single_tracking_backend')
    assert 'kcf' in dict(field['choices'])
    patch = {'backends': {'single_tracking': {'parameters': {'detect_thresh': .25}}}}
    resolved = resolve_selection('full_perception', {'single_tracking_backend': 'kcf'}, cli_config=patch)
    assert resolved['selection']['backends']['single_tracking'] == dict(backend='kcf', parameters={'detect_thresh': .25})
    values = launch_values(resolved)
    assert values['single_tracking_backend'] == 'kcf'
    assert json.loads(values['single_tracking_parameters']) == {'detect_thresh': .25}
    path = Path(get_package_share_directory('easyfly_single_object_tracking'))/'launch/single_object_tracking.launch.py'
    spec = importlib.util.spec_from_file_location('ext08_single_launch', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(tracker_backend=values['single_tracking_backend'],
        backend_parameters=values['single_tracking_parameters'], perception_version='2')
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument): action.execute(context)
    node = next(n for n in description.entities if isinstance(n, Node))
    actual = evaluate_parameters(context, node._Node__parameters)[0]
    assert actual['tracker_backend'] == 'kcf'
    assert json.loads(actual['backend_parameters']) == {'detect_thresh': .25}
    assert actual['perception_version'] == 2
    for bad in [True, -1., 1.1, float('nan')]:
        with pytest.raises(Exception): registry.parameters('single_tracking', 'kcf', {'detect_thresh': bad})
    with pytest.raises(Exception): registry.parameters('single_tracking', 'csrt', {'detect_thresh': .25})


def test_kcf_desktop_and_terminal_transfer_same_selection():
    from easyfly_connection_manager.backend import command
    options = dict(single_tracking_backend='kcf', provider='cpu')
    profile = dict(mode='sitl', selected_profile='full_perception', algorithms=options,
        simulation=dict(scene='warehouse_aws_v1', gazebo_gui=False),
        fcu=dict(transport='udp', bind_host='127.0.0.1', bind_port=14540,
                 remote_host='127.0.0.1', remote_port=14580, target_system=1),
        gcs=dict(enabled=False, host='127.0.0.1', port=14560, bind_port=14558))
    arguments = command(profile)  # Inspect only; do not execute simulator or flight processes.
    transferred = json.loads(next(a.split(':=', 1)[1] for a in arguments if a.startswith('algorithms_json:=')))
    desktop = resolve_selection('full_perception', transferred)
    terminal = resolve_selection('full_perception', options)
    assert desktop['selection'] == terminal['selection']
    assert desktop['selection']['backends']['single_tracking']['backend'] == 'kcf'


def test_real_kcf_node_translation_diagnostics_reset_expiry_and_legacy_isolation(tmp_path):
    f = Fixture(tmp_path)
    try:
        resolved = resolve_selection('full_perception', {'single_tracking_backend': 'kcf'},
            cli_config={'backends': {'single_tracking': {'parameters': {'detect_thresh': .2}}}})
        resolved['enabled_roles'] = ['single_tracking']
        monitor = BackendMonitor(f.node, resolved)
        seen = []
        f.node.create_subscription(SingleTrackStateV2, '/perception/v2/single_track/state', seen.append, 10)
        publisher = f.node.create_publisher(Image, '/camera/image_raw', 10)
        manage = f.node.create_client(ManageSingleTrackV2, '/single_track/v2/manage')
        legacy = f.node.create_client(ManageSingleTrack, '/single_track/manage')
        values = launch_values(resolved)
        f.run('easyfly_single_object_tracking', 'single_object_tracking_node',
            '-p', 'tracker_backend:='+values['single_tracking_backend'],
            '-p', 'backend_parameters:='+json.dumps(values['single_tracking_parameters']),
            '-p', 'max_age_seconds:=1.0')
        f.until(lambda: manage.service_is_ready() and legacy.service_is_ready()
                and publisher.get_subscription_count() > 0
                and monitor.snapshot()['single_tracking']['created'])
        row = monitor.snapshot()['single_tracking']
        assert row['active_backend'] == 'kcf' and not row['ready']
        assert row['effective_parameters'] == {'detect_thresh': .2}
        texture = np.random.default_rng(42).integers(0, 256, (40, 40, 3), dtype=np.uint8)

        def publish(dx=0, dy=0):
            pixels = np.full((160, 200, 3), 30, dtype=np.uint8)
            pixels[50+dy:90+dy, 60+dx:100+dx] = texture
            message = Image(width=200, height=160, step=600, encoding='bgr8', data=pixels.tobytes(),
                header=Header(frame_id='camera_color_optical_frame', stamp=f.node.get_clock().now().to_msg()))
            publisher.publish(message); f.spin(.08)
            return message

        def call(operation, message=None):
            request = ManageSingleTrackV2.Request(operation=operation, roi_x=.3, roi_y=.3125,
                roi_width=.2, roi_height=.25, class_name='texture')
            if message is not None: request.image_header = message.header
            future = manage.call_async(request); f.until(future.done)
            assert future.result().accepted, future.result().message
            return future.result()

        f.spin(.2)
        initial = publish()
        call('initialize', initial)
        f.until(lambda: any(s.valid and s.context.header.stamp == initial.header.stamp for s in seen))
        first = next(s for s in reversed(seen) if s.valid)
        future = legacy.call_async(ManageSingleTrack.Request(operation='reset'))
        f.until(future.done); assert not future.result().accepted
        for dx, dy in [(4, 2), (8, 4), (12, 6)]:
            message = publish(dx, dy)
            f.until(lambda: any(s.valid and s.context.header.stamp == message.header.stamp for s in seen))
            state = next(s for s in reversed(seen) if s.valid and s.context.header.stamp == message.header.stamp)
            assert abs(state.track.filtered_box.x1 - (60+dx)) <= 3
            assert abs(state.track.filtered_box.y1 - (50+dy)) <= 3
            assert state.track.key == first.track.key
            assert state.track.observed_this_frame and not state.track.confidence_valid
            assert state.context.image_width == 200 and state.context.image_height == 160
        f.until(lambda: monitor.snapshot()['single_tracking']['ready'])
        reset = call('reset')
        assert not reset.state.valid and reset.state.context.epoch > first.context.epoch
        image = publish(); call('initialize', image)
        f.until(lambda: any(s.valid and s.context.epoch > reset.state.context.epoch for s in seen))
        second = next(s for s in reversed(seen) if s.valid)
        assert second.track.key != first.track.key
        f.spin(1.3)
        f.until(lambda: seen and not seen[-1].valid and not monitor.snapshot()['single_tracking']['ready'])
    finally:
        f.close()
