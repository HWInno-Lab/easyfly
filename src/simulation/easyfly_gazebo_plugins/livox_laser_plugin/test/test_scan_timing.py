# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual native ray cloud must carry one acquisition time, not future offsets."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

from easyfly_fastdds_compat.runtime import activate
activate()  # Before loading rclpy or generated ROS message libraries.
import rclpy
from ament_index_python.packages import get_package_share_directory
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import qos_profile_sensor_data


def main():
    server, plugin = sys.argv[1:]
    os.environ['ROS_DOMAIN_ID'] = '228'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
               GAZEBO_MODEL_DATABASE_URI='')
    rclpy.init()
    node = rclpy.create_node('livox_scan_timing_contract')
    samples = []
    node.create_subscription(CustomMsg, '/livox/lidar', samples.append, qos_profile_sensor_data)
    with tempfile.TemporaryDirectory(prefix='livox-scan-timing-') as directory:
        model_path = Path(get_package_share_directory('easyfly_gazebo_models')) / \
            'easyfly/runtime_models/mid360/mid360.sdf'
        model = ET.parse(model_path).getroot().find('model')
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = '0 0 1 0 0 0'
        for link in model.findall('link'):
            for sensor in list(link.findall('sensor')):
                if sensor.get('type') != 'ray':
                    link.remove(sensor)
        model.find('.//plugin').set('filename', plugin)
        root = ET.Element('sdf', version='1.6')
        world = ET.SubElement(root, 'world', name='scan_timing')
        world.append(model)
        physics = ET.SubElement(world, 'physics', name='default', type='ode')
        ET.SubElement(physics, 'max_step_size').text = '.002'
        ET.SubElement(physics, 'real_time_update_rate').text = '500'
        ground = ET.SubElement(world, 'model', name='ground')
        ET.SubElement(ground, 'static').text = 'true'
        collision = ET.SubElement(ET.SubElement(ground, 'link', name='body'),
                                  'collision', name='plane')
        plane = ET.SubElement(ET.SubElement(collision, 'geometry'), 'plane')
        ET.SubElement(plane, 'normal').text = '0 0 1'
        ET.SubElement(plane, 'size').text = '100 100'
        path = Path(directory)/'world.sdf'
        ET.ElementTree(root).write(path, encoding='unicode')
        with (Path(directory)/'server.log').open('w+') as log:
            process = subprocess.Popen([server, '-s', 'libgazebo_ros_init.so', str(path)],
                                       env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic()+20
                while len(samples) < 10:
                    assert process.poll() is None
                    assert time.monotonic() < deadline, 'no ten native scans'
                    rclpy.spin_once(node, timeout_sec=.1)
                previous = 0
                for msg in samples:
                    stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                    assert stamp > previous
                    assert msg.timebase == stamp
                    assert msg.point_num == len(msg.points) == 20000
                    assert any(p.x or p.y or p.z for p in msg.points)
                    assert all(p.offset_time == 0 for p in msg.points)
                    previous = stamp
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                node.destroy_node()
                rclpy.try_shutdown()
                log.seek(0)
                assert process.returncode == 0, log.read()
    print('10 real 20000-point scans: original acquisition stamp, zero offsets, clean exit')


if __name__ == '__main__':
    main()
