#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Acquire original Gazebo RGB-D on the already qualified FAST-LIVO2 flight chain."""
import importlib.util
from pathlib import Path
import subprocess

spec = importlib.util.spec_from_file_location('orb_capture_fixture', Path(__file__).with_name('fastlivo_simulation.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


class Capture(fixture.Probe):
    def launch_command(self):
        log = (self.directory/'rgbd-recorder.log').open('x')
        proc = subprocess.Popen([
            'ros2', 'bag', 'record', '-o', str(self.directory/'rgbd-bag'),
            '/clock', '/livox/imu', '/fastlivo/input/imu',
            '/camera/color/image_raw', '/camera/color/camera_info',
            '/camera/aligned_depth_to_color/depth/image_raw',
            '/camera/aligned_depth_to_color/depth/camera_info',
        ], env=self.environment, stdout=log, stderr=subprocess.STDOUT)
        self.extra_processes.append(('rgbd_recorder', proc, log))
        self.report['orbslam3_scope'] = 'original RGB-D acquisition only; flight remains qualified FAST-LIVO2'
        return super().launch_command()


if __name__ == '__main__':
    fixture.Probe = Capture
    raise SystemExit(fixture.main())
