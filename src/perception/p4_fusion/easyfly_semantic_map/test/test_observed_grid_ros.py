# Copyright 2026 EasyFly Project
#
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

"""Actual-node room footprint regression. No simulator, vehicle or flight actions."""
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import UAVState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('mode,half_width', [
    ('cluster_boxes', 3), ('observed_cells', 3), ('observed_cells', 6)])
def test_room_grid_keeps_walls_and_corridor(tmp_path, mode, half_width):
    os.environ['ROS_DOMAIN_ID'] = '187'
    rclpy.init()
    peer = rclpy.create_node('observed_grid_peer')
    clouds = peer.create_publisher(PointCloud2, '/pointcloud_filtered', 10)
    states = peer.create_publisher(UAVState, '/uav_state', 10)
    tf = TransformBroadcaster(peer)
    grids = []
    peer.create_subscription(OccupancyGrid, '/occupancy_grid', grids.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(['x', 'y', 'z', 'intensity'])]
    points = [(x*.1, y*.1, 0., 1.) for x in range(-40, 41) for y in range(-40, 41)]
    for i in range(-half_width*10, half_width*10+1):
        for z in range(5, 16):
            points.extend([(i*.1, -3., z*.1, 1.),
                           (-float(half_width), i*.1, z*.1, 1.),
                           (float(half_width), i*.1, z*.1, 1.)])

    def publish():
        header = Header(frame_id='odom', stamp=peer.get_clock().now().to_msg())
        pose = TransformStamped(header=header, child_frame_id='base_link')
        pose.transform.translation.z = 1.
        pose.transform.rotation.w = 1.
        tf.sendTransform(pose)
        states.publish(UAVState(header=header, connected=True, pose_valid=True,
                                armed=False, position_z=1.))
        clouds.publish(create_cloud(header, fields, points))

    timer = peer.create_timer(.1, publish)
    executable = Path(get_package_prefix('easyfly_semantic_map')) / (
        'lib/easyfly_semantic_map/semantic_map_node')
    process = None
    try:
        with (tmp_path/'semantic.log').open('w') as output:
            command = [str(executable), '--ros-args', '-p',
                       'use_sim_time:=false', '-p', 'grid_footprint_mode:='+mode]
            process = subprocess.Popen(
                command,
                stdout=output, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 15.
            while len([g for g in grids if g.header.stamp.sec > 0]) < 5:
                assert process.poll() is None, (tmp_path/'semantic.log').read_text()
                assert time.monotonic() < deadline, (tmp_path/'semantic.log').read_text()
                rclpy.spin_once(peer, timeout_sec=.02)
            ready = [g for g in grids if g.header.stamp.sec > 0][-5:]
            for grid in ready:
                def occupied(x, y):
                    col = int((x-grid.info.origin.position.x)/grid.info.resolution)
                    row = int((y-grid.info.origin.position.y)/grid.info.resolution)
                    return grid.data[row*grid.info.width+col] >= 50
                assert occupied(0., -2.4) and occupied(half_width-.6, 0.)
                assert occupied(0., 0.) == (mode == 'cluster_boxes')
                assert occupied(1.5, 0.) == (mode == 'cluster_boxes')
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=4.) == 0
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=2.)
        peer.destroy_timer(timer)
        peer.destroy_node()
        rclpy.shutdown()
