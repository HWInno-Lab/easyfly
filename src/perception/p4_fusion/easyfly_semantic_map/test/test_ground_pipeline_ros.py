# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Actual ground lifecycle, without simulator or flight actions."""
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import UAVState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def test_ground_init_armed_freeze_basis_jump_recovery_and_stale_tf(tmp_path):
    rclpy.init()
    peer = rclpy.create_node('alg71_ground_pipeline')
    clouds = peer.create_publisher(PointCloud2, '/pointcloud_filtered', 10)
    states = peer.create_publisher(UAVState, '/uav_state', 10)
    broadcaster = TransformBroadcaster(peer)
    grids, nonground = [], []
    peer.create_subscription(OccupancyGrid, '/occupancy_grid', grids.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    peer.create_subscription(PointCloud2, '/pointcloud_nonground', nonground.append, 10)
    fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
    state = dict(armed=True, floor=0., basis=0., publish_basis=True)
    sent = [0]

    def publish():
        header = Header(frame_id='odom', stamp=peer.get_clock().now().to_msg())
        vehicle = TransformStamped(header=header, child_frame_id='base_link')
        vehicle.transform.translation.z = 1.
        vehicle.transform.rotation.w = 1.
        transforms = [vehicle]
        if state['publish_basis']:
            basis = TransformStamped(header=Header(frame_id='map', stamp=header.stamp), child_frame_id='odom')
            basis.transform.translation.z = state['basis']
            basis.transform.rotation.w = 1.
            transforms.append(basis)
        broadcaster.sendTransform(transforms)
        states.publish(UAVState(header=header, connected=True, pose_valid=True,
                                armed=state['armed'], position_z=1.))
        points = [(x*.1, y*.1, state['floor'], 1.) for x in range(-20, 21) for y in range(-20, 21)]
        points += [(x*.05+.5, y*.05+.5, .2, 2.) for x in range(5) for y in range(5)]
        clouds.publish(create_cloud(header, fields, points))
        sent[0] += 1

    timer = peer.create_timer(.1, publish)
    executable = Path(get_package_prefix('easyfly_semantic_map')) / 'lib/easyfly_semantic_map/semantic_map_node'
    process = None
    log_path = tmp_path / 'ground.log'

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(peer, timeout_sec=.01)

    def until(predicate, timeout=8.):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, log_path.read_text()
            spin(.03)

    def ready():
        return len([g for g in grids if g.header.stamp.sec > 0]) >= 4

    def low_obstacle_present():
        observed = [cloud for cloud in nonground if cloud.width > 0]
        return bool(observed) and any(abs(float(p[0])-.2) < .015
                                     for p in read_points(observed[-1], field_names=('z',), skip_nans=True))

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([str(executable), '--ros-args', '-p', 'use_sim_time:=false',
                                        '-p', 'ground_model_frame:=map'], stdout=log, stderr=subprocess.STDOUT)
            until(lambda: clouds.get_subscription_count() > 0 and states.get_subscription_count() > 0)
            spin(1.)
            assert sent[0] >= 8 and grids and not any(g.header.stamp.sec > 0 for g in grids)
            state['armed'] = False
            grids.clear()
            until(ready)
            assert low_obstacle_present()
            # A fresh but false .3m candidate must not move the trusted plane while armed.
            state.update(armed=True, floor=.3)
            grids.clear()
            nonground.clear()
            spin(1.2)
            assert ready() and low_obstacle_present(), log_path.read_text()
            # A >.25m basis jump resets the model. An armed vehicle cannot reinitialize it.
            state['basis'] = .3
            sent_before_jump = sent[0]
            grids.clear()
            until(lambda: any(g.header.stamp.sec == 0 for g in grids))
            jump_invalid = next(i for i, g in enumerate(grids) if g.header.stamp.sec == 0)
            spin(.8)
            # The node coalesces repeated invalid maps; require no ready output under fresh traffic.
            assert sent[0] >= sent_before_jump + 7
            assert all(g.header.stamp.sec == 0 for g in grids[jump_invalid:])
            state.update(armed=False, floor=0.)
            grids.clear()
            nonground.clear()
            until(ready)
            assert low_obstacle_present()
            # Keep cloud/state/vehicle TF fresh while only the ground basis ages out.
            state['publish_basis'] = False
            grids.clear()
            until(lambda: any(g.header.stamp.sec == 0 for g in grids), timeout=4.)
            first_invalid = next(i for i, g in enumerate(grids) if g.header.stamp.sec == 0)
            sent_before_stale = sent[0]
            spin(.4)
            assert sent[0] >= sent_before_stale + 3
            assert all(g.header.stamp.sec == 0 for g in grids[first_invalid:])
            assert '地面模型坐标变换过期' in log_path.read_text()
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=4.) == 0, log_path.read_text()
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=2.)
        peer.destroy_timer(timer)
        peer.destroy_node()
        rclpy.shutdown()
