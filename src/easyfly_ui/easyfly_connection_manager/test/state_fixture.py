# Copyright 2026 EasyFly Project
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

"""Publish isolated typed observations and record any unexpected control requests."""
import argparse
import json
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from easyfly_msgs import action
from easyfly_msgs.msg import UAVCommand, UAVState
from easyfly_msgs.srv import ManageMission
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--control', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--no-clock', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('connection_test_source')
    state_pub = node.create_publisher(UAVState, '/uav_state', 1)
    clock_pub = node.create_publisher(Clock, '/clock', 1) if not args.no_clock else None
    counts = dict(goals=0, cancels=0, commands=0, mission_writes=0, published=0)
    state = dict(ns=10_000_000_000, revision=-1)

    def command(_):
        counts['commands'] += 1

    subscription = node.create_subscription(UAVCommand, '/uav_command', command, 10)

    def goal(_):
        counts['goals'] += 1
        return GoalResponse.REJECT

    def cancel(_):
        counts['cancels'] += 1
        return CancelResponse.REJECT

    servers = []
    for name, kind in (
        ('/takeoff', action.Takeoff), ('/land', action.Land),
        ('/precision_land', action.PrecisionLand), ('/goto_waypoint', action.GoToWaypoint),
        ('/navigate', action.NavigateTo), ('/follow_trajectory', action.FollowTrajectory),
        ('/track_target', action.TrackTarget), ('/execute_mission', action.ExecuteMission),
        ('/search_area', action.SearchArea),
    ):
        servers.append(ActionServer(node, kind, name, execute_callback=lambda _: None,
                                    goal_callback=goal, cancel_callback=cancel))

    def mission(request, response):
        if request.operation in ('start', 'cancel'):
            counts['mission_writes'] += 1
        return response

    service = node.create_service(ManageMission, '/platform/mission/manage', mission)

    def tick():
        value = {}
        control = Path(args.control)
        if control.exists():
            value = json.loads(control.read_text())
        if value.get('revision', -1) != state['revision']:
            state['revision'] = value.get('revision', -1)
            if value.get('rewind'):
                state['ns'] = 1_000_000_000
        if value.get('mode') != 'pause':
            state['ns'] += 50_000_000
        stamp = Clock().clock
        stamp.sec, stamp.nanosec = divmod(state['ns'], 1_000_000_000)
        if clock_pub is not None:
            message = Clock()
            message.clock = stamp
            clock_pub.publish(message)
        message = UAVState()
        message.header.stamp = stamp
        message.header.frame_id = 'odom'
        message.connected = True
        message.pose_valid = message.velocity_valid = message.attitude_valid = True
        message.battery_valid = True
        message.position_x = args.x
        message.battery_remaining = 0.75
        message.battery_voltage = 12.0
        state_pub.publish(message)
        counts['published'] += 1
        Path(args.report).write_text(json.dumps(counts))

    timer = node.create_timer(0.05, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_subscription(subscription)
        node.destroy_service(service)
        for server in servers:
            server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        Path(args.report).write_text(json.dumps(counts))


if __name__ == '__main__':
    main()
