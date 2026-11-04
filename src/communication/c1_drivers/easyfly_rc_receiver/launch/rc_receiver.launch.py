# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_rc_receiver/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
"""
rc_receiver.launch.py — 启动 easyfly_rc_receiver 节点

用法:
  ros2 launch easyfly_rc_receiver rc_receiver.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rc_node = Node(
        package="easyfly_rc_receiver",
        executable="rc_receiver_node",
        output="screen",
        name="rc_receiver",
    )

    return LaunchDescription([rc_node])
