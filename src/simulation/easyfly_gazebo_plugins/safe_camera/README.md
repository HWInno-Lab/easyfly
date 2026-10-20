# Project-local camera load guard

The source and header are derived from Open Source Robotics Foundation's
`gazebo_ros_pkgs`, tag **3.9.0**, under Apache-2.0. Original copyright and license
notices are retained. Sources:

- https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.9.0/gazebo_plugins/src/gazebo_ros_camera.cpp
- https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.9.0/gazebo_plugins/include/gazebo_plugins/gazebo_ros_camera.hpp

Fetched upstream SHA-256: source
`0385ceec25a7b55f57aea38c060267bc593c4959a0f097ae4cb89561cf34a416`, header
`9d2f1b9808a21648bc852e5358e99ff3d96c9d47fe9544b117f1b675190486ec`.

EasyFly changes are deliberately limited to an independent class/library/header
name, a null-node check immediately after `gazebo_ros::Node::Get`, and reporting
the actual CameraInfo writer history depth in its existing startup log. Duplicate
ROS plugin names remain rejected: the second camera does not publish. No node
names are automatically changed, no original stream is replaced, and a disabled
camera does not authorize flight. Original image, depth, geometry, timestamp and
QoS behavior otherwise remains unchanged.

The library is `libeasyfly_gazebo_ros_camera.so`. It depends on the installed
Gazebo camera bases, multi-camera support and gazebo_ros, but does not link or
overwrite `libgazebo_ros_camera.so`. Switching the explicit SDF filename back to
the system library rolls back this guard. No system package upgrade is required.

This fixes the established duplicate-name null dereference, not the historical
Gazebo stop self-join or the unknown source of the old repeated model load.
