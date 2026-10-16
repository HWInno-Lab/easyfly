cmake_minimum_required(VERSION 3.8)
project(easyfly_gazebo_plugins)

find_package(ament_cmake REQUIRED)

# 阶段三 M0: 启用 Livox MID-360 仿真插件 (ROS2 移植版)
add_subdirectory(livox_laser_plugin)

# M4.1: 巡航移动目标驱动 (纯 Gazebo 插件, 无 ROS 依赖)
add_subdirectory(patrol_mover)

# Filter native IMU samples before the existing 4:1 MAVLink decimator.
add_subdirectory(imu_antialias)

# Independently named upstream camera with a fail-closed duplicate-node guard.
add_subdirectory(safe_camera)

# Exact-version, process-local transport lifetime compatibility server.
add_subdirectory(safe_transport)

# realsense_plugin / velodyne_plugin: 暂不启用
#   - 深度相机仿真使用系统自带 libgazebo_ros_camera.so (无第三方依赖)
#   - velodyne 暂无需求

ament_package()
