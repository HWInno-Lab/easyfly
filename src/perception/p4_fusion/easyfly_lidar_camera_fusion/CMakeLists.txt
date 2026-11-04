# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_lidar_camera_fusion/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_lidar_camera_fusion)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/lidar_camera_fusion DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_projection test/test_projection.py)
  ament_add_pytest_test(test_pending test/test_pending.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=115 ROS_LOG_DIR=/tmp/easyfly_d4_fusion_pending)
  ament_add_pytest_test(test_fusion_ros test/test_fusion_ros.py TIMEOUT 60
    ENV ROS_DOMAIN_ID=110 ROS_LOG_DIR=/tmp/easyfly_d4_fusion)
  ament_add_pytest_test(test_clock_ros test/test_clock_ros.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=114 ROS_LOG_DIR=/tmp/easyfly_d4_fusion_clock)
endif()

# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
