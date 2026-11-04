# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_time_sync/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_time_sync)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(easyfly_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(time_sync_node src/time_sync_node.cpp)
target_include_directories(time_sync_node PRIVATE include)
target_compile_options(time_sync_node PRIVATE -Wall -Wextra)
ament_target_dependencies(time_sync_node rclcpp sensor_msgs easyfly_msgs)
install(TARGETS time_sync_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pairing test/test_pairing.cpp)
  target_include_directories(test_pairing PRIVATE include)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_time_sync_ros test/test_time_sync_ros.py TIMEOUT 40
    ENV ROS_DOMAIN_ID=105 ROS_LOG_DIR=/tmp/easyfly_d4_time_sync)
endif()

# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
