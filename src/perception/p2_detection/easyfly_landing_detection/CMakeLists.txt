# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_landing_detection/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_landing_detection)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(easyfly_msgs REQUIRED)

add_executable(landing_detection_node src/landing_node.cpp)
target_compile_features(landing_detection_node PRIVATE cxx_std_17)
target_include_directories(landing_detection_node PRIVATE include)
ament_target_dependencies(landing_detection_node rclcpp sensor_msgs easyfly_msgs)

install(TARGETS landing_detection_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_timing_trace test/test_timing_trace.cpp)
  target_compile_features(test_timing_trace PRIVATE cxx_std_17)
  target_include_directories(test_timing_trace PRIVATE include)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_landing_e2e test/test_landing_e2e.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=106 ROS_LOG_DIR=/tmp/easyfly_d4_landing_e2e)
  ament_add_pytest_test(test_clock_ros test/test_clock_ros.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=106 ROS_LOG_DIR=/tmp/easyfly_d4_clock_106)
  ament_add_pytest_test(test_landing_ros test/test_landing_ros.py TIMEOUT 60
    ENV ROS_DOMAIN_ID=106 ROS_LOG_DIR=/tmp/easyfly_d4_landing_ros)
endif()

# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
