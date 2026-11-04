# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_obstacle_avoidance/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_obstacle_avoidance)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

add_executable(obstacle_supervisor_node src/obstacle_supervisor_node.cpp)
target_compile_options(obstacle_supervisor_node PRIVATE -Wall -Wextra)
ament_target_dependencies(obstacle_supervisor_node rclcpp easyfly_msgs std_msgs std_srvs)
install(TARGETS obstacle_supervisor_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_supervisor test/test_supervisor.py
    TIMEOUT 45 ENV ROS_LOG_DIR=/tmp/easyfly_d1_supervisor_test ROS_DOMAIN_ID=85)
endif()

# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
