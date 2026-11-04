# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_ellipse_detection/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.16)
project(easyfly_ellipse_detection)
find_package(ament_cmake REQUIRED)
find_package(easyfly_image_detection_common REQUIRED)
find_package(easyfly_ellipse REQUIRED)
add_executable(ellipse_detection_node src/ellipse_node.cpp)
target_compile_options(ellipse_detection_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ellipse_detection_node PRIVATE
  easyfly_image_detection_common::easyfly_image_detection_common easyfly_ellipse::easyfly_ellipse)
install(TARGETS ellipse_detection_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_detector_ros test/test_detector_ros.py TIMEOUT 100
    ENV ROS_DOMAIN_ID=163
    PYTHONNOUSERSITE=1 ROS_LOG_DIR=/tmp/easyfly_ellipse_test)
endif()
# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
