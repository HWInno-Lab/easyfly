# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_lidar_atlas/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_lidar_atlas)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/lidar_atlas DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_geometry test/test_geometry.py TIMEOUT 30)
  ament_add_pytest_test(test_archive test/test_archive.py TIMEOUT 30)
  ament_add_pytest_test(test_engine test/test_engine.py TIMEOUT 30)
  ament_add_pytest_test(test_input test/test_input.py TIMEOUT 30)
  ament_add_pytest_test(test_node_ros test/test_node_ros.py TIMEOUT 45 ENV ROS_DOMAIN_ID=231)
endif()
# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
