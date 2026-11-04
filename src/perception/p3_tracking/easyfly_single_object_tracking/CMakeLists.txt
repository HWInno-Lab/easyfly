# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_single_object_tracking/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_single_object_tracking)
find_package(ament_cmake REQUIRED)
find_package(easyfly_perception_contracts REQUIRED)
include_directories(${easyfly_perception_contracts_INCLUDE_DIRS})
find_package(easyfly_csrt REQUIRED)
find_package(easyfly_kcf REQUIRED)
find_package(easyfly_single_tracking_api REQUIRED)
find_package(easyfly_backend_registry REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(${PROJECT_NAME}_backends SHARED src/backend_factory.cpp)
target_compile_features(${PROJECT_NAME}_backends PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME}_backends PROPERTIES SOVERSION 1)
target_include_directories(${PROJECT_NAME}_backends PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_backends PUBLIC easyfly_single_tracking_api::easyfly_single_tracking_api
  easyfly_backend_registry::easyfly_backend_registry PRIVATE easyfly_csrt::easyfly_csrt
  easyfly_kcf::easyfly_kcf)
install(TARGETS ${PROJECT_NAME}_backends EXPORT singleBackends LIBRARY DESTINATION lib)
ament_export_targets(singleBackends HAS_LIBRARY_TARGET)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(OpenCV REQUIRED)
add_executable(single_object_tracking_node src/single_node.cpp)
target_compile_features(single_object_tracking_node PRIVATE cxx_std_17)
target_include_directories(single_object_tracking_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(single_object_tracking_node rclcpp diagnostic_msgs sensor_msgs easyfly_msgs)
target_link_libraries(single_object_tracking_node ${OpenCV_LIBRARIES} ${PROJECT_NAME}_backends yaml-cpp)
install(TARGETS single_object_tracking_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_backend_factory test/test_backend_factory.cpp)
  target_link_libraries(test_backend_factory ${PROJECT_NAME}_backends)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_clock_ros test/test_clock_ros.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=107 ROS_LOG_DIR=/tmp/easyfly_d4_clock_107)
  ament_add_pytest_test(test_single_ros test/test_single_ros.py TIMEOUT 60
    ENV ROS_DOMAIN_ID=107 ROS_LOG_DIR=/tmp/easyfly_d4_single_ros)
endif()

install(FILES include/safety.hpp include/track_core.hpp DESTINATION include/${PROJECT_NAME})
install(DIRECTORY include/easyfly_single_object_tracking DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package(CONFIG_EXTRAS cmake/backend_dependencies.cmake)
