# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_multi_object_tracking/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_multi_object_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(easyfly_backend_registry REQUIRED)
include_directories(${easyfly_backend_registry_INCLUDE_DIRS})
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(easyfly_perception_contracts REQUIRED)
include_directories(${easyfly_perception_contracts_INCLUDE_DIRS})
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(easyfly_tracking_api REQUIRED)
find_package(easyfly_sort REQUIRED)
find_package(easyfly_track_id_fusion REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Adapter is ROS-independent; the SORT/ID implementations are independent packages.
add_library(${PROJECT_NAME}_backends SHARED src/adapters/sort_adapter.cpp src/adapters/tracker_factory.cpp)
set_target_properties(${PROJECT_NAME}_backends PROPERTIES SOVERSION 3)
target_include_directories(${PROJECT_NAME}_backends PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_backends PUBLIC
  easyfly_backend_registry::easyfly_backend_registry
  easyfly_tracking_api::easyfly_tracking_api
  PRIVATE easyfly_sort::easyfly_sort)
target_compile_options(${PROJECT_NAME}_backends PRIVATE -Wall -Wextra -Wpedantic)

add_library(${PROJECT_NAME}_nodes SHARED
  src/tracking_node.cpp
)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp diagnostic_msgs rcl_interfaces rclcpp_components ament_index_cpp sensor_msgs easyfly_msgs std_srvs)
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME}_backends ${OpenCV_LIBS} yaml-cpp
  easyfly_track_id_fusion::easyfly_track_id_fusion)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_multi_object_tracking::TrackingNode"
  EXECUTABLE tracking_node)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME}_backends EXPORT trackingBackends
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sort_adapter test/test_sort_adapter.cpp)
  target_link_libraries(test_sort_adapter ${PROJECT_NAME}_backends
    easyfly_sort::easyfly_sort easyfly_track_id_fusion::easyfly_track_id_fusion)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tracking_feedback_ros test/test_tracking_feedback_ros.py
    TIMEOUT 90 ENV ROS_DOMAIN_ID=213)
endif()

ament_export_targets(trackingBackends HAS_LIBRARY_TARGET)
# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package(CONFIG_EXTRAS cmake/algorithm_dependencies.cmake)
