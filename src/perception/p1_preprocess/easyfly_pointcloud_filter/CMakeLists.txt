cmake_minimum_required(VERSION 3.8)
project(easyfly_pointcloud_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 节点 (组件库): 滤波核心为 header-only (filter_core.hpp), 无 PCL 依赖
add_library(${PROJECT_NAME}_nodes SHARED
  src/pointcloud_filter_node.cpp
)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp rclcpp_components ament_index_cpp sensor_msgs
  livox_ros_driver2 tf2 tf2_ros tf2_eigen geometry_msgs)
target_link_libraries(${PROJECT_NAME}_nodes Eigen3::Eigen yaml-cpp)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_pointcloud_filter::PointcloudFilterNode"
  EXECUTABLE pointcloud_filter_node)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_filter_core test/test_filter_core.cpp)
  target_include_directories(test_filter_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_transform_timing test/test_transform_timing.py
    ENV "POINTCLOUD_FILTER_EXECUTABLE=$<TARGET_FILE:pointcloud_filter_node>"
    TIMEOUT 45)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs livox_ros_driver2)

ament_package()
