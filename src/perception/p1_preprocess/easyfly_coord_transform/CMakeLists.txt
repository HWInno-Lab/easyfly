cmake_minimum_required(VERSION 3.8)
project(easyfly_coord_transform)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(easyfly_msgs REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED
  src/odom_tf_publisher.cpp
)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_nodes PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp rclcpp_components tf2_ros geometry_msgs nav_msgs easyfly_msgs)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_coord_transform::OdomTfPublisher"
  EXECUTABLE odom_tf_publisher)

# header-only 工具库 (coord_utils.hpp), 供 M2/M5 等下游包使用
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_odom_tf_ros test/test_odom_tf_ros.py TIMEOUT 70)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp tf2_ros)
ament_export_include_directories(include)

ament_package()
