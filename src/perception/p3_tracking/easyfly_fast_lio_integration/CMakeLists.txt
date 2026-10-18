cmake_minimum_required(VERSION 3.8)
project(easyfly_fast_lio_integration)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED
  src/map_odom_tf.cpp
)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp rclcpp_components nav_msgs tf2 tf2_ros geometry_msgs)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_fast_lio_integration::MapOdomTf"
  EXECUTABLE map_odom_tf)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_map_odom_tf_ros test/test_map_odom_tf_ros.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=222)
  add_test(
    NAME easyfly_fast_lio_launch_contract
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_fastlio_launch_contract.py)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
