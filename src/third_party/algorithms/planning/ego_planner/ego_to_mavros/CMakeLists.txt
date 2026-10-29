cmake_minimum_required(VERSION 3.5)
project(ego_to_mavros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  mavros_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(ego_to_mavros_node
  src/bridge.cpp
)

ament_target_dependencies(ego_to_mavros_node ${dependencies})

install(TARGETS
  ego_to_mavros_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bridge_ros test/test_bridge_ros.py TIMEOUT 70)
endif()

install(FILES ../NOTICE.md ../legacy-notice-scope.json
  DESTINATION share/${PROJECT_NAME}/source_notices)
install(DIRECTORY ../licenses DESTINATION share/${PROJECT_NAME}/source_notices)

ament_package()
