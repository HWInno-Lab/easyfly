cmake_minimum_required(VERSION 3.8)
project(easyfly_global_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(easyfly_core REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(easyfly_astar REQUIRED)
find_package(easyfly_grid_planner_api REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_library(${PROJECT_NAME}_backends SHARED src/adapters/astar_adapter.cpp)
target_include_directories(${PROJECT_NAME}_backends PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_backends PUBLIC
  easyfly_grid_planner_api::easyfly_grid_planner_api easyfly_astar::easyfly_astar)

add_library(${PROJECT_NAME}_nodes SHARED
  src/global_planner_node.cpp
  src/navigation_coordinator_node.cpp
)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp rcl_interfaces rclcpp_components rclcpp_action ament_index_cpp nav_msgs geometry_msgs std_msgs std_srvs easyfly_msgs)
target_link_libraries(${PROJECT_NAME}_nodes yaml-cpp easyfly_core::easyfly_core ${PROJECT_NAME}_backends)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_global_planner::GlobalPlannerNode"
  EXECUTABLE global_planner_node)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_global_planner::NavigationCoordinatorNode"
  EXECUTABLE navigation_coordinator_node)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME}_backends EXPORT plannerBackends
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_coordinator test/test_coordinator.py
    TIMEOUT 45 ENV ROS_LOG_DIR=/tmp/easyfly_d1_coordinator_test ROS_DOMAIN_ID=83)
  ament_add_pytest_test(test_local_coordinator test/test_local_coordinator.py
    TIMEOUT 90 ENV ROS_LOG_DIR=/tmp/easyfly_d1_local_coordinator_test ROS_DOMAIN_ID=84)
  ament_add_pytest_test(test_late_session test/test_late_session.py
    TIMEOUT 45 ENV ROS_LOG_DIR=/tmp/easyfly_d1_late_session_test ROS_DOMAIN_ID=86)
  ament_add_pytest_test(test_navigation_chain test/test_navigation_chain.py
    TIMEOUT 180 ENV ROS_LOG_DIR=/tmp/easyfly_d1_navigation_chain_test ROS_DOMAIN_ID=87)
  ament_add_pytest_test(test_astar_deadline test/test_astar_deadline.py
    TIMEOUT 30 ENV ROS_LOG_DIR=/tmp/easyfly_d1_astar_deadline_test ROS_DOMAIN_ID=88)
  ament_add_pytest_test(test_astar_endpoint test/test_astar_endpoint.py
    TIMEOUT 45 ENV ROS_LOG_DIR=/tmp/easyfly_astar_endpoint_test ROS_DOMAIN_ID=89)
  ament_add_gtest(test_map_validity test/test_map_validity.cpp)
  target_include_directories(test_map_validity PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_map_validity nav_msgs)
  ament_add_gtest(test_waypoint_lifecycle test/test_waypoint_lifecycle.cpp)
  target_include_directories(test_waypoint_lifecycle PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_goal_validation test/test_goal_validation.cpp)
  target_include_directories(test_goal_validation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_goal_validation easyfly_msgs)
  ament_add_gtest(test_segment_acceptance test/test_segment_acceptance.cpp)
  target_include_directories(test_segment_acceptance PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_export_targets(plannerBackends HAS_LIBRARY_TARGET)
ament_package(CONFIG_EXTRAS cmake/algorithm_dependencies.cmake)
