cmake_minimum_required(VERSION 3.5)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils_so REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  traj_utils
  traj_utils_so
  cv_bridge
  nav_msgs
  visualization_msgs
  Eigen3
  PCL
  message_filters
  pcl_msgs
  bspline_opt
  plan_env
  path_searching
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${traj_utils_so_INCLUDE_DIRS}
)

add_executable(ego_planner_node
  src/ego_planner_node.cpp
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)

ament_target_dependencies(ego_planner_node ${dependencies})
ament_target_dependencies(ego_planner_node easyfly_msgs)

target_link_libraries(ego_planner_node
  ${PCL_LIBRARIES}
)

add_executable(traj_server src/traj_server.cpp)

# traj_server 只做 B-spline 采样和 PositionCommand 发布。不要继承完整规划器的
# PCL/cv_bridge/plan_env 依赖，否则 ROS interface 的传递库会被重复展开数千次。
ament_target_dependencies(traj_server
  easyfly_msgs
  rclcpp
  traj_utils
  quadrotor_msgs
  bspline_opt
)

install(TARGETS
  traj_server
  ego_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_ego_signal_shutdown_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ego_signal_shutdown.py)
  set_tests_properties(test_ego_signal_shutdown_ros PROPERTIES TIMEOUT 60 LABELS "ros_network"
    RUN_SERIAL TRUE)
  add_test(NAME test_traj_shutdown_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_traj_shutdown.py)
  set_tests_properties(test_traj_shutdown_ros PROPERTIES TIMEOUT 45 LABELS "ros_network"
    ENVIRONMENT "ROS_DOMAIN_ID=193")
  add_test(NAME test_initial_sampling
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_initial_sampling.py)
  add_test(NAME test_navigation_session_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_session.py)
  set_tests_properties(test_navigation_session_ros PROPERTIES TIMEOUT 20 LABELS "ros_network")
  add_test(NAME test_managed_navigation_fault_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_managed_navigation_fault.py)
  set_tests_properties(test_managed_navigation_fault_ros PROPERTIES TIMEOUT 50 LABELS "ros_network")
  add_test(NAME test_managed_refinement_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_managed_refinement.py)
  set_tests_properties(test_managed_refinement_ros PROPERTIES TIMEOUT 35 LABELS "ros_network")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_emergency_stop_gate
    test/test_emergency_stop_gate.cpp)
  target_include_directories(test_emergency_stop_gate PRIVATE include)

  ament_add_gtest(test_swarm_bspline_validator
    test/test_swarm_bspline_validator.cpp)
  target_include_directories(test_swarm_bspline_validator PRIVATE include)
  ament_target_dependencies(test_swarm_bspline_validator bspline_opt)
  target_link_libraries(test_swarm_bspline_validator ${bspline_opt_LIBRARIES})

  ament_add_gtest(test_waypoint_admission
    test/test_waypoint_admission.cpp)
  target_include_directories(test_waypoint_admission PRIVATE include)

  ament_add_gtest(test_trajectory_seed_safety
    test/test_trajectory_seed_safety.cpp)
  target_include_directories(test_trajectory_seed_safety PRIVATE include)
  ament_target_dependencies(test_trajectory_seed_safety plan_env)
endif()

ament_package()
