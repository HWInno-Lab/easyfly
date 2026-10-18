cmake_minimum_required(VERSION 3.8)
project(easyfly_mission_fsm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(easyfly_core REQUIRED)
find_package(easyfly_mission_format REQUIRED)
find_package(easyfly_coverage_planning REQUIRED)

add_executable(mission_fsm_node
  src/mission_fsm_node.cpp
  src/mission_fsm.cpp
)

target_include_directories(mission_fsm_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(mission_fsm_node
  rclcpp
  rclcpp_action
  easyfly_msgs
)
target_link_libraries(mission_fsm_node easyfly_mission_format::easyfly_mission_format easyfly_core::easyfly_core)

add_executable(search_area_node src/search_area_node.cpp)
ament_target_dependencies(search_area_node rclcpp rclcpp_action easyfly_msgs)
target_include_directories(search_area_node PRIVATE include)
target_link_libraries(search_area_node easyfly_core::easyfly_core easyfly_coverage_planning::easyfly_coverage_planning)
add_executable(mission_validate src/mission_validate.cpp)
target_include_directories(mission_validate PRIVATE include)
target_link_libraries(mission_validate easyfly_mission_format::easyfly_mission_format)
install(TARGETS search_area_node mission_validate DESTINATION lib/${PROJECT_NAME})

install(TARGETS mission_fsm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY missions launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(easyfly_mission_format easyfly_coverage_planning)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_test(NAME test_installed_format_compat
    COMMAND ${CMAKE_COMMAND}
      -DCONSUMER_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/test/format_consumer
      -DCONSUMER_BUILD=${CMAKE_CURRENT_BINARY_DIR}/format_consumer
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_format_compat.cmake)
  set_tests_properties(test_installed_format_compat PROPERTIES TIMEOUT 30)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_prevalidation test/test_prevalidation.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=96 ROS_LOG_DIR=/tmp/easyfly_d2_prevalidation)
  ament_add_pytest_test(test_mission_executor test/test_mission_executor.py TIMEOUT 90
    ENV ROS_DOMAIN_ID=101 ROS_LOG_DIR=/tmp/easyfly_d3_executor)
  ament_add_pytest_test(test_search_action test/test_search_action.py TIMEOUT 90
    ENV ROS_DOMAIN_ID=102 ROS_LOG_DIR=/tmp/easyfly_d3_search)
endif()

ament_package(CONFIG_EXTRAS "cmake/mission_format_compat.cmake")
