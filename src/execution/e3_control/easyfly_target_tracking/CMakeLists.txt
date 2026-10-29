cmake_minimum_required(VERSION 3.8)
project(easyfly_target_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED
  src/target_tracking_node.cpp
)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp rclcpp_components rclcpp_action ament_index_cpp easyfly_msgs)
target_link_libraries(${PROJECT_NAME}_nodes yaml-cpp)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_target_tracking::TargetTrackingNode"
  EXECUTABLE target_tracking_node)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Preserve native analysis; teach cppcheck the C++ and test-framework syntax.
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(LANGUAGE c++ LIBRARIES googletest qt
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_lifecycle test/test_waypoint_lifecycle.cpp)
  target_include_directories(test_waypoint_lifecycle PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_target_frame test/test_target_frame.cpp)
  target_include_directories(test_target_frame PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_target_frame easyfly_msgs)
  ament_add_gtest(test_follow_offset test/test_follow_offset.cpp)
  target_include_directories(test_follow_offset PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_export_include_directories(include)
ament_package()
