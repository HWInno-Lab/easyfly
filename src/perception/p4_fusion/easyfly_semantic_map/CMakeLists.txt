cmake_minimum_required(VERSION 3.8)
project(easyfly_semantic_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(easyfly_obstacle_classification)
install(PROGRAMS scripts/obstacle_classifier DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/semantic_shutdown_capture.py
  DESTINATION lib/${PROJECT_NAME} RENAME semantic_shutdown_capture)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED
  src/semantic_map_node.cpp
)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_nodes PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME}_nodes ${PCL_LIBRARIES} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp rclcpp_components ament_index_cpp sensor_msgs nav_msgs visualization_msgs
  easyfly_msgs tf2 tf2_ros tf2_eigen geometry_msgs pcl_conversions)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_semantic_map::SemanticMapNode"
  EXECUTABLE semantic_map_node)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README-ground.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  # Keep the complete cppcheck scan, with its shipped GTest macro definitions.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  ament_cppcheck(LIBRARIES googletest INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_classification test/test_classification.py)
  ament_add_pytest_test(test_observed_grid_ros test/test_observed_grid_ros.py TIMEOUT 120
    ENV ROS_DOMAIN_ID=187)
  ament_add_pytest_test(test_classification_ros test/test_classification_ros.py TIMEOUT 45
    ENV ROS_DOMAIN_ID=111 ROS_LOG_DIR=/tmp/easyfly_d4_classification)
  ament_add_pytest_test(test_shutdown_ros test/test_shutdown_ros.py TIMEOUT 100
    ENV ROS_DOMAIN_ID=186)
  ament_add_pytest_test(test_capture test/test_capture.py TIMEOUT 60)
  ament_add_pytest_test(test_shutdown_stress_ros test/test_shutdown_stress_ros.py TIMEOUT 100
    ENV ROS_DOMAIN_ID=186)
  # Both actual-node fixtures deliberately use one isolated DDS domain.
  set_tests_properties(test_shutdown_ros test_shutdown_stress_ros PROPERTIES RUN_SERIAL TRUE)
  ament_add_gtest(test_ground_plane_tracker test/test_ground_plane_tracker.cpp)
  target_include_directories(test_ground_plane_tracker PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_ground_spatial_support test/test_ground_spatial_support.cpp)
  target_include_directories(test_ground_spatial_support PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_ground_plane_transform test/test_ground_plane_transform.cpp)
  target_include_directories(test_ground_plane_transform PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
  ament_add_gtest(test_ground_candidate_selection test/test_ground_candidate_selection.cpp)
  target_include_directories(test_ground_candidate_selection PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_ground_candidate_selection ${PCL_LIBRARIES})
  ament_add_gtest(test_map_validity test/test_map_validity.cpp)
  target_include_directories(test_map_validity PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_map_validity nav_msgs easyfly_msgs)
  ament_add_gtest(test_grid_inflation test/test_grid_inflation.cpp)
  target_include_directories(test_grid_inflation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_observed_grid test/test_observed_grid.cpp)
  target_include_directories(test_observed_grid PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_dynamic_obstacle_tracker test/test_dynamic_obstacle_tracker.cpp)
  target_include_directories(test_dynamic_obstacle_tracker PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
