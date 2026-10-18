cmake_minimum_required(VERSION 3.8)
project(easyfly_pixel_to_3d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(easyfly_projection REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED
  src/pixel_to_3d_node.cpp
)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_nodes PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp rclcpp_components ament_index_cpp
  sensor_msgs std_msgs visualization_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  easyfly_msgs)
target_link_libraries(${PROJECT_NAME}_nodes yaml-cpp easyfly_projection::easyfly_projection)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_pixel_to_3d::PixelTo3dNode"
  EXECUTABLE pixel_to_3d_node)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_contract test/test_frame_contract.cpp)
  target_include_directories(test_frame_contract PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_frame_contract
    std_msgs easyfly_msgs)
endif()

ament_package()
