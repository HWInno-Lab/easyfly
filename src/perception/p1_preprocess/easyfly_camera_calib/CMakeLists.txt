cmake_minimum_required(VERSION 3.8)
project(easyfly_camera_calib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/online_calibration scripts/rectify_stereo DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED
  src/calib_tf_publisher.cpp
  src/camera_info_checker.cpp
)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp rclcpp_components ament_index_cpp tf2_ros geometry_msgs sensor_msgs diagnostic_msgs)
target_link_libraries(${PROJECT_NAME}_nodes yaml-cpp)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_camera_calib::CalibTfPublisher"
  EXECUTABLE calib_tf_publisher)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_camera_calib::CameraInfoChecker"
  EXECUTABLE camera_info_checker)

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_calibration test/test_calibration.cpp)
  target_include_directories(test_calibration PRIVATE include)
  target_compile_definitions(test_calibration PRIVATE CALIBRATION_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/sensors.yaml")
  target_link_libraries(test_calibration yaml-cpp)
  ament_target_dependencies(test_calibration sensor_msgs)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_solver test/test_solver.py TIMEOUT 30
    APPEND_ENV PYTHONNOUSERSITE=1)
  ament_add_pytest_test(test_calibration_ros test/test_calibration_ros.py TIMEOUT 40
    ENV ROS_DOMAIN_ID=106 ROS_LOG_DIR=/tmp/easyfly_d4_calibration)
endif()

ament_package()
