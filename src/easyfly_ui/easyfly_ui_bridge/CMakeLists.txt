cmake_minimum_required(VERSION 3.8)
project(easyfly_ui_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(easyfly_core REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(easyfly_mission_format REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(${PROJECT_NAME} SHARED src/model.cpp src/mission_details.cpp src/observer.cpp
  src/camera_observations.cpp)
# Graph evidence changes UiSnapshot's ABI; reject the previous unversioned library.
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0 SOVERSION 1)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp easyfly_core easyfly_msgs sensor_msgs rosgraph_msgs)
target_link_libraries(${PROJECT_NAME} PRIVATE JsonCpp::JsonCpp
  PUBLIC easyfly_mission_format::easyfly_mission_format)
install(TARGETS ${PROJECT_NAME} EXPORT bridgeTargets LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(bridgeTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp easyfly_core easyfly_msgs sensor_msgs rosgraph_msgs easyfly_mission_format)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(mavros_msgs REQUIRED)
  ament_add_gtest(test_model test/test_model.cpp)
  target_link_libraries(test_model ${PROJECT_NAME})
  ament_add_gtest(test_camera_observations test/test_camera_observations.cpp)
  target_link_libraries(test_camera_observations ${PROJECT_NAME})
  ament_add_gtest(test_mission_details test/test_mission_details.cpp)
  target_link_libraries(test_mission_details ${PROJECT_NAME})
  ament_add_gtest(test_observer test/test_observer.cpp TIMEOUT 90)
  target_link_libraries(test_observer ${PROJECT_NAME})
  ament_target_dependencies(test_observer mavros_msgs)
  set_tests_properties(test_observer PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=92;ROS_LOCALHOST_ONLY=1")
  foreach(setup_case missing_domain partial_context)
    add_test(NAME test_setup_${setup_case}
      COMMAND ${CMAKE_COMMAND} -DTEST_BINARY=$<TARGET_FILE:test_observer>
        -DPARTIAL_CONTEXT=$<STREQUAL:${setup_case},partial_context>
        -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_setup_failure.cmake)
    set_tests_properties(test_setup_${setup_case} PROPERTIES TIMEOUT 20)
  endforeach()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
