cmake_minimum_required(VERSION 3.19)
project(easyfly_mavros_runtime)
set(CMAKE_CXX_STANDARD 20)
find_package(ament_cmake REQUIRED)
find_package(mavros 2.14.0 EXACT REQUIRED)
foreach(dep rclcpp rclcpp_components pluginlib rcpputils libmavconn mavlink
    tf2_ros tf2_eigen diagnostic_updater mavros_msgs sensor_msgs diagnostic_msgs)
  find_package(${dep} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Threads REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/upstream/ORIGIN.json" origin)
string(JSON source_count LENGTH "${origin}" source_sha256)
math(EXPR source_last "${source_count} - 1")
foreach(index RANGE ${source_last})
  string(JSON source MEMBER "${origin}" source_sha256 ${index})
  string(JSON expected_source GET "${origin}" source_sha256 "${source}")
  file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/upstream/${source}" actual_source)
  if(NOT actual_source STREQUAL expected_source)
    message(FATAL_ERROR "MAVROS upstream source fingerprint mismatch: ${source}")
  endif()
endforeach()
string(JSON expected GET "${origin}" installed_library_sha256)
find_library(system_mavros mavros REQUIRED)
file(SHA256 "${system_mavros}" actual)
if(NOT actual STREQUAL expected)
  message(FATAL_ERROR "MAVROS plugin ABI differs from reviewed 2.14.0 library")
endif()
string(JSON expected_mavconn GET "${origin}" installed_mavconn_library_sha256)
find_library(system_mavconn mavconn REQUIRED)
file(SHA256 "${system_mavconn}" actual_mavconn)
if(NOT actual_mavconn STREQUAL expected_mavconn)
  message(FATAL_ERROR "MAVConn ABI differs from reviewed 2.14.0 library")
endif()
include(em_expand)
foreach(generated include/mavconn/mavlink_dialect.hpp src/mavlink_helpers.cpp)
  em_expand(${CMAKE_CURRENT_SOURCE_DIR}/upstream/libmavconn/mavlink.context.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/mavlink.context.py
    ${CMAKE_CURRENT_SOURCE_DIR}/upstream/libmavconn/${generated}.em
    ${CMAKE_CURRENT_BINARY_DIR}/generated/${generated})
endforeach()
add_library(owned_mavconn SHARED
  ${CMAKE_CURRENT_BINARY_DIR}/generated/src/mavlink_helpers.cpp
  upstream/libmavconn/src/interface.cpp src/mavconn_serial.cpp
  upstream/libmavconn/src/tcp.cpp upstream/libmavconn/src/udp.cpp)
target_include_directories(owned_mavconn PUBLIC upstream/libmavconn/include
  ${CMAKE_CURRENT_BINARY_DIR}/generated/include ${mavlink_INCLUDE_DIRS})
target_link_libraries(owned_mavconn PUBLIC console_bridge::console_bridge Threads::Threads)
set_target_properties(owned_mavconn PROPERTIES OUTPUT_NAME mavconn
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/runtime")
set(library_sources enum_sensor_orientation enum_to_string ftf_frame_conversions
  ftf_quaternion_utils plugin uas_ap uas_data uas_executor uas_stringify uas_tf uas_timesync)
list(TRANSFORM library_sources PREPEND "upstream/mavros/src/lib/")
list(TRANSFORM library_sources APPEND ".cpp")
add_library(owned_mavros SHARED ${library_sources} src/mavros_router.cpp src/mavros_uas.cpp)
target_include_directories(owned_mavros PUBLIC include upstream/mavros/include
  ${mavlink_INCLUDE_DIRS} ${libmavconn_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS})
target_compile_definitions(owned_mavros PUBLIC USE_OLD_RMW_QOS USE_OLD_TF2_ROS
  USE_OLD_RMW_QOS_MESSAGE_FILTERS)
target_link_libraries(owned_mavros PUBLIC Eigen3::Eigen ${mavros_msgs_TARGETS}
  ${sensor_msgs_TARGETS} ${diagnostic_msgs_TARGETS} diagnostic_updater::diagnostic_updater
  pluginlib::pluginlib rclcpp::rclcpp rclcpp_components::component
  rclcpp_components::component_manager rcpputils::rcpputils sensor_msgs::sensor_msgs_library
  tf2_eigen::tf2_eigen tf2_ros::static_transform_broadcaster_node tf2_ros::tf2_ros
  owned_mavconn ${GeographicLib_LIBRARIES})
set_target_properties(owned_mavros PROPERTIES OUTPUT_NAME mavros
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/runtime")
add_executable(mavros_node src/mavros_node.cpp)
target_link_libraries(mavros_node PRIVATE owned_mavros)
target_link_options(mavros_node PRIVATE "-Wl,--disable-new-dtags")
set_target_properties(mavros_node PROPERTIES INSTALL_RPATH "$ORIGIN/runtime"
  BUILD_WITH_INSTALL_RPATH TRUE)
install(TARGETS mavros_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS owned_mavros LIBRARY DESTINATION lib/${PROJECT_NAME}/runtime)
install(TARGETS owned_mavconn LIBRARY DESTINATION lib/${PROJECT_NAME}/runtime)
install(FILES README.md upstream/ORIGIN.json upstream/LICENSE.md upstream/LICENSE-BSD.txt
  DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_shutdown test/test_runtime_shutdown.py TIMEOUT 120
    ENV "RUNTIME_EXECUTABLE=$<TARGET_FILE:mavros_node>")
endif()
ament_package()
