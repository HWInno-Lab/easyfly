cmake_minimum_required(VERSION 3.16)
project(prediction_filter_consumer LANGUAGES CXX)
find_package(easyfly_motion_prediction REQUIRED)
find_package(easyfly_pointcloud_filters REQUIRED)
option(EASYFLY_CHECK_LEGACY "Compile installed legacy forwarding headers" OFF)
add_executable(prediction_filter_consumer main.cpp)
target_link_libraries(prediction_filter_consumer PRIVATE
  easyfly_motion_prediction::easyfly_motion_prediction
  easyfly_pointcloud_filters::easyfly_pointcloud_filters)
if(EASYFLY_CHECK_LEGACY)
  # Legacy ROS package metadata still exports its original ROS dependencies.
  find_package(easyfly_target_prediction REQUIRED)
  find_package(easyfly_pointcloud_filter REQUIRED)
  target_compile_definitions(prediction_filter_consumer PRIVATE EASYFLY_CHECK_LEGACY)
  target_include_directories(prediction_filter_consumer PRIVATE
    ${easyfly_target_prediction_INCLUDE_DIRS} ${easyfly_pointcloud_filter_INCLUDE_DIRS})
endif()
target_compile_options(prediction_filter_consumer PRIVATE -Wall -Wextra -Wpedantic -Werror)
enable_testing()
add_test(NAME installed_prediction_filter_consumer COMMAND prediction_filter_consumer)
