cmake_minimum_required(VERSION 3.16)
project(easyfly_algorithm_api_consumer LANGUAGES CXX)

# This is deliberately a plain CMake project, with no ament or ROS runtime.
find_package(easyfly_tracking_api REQUIRED)
find_package(easyfly_grid_planner_api REQUIRED)

foreach(api easyfly_tracking_api easyfly_grid_planner_api)
  get_target_property(features ${api}::${api} INTERFACE_COMPILE_FEATURES)
  if(NOT "cxx_std_17" IN_LIST features)
    message(FATAL_ERROR "${api} does not export its C++17 requirement")
  endif()
endforeach()

add_executable(api_consumer main.cpp)
target_link_libraries(api_consumer PRIVATE
  easyfly_tracking_api::easyfly_tracking_api
  easyfly_grid_planner_api::easyfly_grid_planner_api)
target_compile_options(api_consumer PRIVATE -Wall -Wextra -Wpedantic -Werror)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/interface-includes.txt" CONTENT
  "tracking=$<TARGET_PROPERTY:easyfly_tracking_api::easyfly_tracking_api,INTERFACE_INCLUDE_DIRECTORIES>\nplanning=$<TARGET_PROPERTY:easyfly_grid_planner_api::easyfly_grid_planner_api,INTERFACE_INCLUDE_DIRECTORIES>\n")

enable_testing()
add_test(NAME installed_tracking_substitution COMMAND api_consumer tracking)
add_test(NAME installed_planning_substitution COMMAND api_consumer planning)
add_test(NAME installed_search_controls COMMAND api_consumer controls)
