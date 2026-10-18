cmake_minimum_required(VERSION 3.16)
project(planning_consumer LANGUAGES CXX)
find_package(easyfly_astar REQUIRED)
find_package(easyfly_coverage_planning REQUIRED)
find_package(easyfly_global_planner REQUIRED)
add_executable(planning_consumer main.cpp)
target_link_libraries(planning_consumer PRIVATE easyfly_astar::easyfly_astar
  easyfly_coverage_planning::easyfly_coverage_planning
  easyfly_global_planner::easyfly_global_planner_backends)
target_compile_options(planning_consumer PRIVATE -Wall -Wextra -Wpedantic -Werror)
enable_testing()
add_test(NAME installed_planning_consumer COMMAND planning_consumer)
