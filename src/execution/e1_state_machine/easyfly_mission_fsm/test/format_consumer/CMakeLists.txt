cmake_minimum_required(VERSION 3.8)
project(installed_legacy_mission_consumer)
set(CMAKE_CXX_STANDARD 17)
find_package(easyfly_mission_fsm REQUIRED)
add_executable(legacy_consumer consumer.cpp)
# Exercise only the old public target; no manual include/library paths.
target_link_libraries(legacy_consumer easyfly_mission_fsm::easyfly_mission_format)
