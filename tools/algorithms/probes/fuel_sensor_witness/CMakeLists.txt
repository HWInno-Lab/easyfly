# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(fuel_sensor_witness LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(easyfly_gazebo_plugins REQUIRED)
get_filename_component(EASYFLY_PLUGIN_PREFIX "${easyfly_gazebo_plugins_DIR}/../../.." ABSOLUTE)
find_file(WITNESS_SOURCE sensor_witness.cpp
  PATHS "${EASYFLY_PLUGIN_PREFIX}/share/easyfly_gazebo_plugins/safe_transport/src"
  NO_DEFAULT_PATH REQUIRED)
find_path(WITNESS_INCLUDE easyfly_transport/compat.hpp
  PATHS "${EASYFLY_PLUGIN_PREFIX}/include" NO_DEFAULT_PATH REQUIRED)
find_library(WITNESS_TRANSPORT NAMES gazebo_transport libgazebo_transport.so.11.10.2
  PATHS "${EASYFLY_PLUGIN_PREFIX}/lib/easyfly_gazebo_plugins/transport"
  NO_DEFAULT_PATH REQUIRED)
# ament symlink-install may install only the versioned link. Resolve that
# installed link so the runtime SONAME lookup uses the same verified image.
get_filename_component(WITNESS_TRANSPORT_REAL "${WITNESS_TRANSPORT}" REALPATH)
get_filename_component(WITNESS_TRANSPORT_DIRECTORY "${WITNESS_TRANSPORT_REAL}" DIRECTORY)
add_executable(fuel_sensor_witness "${WITNESS_SOURCE}")
target_compile_features(fuel_sensor_witness PRIVATE cxx_std_17)
target_include_directories(fuel_sensor_witness PRIVATE "${WITNESS_INCLUDE}")
# Keep the verified private SONAME before gazebo_dev's transitive transport.
target_link_libraries(fuel_sensor_witness "${WITNESS_TRANSPORT}")
ament_target_dependencies(fuel_sensor_witness gazebo_dev)
set_target_properties(fuel_sensor_witness PROPERTIES
  BUILD_RPATH "${WITNESS_TRANSPORT_DIRECTORY}")
