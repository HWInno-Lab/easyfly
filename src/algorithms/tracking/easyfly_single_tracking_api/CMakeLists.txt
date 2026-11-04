# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_single_tracking_api/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.16)
project(easyfly_single_tracking_api)
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} INTERFACE ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package(CONFIG_EXTRAS cmake/dependencies.cmake)
