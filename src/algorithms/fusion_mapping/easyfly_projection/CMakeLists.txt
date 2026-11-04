# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_projection/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.16)
project(easyfly_projection)
find_package(ament_cmake REQUIRED)
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_projection test/test_projection.cpp)
  target_link_libraries(test_projection ${PROJECT_NAME})
  target_compile_definitions(test_projection PRIVATE
    EASYFLY_PROJECTION_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()
install(FILES README.md DESTINATION share/${PROJECT_NAME})
# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
