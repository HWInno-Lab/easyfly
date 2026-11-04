# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_serial/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_serial)

# 默认 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# ============================================================
# 静态库 libeasyfly_serial — 零 ROS2 依赖
# ============================================================
add_library(easyfly_serial STATIC
  src/serial_port.cpp
)

target_include_directories(easyfly_serial
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ============================================================
# 安装
# ============================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS easyfly_serial
  ARCHIVE DESTINATION lib
)

# ============================================================
# 测试（阶段一：自发自收验证）
# ============================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  add_executable(test_loopback test/test_loopback.cpp)
  target_link_libraries(test_loopback easyfly_serial)
  target_include_directories(test_loopback PRIVATE include)

  ament_add_test(
    test_loopback
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:test_loopback>"
  )
endif()

# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
