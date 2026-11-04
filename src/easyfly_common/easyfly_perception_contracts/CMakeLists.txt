# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_perception_contracts/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.16)
project(easyfly_perception_contracts)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
pybind11_add_module(_intake src/intake.cpp)
target_compile_features(_intake PRIVATE cxx_std_17)
target_include_directories(_intake PRIVATE include)
ament_target_dependencies(_intake PUBLIC rclcpp easyfly_msgs sensor_msgs diagnostic_msgs)
install(TARGETS _intake LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp easyfly_msgs diagnostic_msgs)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_contracts test/test_contracts.cpp)
  target_compile_features(test_contracts PRIVATE cxx_std_17)
  target_include_directories(test_contracts PRIVATE include)
  ament_target_dependencies(test_contracts rclcpp easyfly_msgs)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_contracts_python test/test_contracts.py)
endif()
install(FILES README.md DESTINATION share/${PROJECT_NAME})
# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
