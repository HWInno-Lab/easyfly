# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_pybind/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_pybind)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(easyfly_core REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
set(CMAKE_CXX_STANDARD 17)
pybind11_add_module(easyfly_pybind src/bindings.cpp)
target_link_libraries(easyfly_pybind PRIVATE easyfly_core::easyfly_core)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS easyfly_pybind LIBRARY DESTINATION ${PYTHON_INSTALL_DIR})
ament_environment_hooks(env-hooks/pythonpath.dsv.in)
install(PROGRAMS examples/state.py DESTINATION lib/${PROJECT_NAME} RENAME sdk_state_example)

if(BUILD_TESTING)
  add_test(NAME test_sdk_python COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sdk_python.py)
  set_tests_properties(test_sdk_python PROPERTIES TIMEOUT 50)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
