# Copyright 2026 EasyFly Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Exact inherited-file scope; see ../NOTICE.md and ../legacy-notice-scope.json.
# A content change requires an explicit renewed provenance review.
set(_legacy_notice_records
  include/traj_utils/plan_container.hpp
  ed7c2c92ed2b144b0f2f0246244c9a00b912dd8c3fc8bd8371d7452205ed17aa
  include/traj_utils/planning_visualization.h
  7ac48b0fa7149081364d910174a1d413539b1d54b4f1f04c13af48038a711896
  include/traj_utils/polynomial_traj.h
  3bbd62ca4bc703bd886a90801fcb6dd50731c6d2eecf23097acdba16fad5ae78
  src/planning_visualization.cpp
  56a1c905724e8b4e86819491d7981163b411e7a0a4468da2e840009a28c50e80
  src/polynomial_traj.cpp
  c5b9edc791f8abd4a73d62f28dffb2650ebb0ee1c5a8568b6706f94e5e59cccc
)
set(_legacy_notice_files "")
set(_legacy_cpp_files "")
list(LENGTH _legacy_notice_records _record_count)
math(EXPR _last_record "${_record_count} - 1")
foreach(_index RANGE 0 ${_last_record} 2)
  math(EXPR _hash_index "${_index} + 1")
  list(GET _legacy_notice_records ${_index} _relative)
  list(GET _legacy_notice_records ${_hash_index} _expected)
  set(_path "${CMAKE_CURRENT_SOURCE_DIR}/${_relative}")
  file(SHA256 "${_path}" _actual)
  if(NOT _actual STREQUAL _expected)
    message(FATAL_ERROR "Renew the inherited copyright review for ${_relative}")
  endif()
  list(APPEND _legacy_notice_files "${_path}")
  if(_relative MATCHES "\\.(h|hpp|cpp)$")
    list(APPEND _legacy_cpp_files "${_path}")
  endif()
endforeach()
find_package(ament_cmake_copyright REQUIRED)
find_package(ament_cmake_cpplint REQUIRED)
ament_copyright(EXCLUDE ${_legacy_notice_files} ${AMENT_LINT_AUTO_FILE_EXCLUDE})
file(GLOB_RECURSE _all_cpp CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.c" "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hh"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.hxx")
list(REMOVE_ITEM _all_cpp ${_legacy_cpp_files})
# All other cpplint rules still run on these exact inherited C++ files.
if(_all_cpp)
  ament_cpplint(EXCLUDE "${_legacy_cpp_files}")
  ament_cpplint(${_legacy_cpp_files}
    TESTNAME cpplint_legacy_notices FILTERS -legal/copyright)
else()
  ament_cpplint(${_legacy_cpp_files} FILTERS -legal/copyright)
endif()
list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint)
