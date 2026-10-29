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
  include/path_searching/dyn_a_star.h
  3e7d5420f458bd56f55b2f71356ca4677903e2a2307c858502bd74d8fbe185e5
  src/dyn_a_star.cpp
  59a7f4108431839b8753b17f9710e368e292c7cae78634f0d1d7bdfdf2aa4574
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
