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
  include/quadrotor_msgs/comm_types.h
  95b3fa3d793b2034545553b756ccc4fc5a5e790a8618a8a2c39a26b4f272f5e4
  include/quadrotor_msgs/decode_msgs.h
  4856917c84f0a73ae5841380a9e2464ce9533fb13ab3141362d706a4b58949a8
  include/quadrotor_msgs/encode_msgs.h
  0ba07d6cbee818ed0d9c7f3ae8c9f230d9822e834c23d1c9d898dfff62c418f4
  src/decode_msgs.cpp
  470400e0dbec211363c2239aac39429b9b793536a1765e0b6755a3414df512d4
  src/encode_msgs.cpp
  aca4cb1924100ebfb5c116ed721dd425141090d9f2842040dad9ecb1e31bf687
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
