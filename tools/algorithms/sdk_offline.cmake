# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
# Verification-only project hook: reuse the recorded JSON source cache and
# disable the connectivity probe. SDK source files and targets stay unchanged.
if(NOT EXISTS "${ALG83_JSON_SOURCE}/CMakeLists.txt")
  message(FATAL_ERROR "ALG-8.3 requires the recorded local nlohmann/json source cache")
endif()
macro(include alg83_file)
  if("${alg83_file}" STREQUAL "CMake/connectivity_check.cmake")
    set(INTERNET_CONNECTION OFF)
    message(STATUS "ALG-8.3: offline verification; connectivity probe disabled")
  elseif("${alg83_file}" STREQUAL "CMake/external_json.cmake")
    # Other SDK modules also use this conventional binary-tree include path.
    file(COPY "${ALG83_JSON_SOURCE}/" DESTINATION "${CMAKE_BINARY_DIR}/third-party/json")
    add_subdirectory("${CMAKE_BINARY_DIR}/third-party/json" "${CMAKE_BINARY_DIR}/third-party/json/build")
  else()
    _include("${alg83_file}" ${ARGN})
  endif()
endmacro()
