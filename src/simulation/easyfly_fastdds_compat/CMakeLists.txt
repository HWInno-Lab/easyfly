cmake_minimum_required(VERSION 3.16)
project(easyfly_fastdds_compat)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(fastrtps 2.6.11 EXACT REQUIRED)
include(ExternalProject)
set(EASYFLY_FASTDDS_ARCHIVE "https://codeload.github.com/eProsima/Fast-DDS/tar.gz/refs/tags/v2.6.11" CACHE STRING "Pinned archive URL or local archive for offline builds")
set(private_build "${CMAKE_CURRENT_BINARY_DIR}/fastdds-build")
set(private_library "${private_build}/src/cpp/libfastrtps.so.2.6.11")
ExternalProject_Add(fastdds_endpoint_guard
  URL "${EASYFLY_FASTDDS_ARCHIVE}"
  URL_HASH SHA256=81db506b123a6c1fcb4fc16c36173bd68132f71afa610dfb8205a3d6ec40729d
  BINARY_DIR "${private_build}"
  PATCH_COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/patch_endpoint.py" <SOURCE_DIR>
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DSECURITY=ON -DSQLITE3_SUPPORT=ON
    -DFASTDDS_STATISTICS=OFF -DCOMPILE_TOOLS=OFF -DCOMPILE_EXAMPLES=OFF
    -DBUILD_TESTING=OFF -DTHIRDPARTY=OFF
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --parallel 6
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS "${private_library}")
ExternalProject_Add_StepDependencies(fastdds_endpoint_guard patch
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/patch_endpoint.py")
ament_python_install_package(${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/private_library.dsv.in")
install(FILES "${private_library}" DESTINATION lib/${PROJECT_NAME} RENAME libfastrtps.so.2.6)
install(CODE "file(SHA256 \"${private_library}\" native_hash)\nfile(WRITE \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/SHA256\" \"\${native_hash}\\n\")")
ExternalProject_Get_Property(fastdds_endpoint_guard SOURCE_DIR)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES "${SOURCE_DIR}/LICENSE" README.md source-lock.json DESTINATION share/${PROJECT_NAME})
install(FILES cmake/patch_endpoint.py DESTINATION share/${PROJECT_NAME}/source)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_history_notification test/test_history_notification.cpp)
  target_link_libraries(test_history_notification fastrtps)
  add_dependencies(test_history_notification fastdds_endpoint_guard)
  set_tests_properties(test_history_notification PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${private_build}/src/cpp:$ENV{LD_LIBRARY_PATH}")
  # The glibc-versioned scheduling barrier is specific to our Linux x86_64
  # simulation target. It never changes the installed runtime or system mutexes.
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    add_executable(test_flow_flush test/test_flow_flush.cpp)
    target_compile_features(test_flow_flush PRIVATE cxx_std_17)
    target_link_libraries(test_flow_flush fastrtps pthread dl)
    target_link_options(test_flow_flush PRIVATE "-Wl,--export-dynamic")
    add_dependencies(test_flow_flush fastdds_endpoint_guard)
    add_test(NAME test_flow_flush COMMAND ${CMAKE_COMMAND} -E env
      "LD_LIBRARY_PATH=${private_build}/src/cpp:$ENV{LD_LIBRARY_PATH}"
      $<TARGET_FILE:test_flow_flush>)
    set_tests_properties(test_flow_flush PROPERTIES TIMEOUT 20 RUN_SERIAL TRUE)
    add_executable(test_reader_discovery test/test_reader_discovery.cpp)
    target_compile_features(test_reader_discovery PRIVATE cxx_std_17)
    target_link_libraries(test_reader_discovery fastrtps pthread dl)
    target_link_options(test_reader_discovery PRIVATE "-Wl,--export-dynamic")
    add_dependencies(test_reader_discovery fastdds_endpoint_guard)
    foreach(reliability reliable best_effort)
      foreach(event add qos remove)
        set(discovery_test "test_reader_discovery_${reliability}_${event}")
        add_test(NAME ${discovery_test} COMMAND ${CMAKE_COMMAND} -E env
          "LD_LIBRARY_PATH=${private_build}/src/cpp:$ENV{LD_LIBRARY_PATH}"
          $<TARGET_FILE:test_reader_discovery> ${reliability} ${event})
        set_tests_properties(${discovery_test} PROPERTIES TIMEOUT 15 RUN_SERIAL TRUE)
      endforeach()
    endforeach()
  endif()
  foreach(writer_test test_writer_listener test_writer_owner_lifetime)
    add_executable(${writer_test} test/${writer_test}.cpp)
    target_compile_features(${writer_test} PRIVATE cxx_std_17)
    # Exercise the private writer setter, whose inline implementation is patched.
    target_include_directories(${writer_test} BEFORE PRIVATE "${SOURCE_DIR}/include")
    target_link_libraries(${writer_test} fastrtps pthread)
    add_dependencies(${writer_test} fastdds_endpoint_guard)
    add_test(NAME ${writer_test} COMMAND ${CMAKE_COMMAND} -E env
      "LD_LIBRARY_PATH=${private_build}/src/cpp:$ENV{LD_LIBRARY_PATH}" $<TARGET_FILE:${writer_test}>)
    set_tests_properties(${writer_test} PROPERTIES TIMEOUT 15)
  endforeach()
  add_executable(test_component_profiles test/test_component_profiles.cpp)
  target_compile_features(test_component_profiles PRIVATE cxx_std_17)
  target_link_libraries(test_component_profiles fastrtps)
  add_dependencies(test_component_profiles fastdds_endpoint_guard)
  add_test(NAME test_component_profiles COMMAND ${CMAKE_COMMAND} -E env
    "LD_LIBRARY_PATH=${private_build}/src/cpp:$ENV{LD_LIBRARY_PATH}"
    $<TARGET_FILE:test_component_profiles> "${CMAKE_CURRENT_SOURCE_DIR}/config/component_services.xml")
  set_tests_properties(test_component_profiles PROPERTIES TIMEOUT 15)
  add_executable(test_endpoint_listener test/test_endpoint_listener.cpp)
  target_compile_features(test_endpoint_listener PRIVATE cxx_std_17)
  target_link_libraries(test_endpoint_listener fastrtps pthread)
  add_dependencies(test_endpoint_listener fastdds_endpoint_guard)
  add_test(NAME test_endpoint_listener COMMAND ${CMAKE_COMMAND} -E env
    "LD_LIBRARY_PATH=${private_build}/src/cpp:$ENV{LD_LIBRARY_PATH}" $<TARGET_FILE:test_endpoint_listener>)
  set_tests_properties(test_endpoint_listener PROPERTIES TIMEOUT 15)
  add_executable(test_owner_lifetime test/test_owner_lifetime.cpp)
  target_compile_features(test_owner_lifetime PRIVATE cxx_std_17)
  target_link_libraries(test_owner_lifetime fastrtps pthread)
  add_dependencies(test_owner_lifetime fastdds_endpoint_guard)
  add_test(NAME test_owner_lifetime COMMAND ${CMAKE_COMMAND} -E env
    "LD_LIBRARY_PATH=${private_build}/src/cpp:$ENV{LD_LIBRARY_PATH}" $<TARGET_FILE:test_owner_lifetime>)
  set_tests_properties(test_owner_lifetime PROPERTIES TIMEOUT 15)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime test/test_runtime.py APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()
ament_package()
