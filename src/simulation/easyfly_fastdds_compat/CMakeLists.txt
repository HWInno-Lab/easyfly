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
install(FILES "${private_library}" DESTINATION lib/${PROJECT_NAME} RENAME libfastrtps.so.2.6)
install(CODE "file(SHA256 \"${private_library}\" native_hash)\nfile(WRITE \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/SHA256\" \"\${native_hash}\\n\")")
ExternalProject_Get_Property(fastdds_endpoint_guard SOURCE_DIR)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES "${SOURCE_DIR}/LICENSE" README.md source-lock.json DESTINATION share/${PROJECT_NAME})
install(FILES cmake/patch_endpoint.py DESTINATION share/${PROJECT_NAME}/source)
if(BUILD_TESTING)
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
