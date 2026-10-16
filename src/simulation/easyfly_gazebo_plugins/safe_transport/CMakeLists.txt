# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0.
find_package(gazebo 11.10.2 EXACT REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(TBB REQUIRED)
find_file(EASYFLY_IO_HEADER gazebo/transport/IOManager.hh
  PATHS ${GAZEBO_INCLUDE_DIRS} NO_DEFAULT_PATH REQUIRED)
file(SHA256 "${EASYFLY_IO_HEADER}" EASYFLY_IO_HEADER_SHA)
if(NOT EASYFLY_IO_HEADER_SHA STREQUAL
    "81584e70ee4e7042dc75fbf9fe703ba6f21df7da20ed9873a8305e3255709b63")
  message(FATAL_ERROR "Unsupported Gazebo IOManager header; no unverified fallback")
endif()

# A complete private transport build is required: the installed library binds
# internal IOManager calls directly, so symbol interposition cannot fix them.
find_library(EASYFLY_SYSTEM_TRANSPORT gazebo_transport
  PATHS ${GAZEBO_LIBRARY_DIRS} NO_DEFAULT_PATH REQUIRED)
file(SHA256 "${EASYFLY_SYSTEM_TRANSPORT}" EASYFLY_SYSTEM_TRANSPORT_SHA)
if(NOT EASYFLY_SYSTEM_TRANSPORT_SHA STREQUAL
    "fd084b63c69f7b0157a8ec99e423d4cfd3f7005b89af67fd2f1ac88ac14ed90d")
  message(FATAL_ERROR "Unsupported installed transport build; review required")
endif()
get_filename_component(EASYFLY_TRANSPORT_HEADERS "${EASYFLY_IO_HEADER}" DIRECTORY)
add_library(easyfly_transport_compat SHARED src/io_manager.cpp
  upstream/CallbackHelper.cc src/connection.cpp src/connection_manager.cpp
  upstream/Node.cc upstream/Publication.cc upstream/PublicationTransport.cc
  upstream/Publisher.cc upstream/Subscriber.cc upstream/SubscriptionTransport.cc
  upstream/TopicManager.cc upstream/TransportIface.cc)
target_compile_features(easyfly_transport_compat PUBLIC cxx_std_17)
target_include_directories(easyfly_transport_compat PUBLIC include ${GAZEBO_INCLUDE_DIRS}
  PRIVATE "${EASYFLY_TRANSPORT_HEADERS}")
target_compile_definitions(easyfly_transport_compat PRIVATE
  $<$<CONFIG:Debug>:EASYFLY_TRANSPORT_DIAGNOSTICS>
  BUILDING_DLL_GZ_TRANSPORT GAZEBO_DEFAULT_MASTER_HOST="localhost"
  GAZEBO_DEFAULT_MASTER_PORT=11345)
target_link_libraries(easyfly_transport_compat PUBLIC
  gazebo_common gazebo_msgs ${Boost_LIBRARIES} ${IGNITION-MATH_LIBRARIES}
  OpenSSL::Crypto Threads::Threads TBB::tbb ${CMAKE_DL_LIBS})
set_target_properties(easyfly_transport_compat PROPERTIES
  OUTPUT_NAME gazebo_transport VERSION 11.10.2 SOVERSION 11
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/transport")

set(EASYFLY_SERVER_LIBRARIES ${GAZEBO_LIBRARIES})
list(FILTER EASYFLY_SERVER_LIBRARIES EXCLUDE REGEX "libgazebo_transport\\.so")

add_executable(easyfly_gzserver src/server_main.cpp)
target_link_libraries(easyfly_gzserver PRIVATE easyfly_transport_compat
  ${EASYFLY_SERVER_LIBRARIES})
set_target_properties(easyfly_gzserver PROPERTIES INSTALL_RPATH "$ORIGIN/transport")
install(TARGETS easyfly_gzserver RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS easyfly_transport_compat LIBRARY DESTINATION lib/${PROJECT_NAME}/transport)
install(FILES README.md ../safe_camera/LICENSE
  DESTINATION share/${PROJECT_NAME}/safe_transport)
install(FILES upstream/SHA256SUMS DESTINATION share/${PROJECT_NAME}/safe_transport)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_transport_patch_contract
    test/test_patch_contract.py TIMEOUT 15)
  ament_add_gtest(test_transport_lifetime test/test_transport_lifetime.cpp TIMEOUT 30)
  target_link_libraries(test_transport_lifetime easyfly_transport_compat)
  target_link_libraries(test_transport_lifetime TBB::tbb)
  ament_add_gtest(test_transport_finish test/test_transport_finish.cpp TIMEOUT 15)
  target_link_libraries(test_transport_finish easyfly_transport_compat TBB::tbb)
  ament_add_gtest(test_transport_pending_read test/test_pending_read.cpp TIMEOUT 20)
  target_link_libraries(test_transport_pending_read easyfly_transport_compat ${EASYFLY_SERVER_LIBRARIES})
  ament_add_gtest(test_transport_orphan_read test/test_orphan_read.cpp TIMEOUT 20)
  target_link_libraries(test_transport_orphan_read easyfly_transport_compat ${EASYFLY_SERVER_LIBRARIES})
  ament_add_gtest(test_transport_task_retirement test/test_task_retirement.cpp TIMEOUT 20)
  target_link_libraries(test_transport_task_retirement easyfly_transport_compat ${EASYFLY_SERVER_LIBRARIES})
endif()
