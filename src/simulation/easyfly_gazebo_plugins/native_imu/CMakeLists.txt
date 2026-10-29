# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
find_package(gazebo 11.10.2 EXACT REQUIRED)
find_package(Eigen3 REQUIRED)
find_path(EASYFLY_GAZEBO_PROTO_INCLUDE quaternion.pb.h HINTS ${GAZEBO_INCLUDE_DIRS} PATH_SUFFIXES gazebo/msgs REQUIRED)
set(EASYFLY_PX4_GAZEBO_BUILD "" CACHE PATH "Reviewed PX4 Classic build containing Imu.pb.h and libsensor_msgs.so")
find_path(EASYFLY_IMU_PROTO_INCLUDE Imu.pb.h HINTS "${EASYFLY_PX4_GAZEBO_BUILD}" NO_DEFAULT_PATH REQUIRED)
find_library(EASYFLY_IMU_PROTO_LIBRARY sensor_msgs HINTS "${EASYFLY_PX4_GAZEBO_BUILD}" NO_DEFAULT_PATH REQUIRED)
file(SHA256 "${EASYFLY_IMU_PROTO_LIBRARY}" EASYFLY_IMU_PROTO_SHA)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/native_imu.json"
  "{\"schema\":1,\"sensor_msgs_sha256\":\"${EASYFLY_IMU_PROTO_SHA}\",\"revision\":\"70683dc759cbd3ebccfa78429b564f8b1fb5d149\"}\n")
add_library(easyfly_native_imu SHARED native_imu.cpp)
target_compile_features(easyfly_native_imu PRIVATE cxx_std_17)
target_include_directories(easyfly_native_imu PRIVATE . upstream ${GAZEBO_INCLUDE_DIRS} "${EASYFLY_IMU_PROTO_INCLUDE}" "${EASYFLY_GAZEBO_PROTO_INCLUDE}")
target_link_libraries(easyfly_native_imu PRIVATE ${GAZEBO_LIBRARIES} Eigen3::Eigen "${EASYFLY_IMU_PROTO_LIBRARY}")
install(TARGETS easyfly_native_imu LIBRARY DESTINATION lib)
install(FILES README.md NOTICE ../safe_camera/LICENSE upstream/SHA256SUMS "${CMAKE_CURRENT_BINARY_DIR}/native_imu.json"
  DESTINATION share/${PROJECT_NAME}/native_imu)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_native_imu_contract test_contract.py TIMEOUT 30)
  add_executable(test_native_imu_stream test_stream.cpp)
  target_compile_features(test_native_imu_stream PRIVATE cxx_std_17)
  target_include_directories(test_native_imu_stream PRIVATE ${GAZEBO_INCLUDE_DIRS}
    "${EASYFLY_IMU_PROTO_INCLUDE}" "${EASYFLY_GAZEBO_PROTO_INCLUDE}")
  target_compile_definitions(test_native_imu_stream PRIVATE
    NATIVE_IMU_LIBRARY="$<TARGET_FILE:easyfly_native_imu>"
    IMU_RELAY_LIBRARY="$<TARGET_FILE:easyfly_imu_antialias>")
  target_link_libraries(test_native_imu_stream easyfly_transport_compat ${GAZEBO_LIBRARIES}
    "${EASYFLY_IMU_PROTO_LIBRARY}")
  add_dependencies(test_native_imu_stream easyfly_native_imu easyfly_imu_antialias)
  add_test(NAME test_native_imu_stream COMMAND test_native_imu_stream)
  set_tests_properties(test_native_imu_stream PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
  add_test(NAME test_native_imu_reset COMMAND test_native_imu_stream --reset-time)
  set_tests_properties(test_native_imu_reset PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
endif()
