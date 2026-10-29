# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0.
find_package(gazebo 11.10.2 EXACT REQUIRED)
find_library(EASYFLY_SYSTEM_SENSORS gazebo_sensors
  PATHS ${GAZEBO_LIBRARY_DIRS} NO_DEFAULT_PATH REQUIRED)
file(SHA256 "${EASYFLY_SYSTEM_SENSORS}" EASYFLY_SYSTEM_SENSORS_SHA)
if(NOT EASYFLY_SYSTEM_SENSORS_SHA STREQUAL
    "7c065bd8bc755428665c6fe7771b59d9c0179907ee9f300cf2d39870d8883aaa")
  message(FATAL_ERROR "Unsupported Gazebo sensors build; compatibility review required")
endif()
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include/gazebo")
file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/upstream"
  "${CMAKE_CURRENT_BINARY_DIR}/include/gazebo/sensors" SYMBOLIC)
add_library(easyfly_sensor_compat SHARED
  upstream/AltimeterSensor.cc
  upstream/CameraSensor.cc
  upstream/ContactSensor.cc
  upstream/DepthCameraSensor.cc
  upstream/ForceTorqueSensor.cc
  upstream/GaussianNoiseModel.cc
  upstream/GpsSensor.cc
  upstream/GpuRaySensor.cc
  upstream/ImuSensor.cc
  upstream/LogicalCameraSensor.cc
  upstream/MagnetometerSensor.cc
  upstream/MultiCameraSensor.cc
  upstream/Noise.cc
  upstream/RaySensor.cc
  upstream/RFIDSensor.cc
  upstream/RFIDTag.cc
  upstream/SensorsIface.cc
  src/sensor.cpp
  upstream/SensorFactory.cc
  upstream/SensorManager.cc
  upstream/SensorTypes.cc
  upstream/SonarSensor.cc
  upstream/WideAngleCameraSensor.cc
  upstream/WirelessReceiver.cc
  upstream/WirelessTransceiver.cc
  upstream/WirelessTransmitter.cc
)
target_include_directories(easyfly_sensor_compat PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/include" ${GAZEBO_INCLUDE_DIRS})
target_compile_features(easyfly_sensor_compat PRIVATE cxx_std_17)
target_compile_definitions(easyfly_sensor_compat PRIVATE BUILDING_DLL_GZ_SENSORS)
target_link_libraries(easyfly_sensor_compat PRIVATE
  gazebo_common gazebo_rendering gazebo_physics ${Boost_LIBRARIES})
# Upstream factory symbols otherwise collide with the ROS gazebo_msgs factory.
target_link_options(easyfly_sensor_compat PRIVATE "-Wl,-Bsymbolic-functions")
set_target_properties(easyfly_sensor_compat PROPERTIES
  OUTPUT_NAME gazebo_sensors VERSION 11.10.2 SOVERSION 11
  INSTALL_RPATH "$ORIGIN/../transport")
install(TARGETS easyfly_sensor_compat
  LIBRARY DESTINATION lib/${PROJECT_NAME}/sensors)
install(FILES README.md NOTICE ../safe_camera/LICENSE upstream/SHA256SUMS
  DESTINATION share/${PROJECT_NAME}/safe_sensors)
if(BUILD_TESTING)
  add_executable(test_sensor_clock test/test_sensor_clock.cpp)
  target_include_directories(test_sensor_clock PRIVATE ${GAZEBO_INCLUDE_DIRS})
  target_compile_features(test_sensor_clock PRIVATE cxx_std_17)
  target_link_libraries(test_sensor_clock PRIVATE easyfly_sensor_compat ${GAZEBO_LIBRARIES})
  add_test(NAME test_sensor_clock COMMAND test_sensor_clock)
  set_tests_properties(test_sensor_clock PROPERTIES TIMEOUT 20)
endif()
