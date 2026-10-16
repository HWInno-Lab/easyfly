cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake REQUIRED)
project(easyfly_px4_sitl)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_hrt_compatibility test/test_hrt_compatibility.py)
  ament_add_pytest_test(test_hrt_selection test/test_hrt_selection.py)
  ament_add_pytest_test(test_mag_compatibility test/test_mag_compatibility.py)
  ament_add_pytest_test(test_scoped_timer test/test_scoped_timer.py TIMEOUT 15)
  ament_add_pytest_test(test_telemetry test/test_telemetry.py)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME easyfly_perception_sitl_launch_contract
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_perception_sitl_launch_contract.py)
  add_test(
    NAME easyfly_iris_sdf_physics_and_camera_contract
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/validate_easyfly_iris_sdf.py)
  add_test(
    NAME easyfly_sitl_session_guard
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_session_guard.py)
  set_tests_properties(easyfly_sitl_session_guard PROPERTIES TIMEOUT 30)
endif()

install(PROGRAMS scripts/prepare_px4_hrt.py
  DESTINATION lib/${PROJECT_NAME} RENAME prepare_px4_hrt)
install(DIRECTORY launch models rviz compat
  DESTINATION share/easyfly_px4_sitl
  PATTERN ".git" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
