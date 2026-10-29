cmake_minimum_required(VERSION 3.8)
project(easyfly_openvins_integration)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/openvins_guard scripts/openvins_alignment scripts/openvins_startup
  scripts/openvins_navigation
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md CONTRACT.md LICENSE DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_alignment test/test_alignment.py)
  ament_add_pytest_test(test_alignment_policy_buffer test/test_alignment_policy_buffer.py)
  ament_add_pytest_test(test_alignment_ros test/test_alignment_ros.py TIMEOUT 60
    ENV ROS_DOMAIN_ID=207)
  ament_add_pytest_test(test_gate test/test_gate.py)
  ament_add_pytest_test(test_guard_ros test/test_guard_ros.py TIMEOUT 60
    ENV ROS_DOMAIN_ID=218)
  ament_add_pytest_test(test_launch test/test_launch.py)
  ament_add_pytest_test(test_startup test/test_startup.py)
  ament_add_pytest_test(test_shutdown test/test_shutdown.py)
  ament_add_pytest_test(test_navigation test/test_navigation.py)
  ament_add_pytest_test(test_navigation_ros test/test_navigation_ros.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=211)
  ament_add_pytest_test(test_native_timing_ros test/test_native_timing_ros.py TIMEOUT 40
    ENV ROS_DOMAIN_ID=208)
endif()
ament_package()
