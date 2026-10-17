cmake_minimum_required(VERSION 3.8)
project(easyfly_mission_srvs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(easyfly_core REQUIRED)
find_package(easyfly_mission_fsm REQUIRED)
find_package(ament_index_cpp REQUIRED)
set(CMAKE_CXX_STANDARD 17)
add_executable(mission_services src/mission_services.cpp src/import_catalog.cpp src/import_worker.cpp)
get_filename_component(EASYFLY_IMPORT_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
get_filename_component(EASYFLY_IMPORT_INSTALL_ROOT "${CMAKE_INSTALL_PREFIX}/.." ABSOLUTE)
target_compile_definitions(mission_services PRIVATE
  EASYFLY_IMPORT_SOURCE_ROOT="${EASYFLY_IMPORT_SOURCE_ROOT}"
  EASYFLY_IMPORT_INSTALL_ROOT="${EASYFLY_IMPORT_INSTALL_ROOT}")
ament_target_dependencies(mission_services rclcpp rclcpp_action easyfly_msgs easyfly_mission_fsm ament_index_cpp)
target_link_libraries(mission_services easyfly_mission_fsm::easyfly_mission_format easyfly_core::easyfly_core)
target_compile_options(mission_services PRIVATE -Wall -Wextra)
install(TARGETS mission_services DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_import_worker test/test_import_worker.cpp src/import_worker.cpp TIMEOUT 30)
  target_include_directories(test_import_worker PRIVATE src)
  ament_target_dependencies(test_import_worker easyfly_msgs)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_import test/test_import.py TIMEOUT 90
    ENV ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/easyfly_ui_import_test)
  ament_add_pytest_test(test_reviewed test/test_reviewed.py TIMEOUT 120
    ENV ROS_DOMAIN_ID=111 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/easyfly_ui_reviewed_test)
  ament_add_pytest_test(test_details test/test_details.py TIMEOUT 60
    ENV ROS_DOMAIN_ID=109 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/easyfly_ui_details_test)
  ament_add_pytest_test(test_services test/test_services.py TIMEOUT 60
    ENV ROS_DOMAIN_ID=95 ROS_LOG_DIR=/tmp/easyfly_d2_missions)
  ament_add_pytest_test(test_launch test/test_launch.py
    ENV ROS_LOG_DIR=/tmp/easyfly_d2_launch)
  ament_add_pytest_test(test_platform_smoke test/test_platform_smoke.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=97 ROS_LOG_DIR=/tmp/easyfly_d2_platform_smoke)
endif()

ament_package()
