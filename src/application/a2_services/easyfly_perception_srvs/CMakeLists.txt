cmake_minimum_required(VERSION 3.8)
project(easyfly_perception_srvs)

find_package(ament_cmake REQUIRED)
find_package(easyfly_perception_contracts REQUIRED)
include_directories(${easyfly_perception_contracts_INCLUDE_DIRS})
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/perception_services DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_shutdown test/test_shutdown.py TIMEOUT 45
    ENV ROS_DOMAIN_ID=194 ROS_LOG_DIR=/tmp/easyfly_d4_perception_shutdown)
  ament_add_pytest_test(test_services test/test_services.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=94 ROS_LOG_DIR=/tmp/easyfly_d2_perception
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
