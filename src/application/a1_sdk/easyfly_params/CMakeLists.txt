cmake_minimum_required(VERSION 3.8)
project(easyfly_params)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(easyfly_backend_registry REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} PATTERN algorithm_ui.json EXCLUDE)
install(FILES "${easyfly_backend_registry_DIR}/../config/algorithm_ui.json" DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/config_tool scripts/backend_selection scripts/parameter_manager DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_configuration test/test_configuration.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_simulation_modules test/test_simulation_modules.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_runtime test/test_runtime.py TIMEOUT 60
    ENV ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/easyfly_d2_params
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()
ament_package()
