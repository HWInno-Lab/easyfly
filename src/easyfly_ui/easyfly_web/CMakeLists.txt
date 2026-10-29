cmake_minimum_required(VERSION 3.8)
project(easyfly_web)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/web_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config web DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_operations test/test_operations.py TIMEOUT 45
    APPEND_ENV PYTHONNOUSERSITE=1)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
