cmake_minimum_required(VERSION 3.8)
project(easyfly_sim_launcher)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/d5_simulation DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/scene_catalog DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/scene_generator DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/scene_sitl.py DESTINATION lib/${PROJECT_NAME} RENAME scene_sitl)
install(PROGRAMS scripts/navigation_sitl.py
  DESTINATION lib/${PROJECT_NAME} RENAME navigation_sitl)
install(PROGRAMS scripts/battery_sitl.py
  DESTINATION lib/${PROJECT_NAME} RENAME battery_sitl)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_profile_launch test/test_profile_launch.py)
  ament_add_pytest_test(test_navigation_sitl test/test_navigation_sitl.py TIMEOUT 15)
  ament_add_pytest_test(test_battery_sitl test/test_battery_sitl.py TIMEOUT 15)
  ament_add_pytest_test(test_scene_catalog test/test_scene_catalog.py TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_scene_launch test/test_scene_launch.py TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_indoor_scene test/test_indoor_scene.py TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_scene_generator test/test_scene_generator.py TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_warehouse_scene test/test_warehouse_scene.py TIMEOUT 30
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
