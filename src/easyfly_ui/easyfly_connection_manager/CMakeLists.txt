cmake_minimum_required(VERSION 3.8)
project(easyfly_connection_manager)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(easyfly_params REQUIRED)
find_package(easyfly_backend_registry REQUIRED)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${easyfly_backend_registry_DIR}/../config/algorithm_ui.json")
file(READ "${easyfly_backend_registry_DIR}/../config/algorithm_ui.json" ALGORITHM_UI_SCHEMA)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${easyfly_params_DIR}/../config/simulation_modules.json")
file(READ "${easyfly_params_DIR}/../config/simulation_modules.json" SIMULATION_MODULE_SCHEMA)
configure_file(cmake/algorithm_schema.hpp.in algorithm_schema.hpp @ONLY)
# Record an installation default as JSON data, never as shell source.
set(EASYFLY_PX4_COMPAT_ROOT "$ENV{EASYFLY_PX4_HRT_COMPAT_DIR}" CACHE PATH
  "Reviewed PX4 compatibility cache root containing px4-input")
execute_process(COMMAND ${CMAKE_COMMAND} -E env
  "EASYFLY_PX4_HRT_COMPAT_DIR=${EASYFLY_PX4_COMPAT_ROOT}" ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/write_installation.py
  ${CMAKE_CURRENT_BINARY_DIR}/installation.json COMMAND_ERROR_IS_FATAL ANY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/installation.json DESTINATION share/${PROJECT_NAME})
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network)
find_package(X11 REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
add_library(connection_components STATIC
  src/profile.cpp src/rpc.cpp src/supervisor.cpp src/manager_window.cpp src/native_window.cpp
  src/simulation_algorithm_panel.cpp
  src/scene_preview.cpp
  src/onboard_page.cpp include/easyfly_connection_manager/onboard_page.hpp
  include/easyfly_connection_manager/rpc.hpp)
target_include_directories(connection_components PUBLIC include)
target_include_directories(connection_components PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(connection_components PUBLIC Qt5::Widgets Qt5::Network ${X11_LIBRARIES})
target_include_directories(connection_components PRIVATE ${X11_INCLUDE_DIR})
target_compile_options(connection_components PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(connection_components PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(easyfly_connection_page SHARED src/page_plugin.cpp)
target_link_libraries(easyfly_connection_page connection_components)
install(TARGETS easyfly_connection_page LIBRARY DESTINATION lib)
add_executable(easyfly_connection_manager src/main.cpp)
target_link_libraries(easyfly_connection_manager connection_components)
add_executable(easyfly_connection_supervisor src/supervisor_main.cpp)
target_link_libraries(easyfly_connection_supervisor connection_components)
install(TARGETS easyfly_connection_manager easyfly_connection_supervisor
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/easyfly_connections scripts/easyfly_connection_backend
  scripts/easyfly_remote_service scripts/easyfly_remote_client scripts/easyfly_remote_algorithms
  scripts/easyfly_onboard_service scripts/easyfly_onboard_manager
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
configure_file(assets/easyfly-connections.desktop.in easyfly-connections.desktop @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/easyfly-connections.desktop DESTINATION share/applications)
install(FILES README.md CONTRACT.md REMOTE.md ONBOARD.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  configure_file(scripts/easyfly_connection_backend easyfly_connection_backend COPYONLY)
  add_executable(capture_manager test/capture_manager.cpp)
  target_link_libraries(capture_manager connection_components)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Test)
  ament_add_gtest(test_connections test/test_connections.cpp test/qt_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 90)
  target_link_libraries(test_connections connection_components Qt5::Test)
  target_compile_definitions(test_connections PRIVATE
    DESKTOP_BINARY_FROM_TEST="${CMAKE_CURRENT_BINARY_DIR}/fixture_workspace")
  add_executable(fixture_workspace test/fixture_workspace.cpp)
  target_link_libraries(fixture_workspace Qt5::Network)
  add_dependencies(test_connections fixture_workspace)
  set_tests_properties(test_connections PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  ament_add_gtest(test_scene_preview test/test_scene_preview.cpp test/qt_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 30)
  target_link_libraries(test_scene_preview connection_components Qt5::Test)
  set_tests_properties(test_scene_preview PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  ament_add_gtest(test_onboard_page test/test_onboard_page.cpp test/qt_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  target_link_libraries(test_onboard_page connection_components Qt5::Test)
  set_tests_properties(test_onboard_page PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  ament_add_gtest(test_onboard_interactions test/test_onboard_interactions.cpp test/qt_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  target_link_libraries(test_onboard_interactions connection_components Qt5::Test)
  set_tests_properties(test_onboard_interactions PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_backend test/test_backend.py TIMEOUT 60)
  ament_add_pytest_test(test_scene_previews test/test_scene_previews.py TIMEOUT 30)
  ament_add_pytest_test(test_installed_scenes test/test_installed_scenes.py TIMEOUT 60)
  ament_add_pytest_test(test_ownership test/test_ownership.py TIMEOUT 30)
  ament_add_pytest_test(test_remote_profiles test/test_remote_profiles.py TIMEOUT 30
    ENV PYTHONNOUSERSITE=1)
  ament_add_pytest_test(test_remote_observer test/test_remote_observer.py TIMEOUT 30
    ENV PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1)
  set_tests_properties(test_remote_observer PROPERTIES RUN_SERIAL TRUE)
  ament_add_pytest_test(test_live_session test/test_live_session.py TIMEOUT 120)
  ament_add_pytest_test(test_onboard test/test_onboard.py TIMEOUT 90
    ENV PYTHONNOUSERSITE=1 QT_QPA_PLATFORM=offscreen)
  ament_add_pytest_test(test_onboard_ui_bridge test/test_onboard_ui_bridge.py TIMEOUT 60
    ENV PYTHONNOUSERSITE=1)
  ament_add_pytest_test(test_onboard_adapters test/test_onboard_adapters.py TIMEOUT 20)
  ament_add_pytest_test(test_onboard_parameter_templates test/test_onboard_parameter_templates.py
    TIMEOUT 60 ENV PYTHONNOUSERSITE=1)
  ament_add_pytest_test(test_onboard_integration test/test_onboard_integration.py TIMEOUT 90
    ENV PYTHONNOUSERSITE=1 QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1)
  set_tests_properties(test_onboard_integration PROPERTIES RUN_SERIAL TRUE)
  set_tests_properties(test_live_session PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_LOCALHOST_ONLY=1" RUN_SERIAL TRUE)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
