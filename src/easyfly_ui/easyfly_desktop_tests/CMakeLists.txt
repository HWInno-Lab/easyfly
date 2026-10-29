cmake_minimum_required(VERSION 3.8)
project(easyfly_desktop_tests)
find_package(ament_cmake REQUIRED)
find_package(easyfly_desktop REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Test Network)
  find_package(ament_index_cpp REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  # Shared value/Qt/tool fixtures stay in the same workspace checkout; no copied implementation.
  set(DESKTOP_TEST_SUPPORT "${CMAKE_CURRENT_SOURCE_DIR}/../easyfly_desktop/test")
  if(NOT EXISTS "${DESKTOP_TEST_SUPPORT}/support.hpp")
    message(FATAL_ERROR "Desktop integration tests require the EasyFly workspace source checkout")
  endif()
  set(QT_TEST_MAIN "${DESKTOP_TEST_SUPPORT}/qt_test_main.cpp")
  add_executable(tool_fixture "${DESKTOP_TEST_SUPPORT}/tool_fixture.cpp")
  target_link_libraries(tool_fixture Qt5::Core Qt5::Network)
  ament_add_gtest(test_import_controls test/test_import_controls.cpp "${QT_TEST_MAIN}"
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  ament_add_gtest(test_parameter_controls test/test_parameter_controls.cpp
    test/test_parameter_media.cpp "${QT_TEST_MAIN}" SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  ament_add_gtest(test_managed_stack test/test_managed_stack.cpp "${QT_TEST_MAIN}"
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 90)
  foreach(test_target test_import_controls test_parameter_controls test_managed_stack)
    target_include_directories(${test_target} PRIVATE "${DESKTOP_TEST_SUPPORT}")
    target_link_libraries(${test_target} easyfly_desktop::desktop_widgets Qt5::Test)
    ament_target_dependencies(${test_target} ament_index_cpp)
  endforeach()
  ament_target_dependencies(test_parameter_controls std_srvs)
  ament_target_dependencies(test_managed_stack rosgraph_msgs)
  if(easyfly_desktop_HAS_MEDIA)
    target_link_libraries(test_parameter_controls easyfly_desktop::local_playback)
    target_link_libraries(test_managed_stack easyfly_desktop::local_playback)
  endif()
  target_compile_definitions(test_import_controls PRIVATE
    EASYFLY_IMPORT_SCREENSHOT_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/ui/d6-5/import")
  target_compile_definitions(test_parameter_controls PRIVATE
    EASYFLY_PARAMETER_SCREENSHOT_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/ui/d6-5/parameters")
  target_compile_definitions(test_managed_stack PRIVATE
    EASYFLY_TOOL_FIXTURE="$<TARGET_FILE:tool_fixture>")
  add_dependencies(test_managed_stack tool_fixture)
  set_tests_properties(test_import_controls PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=129;ROS_LOCALHOST_ONLY=1")
  set_tests_properties(test_parameter_controls PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=124;ROS_LOCALHOST_ONLY=1")
  set_tests_properties(test_managed_stack PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=115;ROS_LOCALHOST_ONLY=1")
  find_package(ament_lint_auto REQUIRED)
  # Preserve native analysis; teach cppcheck the C++ and test-framework syntax.
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(LANGUAGE c++ LIBRARIES googletest qt
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../easyfly_desktop/test)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()
endif()
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
