cmake_minimum_required(VERSION 3.8)
project(easyfly_desktop)

find_package(ament_cmake REQUIRED)
find_package(easyfly_ui_bridge REQUIRED)
find_package(easyfly_ui_control REQUIRED)
find_package(easyfly_mission_format REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Network)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(easyfly_msgs REQUIRED)
option(EASYFLY_ENABLE_MEDIA "Build opt-in local MJPEG recording backend" OFF)
if(EASYFLY_ENABLE_MEDIA)
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio)
endif()
option(EASYFLY_ENABLE_RVIZ_EMBED "Build experimental version-pinned read-only RViz embedding" OFF)
if(EASYFLY_ENABLE_RVIZ_EMBED)
  find_package(rviz_common 11.2.26 EXACT REQUIRED)
  find_package(rviz_rendering 11.2.26 EXACT REQUIRED)
  find_package(rviz_default_plugins 11.2.26 EXACT REQUIRED)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(desktop_widgets STATIC
  include/easyfly_desktop/mission_editor.hpp
  include/easyfly_desktop/mission_geometry.hpp
  include/easyfly_desktop/parameter_controls.hpp
  include/easyfly_desktop/import_controls.hpp
  include/easyfly_desktop/rviz_view.hpp
  src/source.cpp src/presentation.cpp src/mission_presentation.cpp src/image_pipeline.cpp src/widgets.cpp
  src/camera_overlay.cpp
  assets/easyfly_assets.qrc src/branding.cpp
  src/theme.cpp src/tools.cpp src/mission_controls.cpp src/mission_draft.cpp
  src/mission_geometry.cpp src/mission_editor.cpp src/parameter_controls.cpp src/import_controls.cpp
  src/rviz_view.cpp src/rviz_geometry.cpp src/rviz_layers.cpp src/recording.cpp
  src/recording_panel.cpp src/main_window.cpp)
target_include_directories(desktop_widgets PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE src)
target_compile_options(desktop_widgets PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(desktop_widgets PROPERTIES POSITION_INDEPENDENT_CODE ON AUTOMOC ON AUTORCC ON)
ament_target_dependencies(desktop_widgets PUBLIC easyfly_ui_bridge easyfly_ui_control easyfly_mission_format ament_index_cpp)
ament_target_dependencies(desktop_widgets PUBLIC nav_msgs sensor_msgs visualization_msgs easyfly_msgs)
get_filename_component(EASYFLY_DRAFT_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(EASYFLY_DRAFT_INSTALL_ROOT "${CMAKE_INSTALL_PREFIX}/.." ABSOLUTE)
target_compile_definitions(desktop_widgets PRIVATE
  EASYFLY_DRAFT_SOURCE_ROOT="${EASYFLY_DRAFT_SOURCE_ROOT}"
  EASYFLY_DRAFT_INSTALL_ROOT="${EASYFLY_DRAFT_INSTALL_ROOT}")
target_link_libraries(desktop_widgets PUBLIC Qt5::Widgets)
if(EASYFLY_ENABLE_MEDIA)
  target_compile_definitions(desktop_widgets PUBLIC EASYFLY_ENABLE_MEDIA)
  target_include_directories(desktop_widgets SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(desktop_widgets PUBLIC ${OpenCV_LIBS})
endif()
if(EASYFLY_ENABLE_RVIZ_EMBED)
  target_compile_definitions(desktop_widgets PUBLIC EASYFLY_ENABLE_RVIZ_EMBED)
  target_link_libraries(desktop_widgets PUBLIC rviz_common::rviz_common rviz_rendering::rviz_rendering
    rviz_default_plugins::rviz_default_plugins)
endif()
add_executable(easyfly_desktop src/main.cpp)
target_link_libraries(easyfly_desktop desktop_widgets Qt5::Network)
install(TARGETS easyfly_desktop DESTINATION lib/${PROJECT_NAME})
install(TARGETS desktop_widgets EXPORT desktopTargets ARCHIVE DESTINATION lib)
if(EASYFLY_ENABLE_MEDIA)
  # Intentionally independent of desktop_widgets and all ament/ROS target dependencies.
  add_library(local_playback STATIC src/playback.cpp src/playback_window.cpp src/theme.cpp)
  target_include_directories(local_playback PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
    PRIVATE src)
  target_include_directories(local_playback SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(local_playback PUBLIC Qt5::Widgets opencv_core opencv_imgproc opencv_imgcodecs)
  target_compile_options(local_playback PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(easyfly_recording_player src/playback_main.cpp)
  target_link_libraries(easyfly_recording_player local_playback)
  install(TARGETS easyfly_recording_player DESTINATION lib/${PROJECT_NAME})
  install(TARGETS local_playback EXPORT desktopTargets ARCHIVE DESTINATION lib)
endif()
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md OPERATIONS.md DESTINATION share/${PROJECT_NAME})
install(FILES cmake/connection_protocol.json DESTINATION share/${PROJECT_NAME})
ament_export_targets(desktopTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(easyfly_ui_bridge easyfly_ui_control easyfly_mission_format
  ament_index_cpp nav_msgs sensor_msgs visualization_msgs easyfly_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Test Network)
  find_package(mavros_msgs REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(tf2_msgs REQUIRED)
  ament_add_gtest(test_camera_overlay test/test_camera_overlay.cpp)
  target_link_libraries(test_camera_overlay desktop_widgets Qt5::Test)
  set_tests_properties(test_camera_overlay PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  if(EASYFLY_ENABLE_MEDIA)
    ament_add_gtest(test_playback test/test_playback.cpp test/qt_test_main.cpp
      SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
    target_link_libraries(test_playback local_playback desktop_widgets Qt5::Test)
    target_compile_definitions(test_playback PRIVATE
      EASYFLY_PLAYBACK_BINARY="$<TARGET_FILE:easyfly_recording_player>")
    add_dependencies(test_playback easyfly_recording_player)
    set_tests_properties(test_playback PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  endif()
  ament_add_gtest(test_recording test/test_recording.cpp test/qt_test_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  target_link_libraries(test_recording desktop_widgets Qt5::Test)
  set_tests_properties(test_recording PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  ament_add_gtest(test_rviz_geometry test/test_rviz_geometry.cpp TIMEOUT 30)
  target_link_libraries(test_rviz_geometry desktop_widgets)
  ament_add_gtest(test_rviz_view test/test_rviz_view.cpp test/qt_test_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  target_link_libraries(test_rviz_view desktop_widgets Qt5::Test)
  ament_target_dependencies(test_rviz_view nav_msgs rosgraph_msgs tf2_msgs)
  target_compile_definitions(test_rviz_view PRIVATE
    EASYFLY_RVIZ_DESKTOP_BINARY="$<TARGET_FILE:easyfly_desktop>"
    EASYFLY_RVIZ_SCREENSHOT_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/ui/d6-5/rviz")
  add_dependencies(test_rviz_view easyfly_desktop)
  set_tests_properties(test_rviz_view PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=131;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_mission_geometry test/test_mission_geometry.cpp test/qt_test_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  target_link_libraries(test_mission_geometry desktop_widgets Qt5::Test)
  target_compile_definitions(test_mission_geometry PRIVATE
    EASYFLY_GEOMETRY_SCREENSHOT_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/ui/d6-5/geometry")
  set_tests_properties(test_mission_geometry PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=120;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_mission_draft test/test_mission_draft.cpp test/qt_test_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  target_link_libraries(test_mission_draft desktop_widgets Qt5::Test)
  target_compile_definitions(test_mission_draft PRIVATE
    EASYFLY_DRAFT_SCREENSHOT_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/ui/d6-5")
  set_tests_properties(test_mission_draft PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=119;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_mission_controls test/test_mission_controls.cpp TIMEOUT 90)
  target_link_libraries(test_mission_controls desktop_widgets Qt5::Test)
  target_compile_definitions(test_mission_controls PRIVATE
    EASYFLY_DESKTOP_BINARY="$<TARGET_FILE:easyfly_desktop>"
    EASYFLY_MANAGED_SCREENSHOT_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/ui/d6-4")
  add_dependencies(test_mission_controls easyfly_desktop)
  set_tests_properties(test_mission_controls PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=93;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_desktop test/test_desktop.cpp TIMEOUT 90)
  target_link_libraries(test_desktop desktop_widgets Qt5::Test)
  target_compile_definitions(test_desktop PRIVATE
    EASYFLY_SCREENSHOT_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/ui/d6-2")
  ament_add_gtest(test_images test/test_images.cpp TIMEOUT 30)
  target_link_libraries(test_images desktop_widgets)
  add_executable(tool_fixture test/tool_fixture.cpp)
  target_link_libraries(tool_fixture Qt5::Core Qt5::Network)
  target_compile_definitions(test_mission_controls PRIVATE
    EASYFLY_TOOL_FIXTURE="$<TARGET_FILE:tool_fixture>")
  add_dependencies(test_mission_controls tool_fixture)
  ament_add_gtest(test_tools test/test_tools.cpp test/qt_test_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 60)
  target_link_libraries(test_tools desktop_widgets Qt5::Test Qt5::Network)
  target_compile_definitions(test_tools PRIVATE
    EASYFLY_TOOL_FIXTURE="$<TARGET_FILE:tool_fixture>")
  add_dependencies(test_tools tool_fixture)
  ament_add_gtest(test_desktop_ros test/test_desktop_ros.cpp TIMEOUT 90)
  target_link_libraries(test_desktop_ros desktop_widgets Qt5::Test)
  ament_target_dependencies(test_desktop_ros mavros_msgs)
  set_tests_properties(test_desktop test_desktop_ros test_tools PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;ROS_DOMAIN_ID=93;ROS_LOCALHOST_ONLY=1")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS "cmake/desktop_features.cmake.in")
