cmake_minimum_required(VERSION 3.8)
project(easyfly_precision_landing)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/landing_transform_node scripts/landing_gateway_node
  scripts/landing_camera_mount_node scripts/prepare_down_camera.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DYNAMIC.md DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/ DESTINATION include)
  ament_export_include_directories(include)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_policy test/test_policy.cpp)
  target_include_directories(test_policy PRIVATE include)
  ament_add_pytest_test(test_geometry test/test_geometry.py)
  ament_add_pytest_test(test_extrinsics test/test_extrinsics.py TIMEOUT 15)
  ament_add_pytest_test(test_dynamic_mount_ros test/test_dynamic_mount_ros.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=232)
  ament_add_pytest_test(test_configuration test/test_configuration.py TIMEOUT 15)
  ament_add_pytest_test(test_simulation_setup test/test_simulation_setup.py TIMEOUT 15)
  ament_add_pytest_test(test_simulation_scenarios test/test_simulation_scenarios.py TIMEOUT 15)
  ament_add_pytest_test(test_simulation_faults test/test_simulation_faults.py TIMEOUT 20
    ENV ROS_DOMAIN_ID=209)
  ament_add_pytest_test(test_simulation_bridge_faults test/test_simulation_bridge_faults.py TIMEOUT 20
    ENV ROS_DOMAIN_ID=211)
  ament_add_pytest_test(test_simulation_arm_faults test/test_simulation_arm_faults.py TIMEOUT 25
    ENV ROS_DOMAIN_ID=213)
  ament_add_pytest_test(test_simulation_transaction_faults
    test/test_simulation_transaction_faults.py TIMEOUT 25 ENV ROS_DOMAIN_ID=214)
  ament_add_pytest_test(test_simulation_precision_cancel_faults
    test/test_simulation_precision_cancel_faults.py TIMEOUT 15)
  ament_add_pytest_test(test_simulation_land_cancel_faults
    test/test_simulation_land_cancel_faults.py TIMEOUT 15)
  ament_add_pytest_test(test_signal_order test/test_signal_order.py TIMEOUT 15
    ENV ROS_DOMAIN_ID=125 ROS_LOG_DIR=/tmp/easyfly_d5_signal_order)
  ament_add_pytest_test(test_signal_rebuild_ros test/test_signal_rebuild_ros.py TIMEOUT 20
    ENV ROS_DOMAIN_ID=129)
  ament_add_pytest_test(test_signal_take_ros test/test_signal_take_ros.py TIMEOUT 30
    ENV ROS_DOMAIN_ID=126 ROS_LOG_DIR=/tmp/easyfly_d5_signal_take)
  ament_add_pytest_test(test_ground_reference test/test_ground_reference.py TIMEOUT 15)
  ament_add_pytest_test(test_reference_admission test/test_reference_admission.py TIMEOUT 15)
  ament_add_pytest_test(test_simulation_collision test/test_simulation_collision.py TIMEOUT 15)
  ament_add_pytest_test(test_replay_tag_frames test/test_replay_tag_frames.py TIMEOUT 15)
  ament_add_pytest_test(test_diagnostics test/test_diagnostics.py TIMEOUT 15)
  ament_add_pytest_test(test_observation_ros test/test_observation_ros.py TIMEOUT 30)
  set_tests_properties(test_observation_ros PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=192")
  ament_add_pytest_test(test_gateway_ros test/test_gateway_ros.py TIMEOUT 30)
  set_tests_properties(test_gateway_ros PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=189")
  ament_add_pytest_test(test_timing_pair_ros test/test_timing_pair_ros.py TIMEOUT 30)
  set_tests_properties(test_timing_pair_ros PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=198")
  ament_add_pytest_test(test_timing_queue_ros test/test_timing_queue_ros.py TIMEOUT 30)
  set_tests_properties(test_timing_queue_ros PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=199")
  ament_add_pytest_test(test_receiver_recovery test/test_receiver_recovery.py TIMEOUT 15)
  set_tests_properties(test_receiver_recovery PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=202")
  ament_add_pytest_test(test_pair_history test/test_pair_history.py TIMEOUT 15)
  set_tests_properties(test_pair_history PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=206")
  ament_add_pytest_test(test_stream_pair_ros test/test_stream_pair_ros.py TIMEOUT 40)
  set_tests_properties(test_stream_pair_ros PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=205")
  ament_add_pytest_test(test_frame_timing_ros test/test_frame_timing_ros.py TIMEOUT 30)
  ament_add_pytest_test(test_publication_backpressure_ros
    test/test_publication_backpressure_ros.py TIMEOUT 45)
  set_tests_properties(test_frame_timing_ros test_publication_backpressure_ros
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=207" RUN_SERIAL TRUE)
endif()

ament_package()
