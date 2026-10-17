# Installed-package regression; never starts a ROS context or backend.
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CONSUMER_SOURCE}" -B "${CONSUMER_BUILD}"
  RESULT_VARIABLE configure_result OUTPUT_VARIABLE configure_out ERROR_VARIABLE configure_error)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR "Legacy consumer configure failed: ${configure_out}\n${configure_error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${CONSUMER_BUILD}" --parallel 2
  RESULT_VARIABLE build_result OUTPUT_VARIABLE build_out ERROR_VARIABLE build_error)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR "Legacy consumer build failed: ${build_out}\n${build_error}")
endif()
execute_process(COMMAND "${CONSUMER_BUILD}/legacy_consumer" RESULT_VARIABLE run_result)
if(NOT run_result EQUAL 0)
  message(FATAL_ERROR "Legacy consumer result mismatch: ${run_result}")
endif()
