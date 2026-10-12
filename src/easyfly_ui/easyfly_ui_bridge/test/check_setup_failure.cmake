# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(test_environment ${CMAKE_COMMAND} -E env)
if(PARTIAL_CONTEXT)
  list(APPEND test_environment "ROS_DOMAIN_ID=92" "EASYFLY_UI_TEST_FAIL_AFTER_CONTEXT=1")
  set(expected_message "injected setup failure after context")
else()
  list(APPEND test_environment --unset=ROS_DOMAIN_ID --unset=EASYFLY_UI_TEST_FAIL_AFTER_CONTEXT)
  set(expected_message "Expected equality")
endif()
list(APPEND test_environment "ROS_LOCALHOST_ONLY=1")
execute_process(
  COMMAND ${test_environment} "${TEST_BINARY}"
    --gtest_filter=ObserverTest.AllReadonlyRequestsPreserveTaskSemanticsAndOtherContext
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE errors TIMEOUT 15)
if(NOT result STREQUAL "1" OR NOT output MATCHES "${expected_message}")
  message(FATAL_ERROR "Expected an assertion failure (exit 1), not a crash: ${result}\n${output}\n${errors}")
endif()
message(STATUS "Expected setup assertion failure cleaned safely, exit 1")
