# EasyFly desktop platform integration tests

This developer-only package retains the existing import, parameter/media and
managed-task integration suites. Tests use isolated platform services, private
temporary files and MOCK action/telemetry/image backends, never a real aircraft.
It is deliberately outside the read-only desktop's package dependency closure.

Build the updated desktop, parser and platform services first, then explicitly
build and test `easyfly_desktop_tests` with `BUILD_TESTING=ON`. The installed
`easyfly_desktop::desktop_widgets` target propagates its actual MEDIA/RVIZ feature
configuration; MEDIA OFF does not count as recording/timeline coverage.
Fixtures are shared from the adjacent desktop source checkout, not copied.

```bash
colcon build --packages-select easyfly_desktop_tests --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select easyfly_desktop_tests --ctest-args --output-on-failure -j1
colcon test-result --test-result-base build/easyfly_desktop_tests/test_results --verbose
```

Desktop-only unit/ROS mock tests remain in `easyfly_desktop`; all three platform
suites must now be run here as well. Do not run ON/OFF copies concurrently in
the same fixed domains 129, 124 or 115, or use those domains for live platforms.
No test is removed or treated as passed merely because it moved packages.
