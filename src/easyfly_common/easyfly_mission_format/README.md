# EasyFly shared mission format

Pure C++17 mission models, legacy and YAML v1 static validation, bounded regular-file
reading, UTF-8 checks and exact-byte SHA-256 metadata. No ROS runtime, Qt, SDK,
flight controller, action client, backend process or task execution is included.
Validation is not authorization to fly. Existing grammar, limits, defaults and
the `easyfly_mission_fsm` C++ namespace are retained.

Consumers use `find_package(easyfly_mission_format REQUIRED)`, link
`easyfly_mission_format::easyfly_mission_format`, and include
`easyfly_mission_format/mission_format.hpp` or `legacy_mission.hpp`.
The runtime package retains the old `easyfly_mission_fsm/...` forwarding headers,
old CMake target and old `ros2 run easyfly_mission_fsm mission_validate` entry.
There is one parser implementation; rebuild dependent packages together.

For offline validation without the runtime package:

```bash
ros2 run easyfly_mission_format mission_validate /absolute/path/mission.yaml
```

The package requires yaml-cpp and OpenSSL. Its ament build metadata does not make
it a ROS node. New-prefix and compatibility acceptance results are recorded in
the repository development records; clean-machine dependency installation remains separate.
