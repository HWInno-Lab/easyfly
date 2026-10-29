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

YAML v1 also supports an explicit `explore` step. Legacy flat grammar is unchanged:

```yaml
schema_version: 1
frame_id: odom
units: {position: m, angle: rad, time: s}
failure_policy: abort
steps:
  - type: explore
    min_bound: [-2, -2, 0.5]
    max_bound: [2, 2, 2.5]
    return_home: true
    timeout: 120
    forbidden_boxes:
      - min: [1, 1, 0.5]
        max: [1.5, 1.5, 2]
```

The bounds are XYZ meters in odom. Each axis must span at least 0.8 m; Z must
remain inside 0.5–2.5 m. `return_home` is a required unquoted `true` or `false`;
`timeout` defaults to 120 s and must be 5–900 s. At most 16 forbidden boxes are
accepted, each with finite strict min < max. Omitted boxes mean an empty list.
This example is not a ready-to-fly mission: FUEL additionally checks the actual
map volume, minimum eligible coverage, current in-bounds armed OFFBOARD state
and fresh localization. No takeoff or landing is added implicitly. The mission
executor discovers `/explore` and `/exploration/ownership` before any step, and
keeps its lease after cancellation or public failure until child ownership drains.
