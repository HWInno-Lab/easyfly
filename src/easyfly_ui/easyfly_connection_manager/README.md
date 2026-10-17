# EasyFly connection manager

CM-0/CM-1 initial observation implementation for Ubuntu 22.04, ROS Humble and Qt5.
The manager and supervisor initialize no ROS contexts. Each connection starts an
independent existing desktop with a fixed, explicit DDS environment. They never
start MAVROS, SITL, QGC, flight commands, recording or mutable UI ports.

Build with the system ROS underlay and a new installation prefix:

```bash
source /opt/ros/humble/setup.bash
connection_build_root=/absolute/path/new-connection-build
colcon --log-base "$connection_build_root/log" build \
  --build-base "$connection_build_root/build" --install-base "$connection_build_root/install" \
  --packages-up-to easyfly_connection_manager --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DEASYFLY_ENABLE_MEDIA=OFF -DEASYFLY_ENABLE_RVIZ_EMBED=OFF
source "$connection_build_root/install/setup.bash"
ros2 run easyfly_connection_manager easyfly_connections
```

The manager requires the installed desktop's connection protocol/ABI1 manifest.
The graph-evidence snapshot uses libeasyfly_ui_bridge.so.1. Rebuild the bridge,
desktop and linked consumers together. Do not mix source headers or old overlays.

Run the installed-prefix tests explicitly; omitting test's install-base selects
the workspace's unrelated default install, even if build-base was specified:

```bash
colcon --log-base "$connection_build_root/test-log" test \
  --build-base "$connection_build_root/build" --install-base "$connection_build_root/install" \
  --packages-select easyfly_connection_manager easyfly_ui_bridge easyfly_desktop \
  --executor sequential --return-code-on-test-failure --ctest-args --output-on-failure
```

The installed `share/applications/easyfly-connections.desktop` launches the same
packaged script without a terminal. It is not automatically copied into a user's
desktop or system applications directory. Regenerate this file after relocating
the installation; its Exec field contains the installation prefix.

Create a profile, save it, then choose Connect observation. Supported fields are
domain 0–232, localhost/LAN, simulation/system time, UI profile and camera topics.
The current installation environment and `rmw_fastrtps_cpp` default discovery are
used. Advanced interface/peer/discovery-server presets are not implemented in this
first batch. LAN needs compatible peers and network access; a domain is not an IP
address or authentication. PX4-native `/fmu/*` requires an EasyFly adapter.
Match the existing platform's domain. New environments can use the recommended
0–101 range; other valid IDs still depend on available participant/OS ports.
See the [Humble domain guide](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Domain-ID.rst).

Profiles use schema 1 and are atomically stored under
`$XDG_CONFIG_HOME/easyfly/connections.json` (the normal Qt fallback is ~/.config).
The containing directory must be owned by this user with mode0700, and files
mode0600. Invalid existing permissions/schema/data are preserved and rejected,
never silently repaired or overwritten. Import/export uses the same private-file
rules. Import creates new IDs and does not connect. At most 32 profiles are kept.

Connect starts only a default read-only desktop. The effective profile is frozen
inside that workspace; use its Return to connection manager menu to change it.
Close the old workspace normally before connecting another. Existing draft exit
questions are displayed in the workspace. Unresolved operations reject a managed
close. There is no process kill or backend restart fallback.

The supervisor remains when either window closes, allowing the manager to reopen
and query the same session. After closing the workspace, an idle supervisor can
be stopped with the management-service menu (the local exit_service operation). This batch does
not adopt sessions after a supervisor crash. It does not delete old task journals.

The manager distinguishes a running process from fresh workspace status, graph
discovery, clock validity and actual fresh `/uav_state`. Multiple state publishers
invalidate state/trajectory rather than combine aircraft. Graph evidence is
sampled at most once per second; discovery loss is not an immediate physical
disconnect. Neither UAVState nor graph endpoint names authenticate a vehicle.

Tests include bounded file/IPC input, configuration isolation, request replay,
concurrent sessions, actual worker lifecycle and Qt form behavior. test_live_session
uses real DDS/ROS desktop processes with typed fixtures on localhost domains97/98;
run these fixed-domain tests serially. It checks source conflict, pause/rewind and
zero control requests without a flight backend. Native UI, real two-machine networking and
hardware results must be recorded independently, not inferred from mock tests.
See CONTRACT.md for the frozen initial protocol and scope.
