# Managed simulation Fast DDS compatibility

This package builds the complete upstream Fast DDS **v2.6.11**, with one checked
change: snapshot the reader listener once under its existing recursive mutex, then
release that mutex before both unmatch callbacks and PDP status updates.
`RTPSReader::setListener` uses the same mutex. The existing participant
endpoint-list shared lock prevents native reader removal during traversal.
The DDS inner listener is a member of DataReaderImpl, whose destructor removes
the native reader before destroying its members, so its lifetime spans the
traversal. No new reader/PDP lock ordering is introduced. No system library is overwritten and no DDS packet, QoS,
transport, endpoint layout or safety policy is changed.

A real warehouse shutdown core stopped at `EDP::unpairWriterProxy + 406`:
the third `getListener()` returned null while a peer writer was being disposed.
The real native regression starts two RTPS participants and matched endpoints;
a separate thread clears the reader listener during the first removal callback.
The original installed 2.6.11 crashes at the same instruction. The fixed version
permits listener clearing during the first callback, keeps the captured DDS
inner listener valid through both callbacks, and exits normally. A prior
candidate held the reader mutex over callbacks; real shutdown exposed a
reader/PDP lock inversion. That failed candidate is not shipped.
The callback's bounded delay controls real concurrency; no fault is injected.

The official Apache-2.0 source archive is SHA256-pinned in CMake. For offline
builds, supply `-DEASYFLY_FASTDDS_ARCHIVE=/absolute/path/Fast-DDS-v2.6.11.tar.gz`.
Only the complete private library, its SHA256, original LICENSE, patch and this
README are installed. Headers come from the existing exact 2.6.11 dependency.
A first build downloads about 8 MB and builds the native middleware once.

Managed scene launch and explicit `scene_sitl` activate this library **before**
DDS initialization, validate its installed fingerprint and reject a process
already using another Fast DDS library. Children inherit its private search
path. The activation is process-local; empty-scene legacy launch is unchanged.
Ordinary tests exercise native endpoints and guards, never Gazebo, PX4 or flight.
This fixes the captured DDS race; it does not uniquely attribute old crashes
whose native stacks were not retained.

Managed scenes also load six explicitly named component-loader reply profiles.
A recorded launch lost replies for already-instantiated components because RMW
waited only the default 100 ms for the server's response writer to discover the
client's response reader. The abandoned sequential loader then encountered an
InvalidHandle during shutdown. The launch exit code remained a failed check.
`config/component_services.xml` gives only the six
`rr/<container>/_container/load_nodeReply` writers a 2-second discovery/send
budget. Defaults, participant transports, control/action service QoS, and all
flight admission limits remain unchanged. No component request is retried or
silently treated as successful. The scene probe requires all seven expected
components, including the second calibration component, before inspection.

Activation merges these named profiles with the caller's existing DDS XML into
an owned temporary file and propagates both DDS profile environment variables
through the launch context. The input is left intact; conflicting named policies
are rejected. The file remains alive until the parent has joined its children.
The scene probe verifies each component container inherits the six reply policies and every
owned DDS process retains the original transport configuration, recording the
actual bytes/hashes in the run evidence. Native QoS and subprocess inheritance
tests cover the 2-second named policies and unchanged 100-ms default.
The explicit middleware failure comes from
[Humble rmw_response.cpp](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/rmw_response.cpp).
