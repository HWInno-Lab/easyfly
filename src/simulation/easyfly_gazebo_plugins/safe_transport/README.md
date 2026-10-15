# Project-local Gazebo transport lifetime compatibility

`easyfly_gzserver` is an independently named server executable. Its private RPATH
selects a complete project-local `libgazebo_transport.so.11` in its own
`transport/` subdirectory. The installed library binds internal IOManager calls
directly: an executable-only symbol override was tested and rejected as ineffective.
The private library is not installed on a general library search path. No launch
environment variable exposes it to `gz`, `gzclient`, PX4, ROS nodes, or other apps.
There is no LD_PRELOAD, system library overwrite, exception suppression, detached
worker, or change to the transport Connection protocol.

Derived Apache-2.0 sources:

- [Gazebo 11.10.2 IOManager.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/transport/IOManager.cc)
- [Gazebo 11.10.2 server_main.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/server_main.cc)

Eleven remaining transport implementation files in `upstream/` are byte-identical
imports from that tag, verified by `upstream/SHA256SUMS`. They are compiled against
installed headers, which match the tag (transport_pch.hh is not used). Original
copyright notices are retained. The source archive SHA256 is
`51976f29e375463e3bbf1343b7532b038728f13389e94e78b2fa8271911b2dcb`.

The ordinary destructor stops and joins the worker synchronously. If destruction
occurs on the IO worker, it transfers the entire private state to an independent
cleanup thread; that thread joins the original worker BEFORE deleting its thread,
work guard, service, and state. Stop on the worker only requests stop, retaining
all resources until destruction. No reset is called while run is active.

The server explicitly destroys Server, drains all deferred cleanup, requires zero
live managers, and joins the cleanup thread before successful exit. Finish waits
up to five seconds for both live managers and pending retirement to reach zero:
the last connection may still be owned by an executing WeakBinder callback even
when the retirement queue is empty. The reaper remains available during this
wait. A timeout still fails and does not close or free live state. Late manager
creation or incomplete cleanup fails loudly rather than freeing active resources.
A stuck callback means stuck cleanup, not a false successful shutdown.

Supported installation is Gazebo 11.10.2, installed IOManager header SHA256
`81584e70ee4e7042dc75fbf9fe703ba6f21df7da20ed9873a8305e3255709b63`
and transport library SHA256
`fd084b63c69f7b0157a8ec99e423d4cfd3f7005b89af67fd2f1ac88ac14ed90d`.
Configuration checks the original installed transport SHA256. Runtime requires
Connection to resolve to the same private library image as the compatibility
code, and checks the actual loaded Gazebo core SHA256
`984c29da37f618c4f72e2023733df82479bc6cc98352f6fb6f30e3ddd4c5899d`,
not only a version string or guessed library path.
Other distributions/builds are intentionally refused until independently reviewed.

Both EasyFly SITL launch entries explicitly select the private server. All
original arguments, model rates, session reservation and launch shutdown ownership
remain unchanged. To roll back, restore `gzserver` as the first command item in
both launch files and rebuild `easyfly_px4_sitl`; the original system executable
and library remain present. Do not add the private `transport/` directory to a
global LD_LIBRARY_PATH or Gazebo plugin path.

This addresses the deterministic last-connection-on-IO-thread defect. Without
the old natural crash stack it does not uniquely attribute the historic shutdown
incident, nor resolve the independent historic EGO SIGSEGV.

`test_transport_finish` holds the last actual TCP connection in its IO callback,
starts Finish while live=1/pending=0, then releases the callback. It requires
Finish to wait, join the deferred worker and return with both counts zero. This
reproduces the premature-failure window; it does not uniquely attribute every
historic Gazebo abort. The existing lifetime test also requires Finish to refuse
a manager that stays live through the deadline and allow further cleanup.
