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

The eleven transport source snapshots in `upstream/` are byte-identical
imports from that tag, verified by `upstream/SHA256SUMS`. Nine are compiled directly;
`src/connection.cpp` and `src/connection_manager.cpp` are separately maintained
derivatives of the corresponding upstream snapshots.
They are compiled against
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

The Connection derivative serializes shared IOManager creation, reference-count
changes and final unpublication under one lifecycle mutex. Previously two
concurrent destructors could both decrement the count, both observe zero and
both delete the same manager; a real two-connection native reproducer produced
a SIGSEGV through Connection::~Connection → IOManager::~IOManager → Dispose.
Final deletion occurs outside the mutex so joining an IO callback cannot block
another Connection destructor on that mutex. Concurrent construction is also
covered: it must not publish two managers or race the connection ID counter.
The object ABI, TCP protocol and all upstream snapshots remain unchanged.
This is a reproduced defect; it does not identify the missing historic planner
crash stacks or, by itself, explain every retained-manager timeout.

ConnectionManager::Fini cancels and closes every connection it owns before
releasing its list. OnAccept binds a strong ConnectionPtr into a pending read;
clearing the list alone leaves an operation → handler → connection → IOManager
ownership cycle while a peer remains open. The native pending-read test runs
an actual Master and ConnectionManager, keeps a TCP peer open without sending a
header, finalizes transport, and requires Finish to drain successfully before
closing that peer. The old code deterministically left live=1/pending=0 and
failed; the corrected code drains to zero. The list is moved under its mutex,
then Shutdown runs outside that mutex to allow reentrant shutdown callbacks.
This fixes a reproduced retained-owner defect without extending the deadline,
forcing successful exit, detaching threads, or suppressing an incomplete drain.

The server's terminal phase also calls `ShutdownConnections` after Server and
transport managers have finalized. Successful outgoing connects and accepted
streams are weakly registered without adding an owner or changing Connection's
layout. The terminal phase snapshots surviving streams, cancels/closes them
outside registry/manager locks, and rejects late stream admission. This reaches
pending reads whose connection was already removed from the manager's list.
Ordinary `Finish` does not initiate this terminal phase, so a refused Finish
continues to permit ordinary cleanup and further managers as before.

A native removed-read fixture reproduces live=1/pending=0 even after the fixed
manager Fini: the public RemoveConnection API has already removed its entry,
while the pending read handler still owns that connection. Terminal cancellation
breaks that cycle while the raw TCP peer stays open. The regression additionally
retains a separate explicit Connection owner: Finish must still refuse it through
the unchanged five-second deadline, then join and succeed only after it is
released. No live count is reset and no still-owned IO state is discarded.
This additional reproduced case does not uniquely attribute every historical
retained-manager observation.

Nonempty socket reads run `ConnectionReadTask` in the Connection's own oneTBB
TaskGroup. oneTBB destroys the task functor before releasing its wait context.
If the functor releases the last Connection reference, the member TaskGroup
would therefore wait for the very task currently destroying it. A native TCP
fixture reproduces this self-wait and its retained live-manager footprint.

All five Connection factory sites in the private transport use `MakeConnection`.
Its deleter transfers the whole object to the independent cleanup thread only
when `tbb::task::current_context()` identifies an executing task. Ordinary owner
reset remains synchronous, including waiting for callbacks that use raw owners.
The cleanup thread executes the original complete Connection destructor and
TaskGroup wait; pending accounting covers that entire operation. No task wait
or IO join is skipped. The native regression checks both ordinary owner blocking
and last-reference release from a real nonempty TCP read task.

The reviewed oneTBB implementation returns null outside an executing task,
including after a task wait and within an otherwise idle task arena:
[oneTBB v2021.5.0 task_dispatcher.cpp](https://github.com/oneapi-src/oneTBB/blob/v2021.5.0/src/tbb/task_dispatcher.cpp#L128).
This fixes the reproduced self-wait; independent scene exits still determine
whether the full runtime qualifies.

A captured bookstore shutdown core retained a closed Connection through a
SubscriptionTransport in `/gazebo/default/response`. Its Publication still
owned four Publishers; all four had already cleared their node but retained
that Publication. Gazebo finalizes transport before physics/sensors, so a late
Publisher::Fini can find an empty topic table. The original Unadvertise lookup
then cannot remove its Publisher, leaving the reciprocal ownership cycle.
`src/publisher.cpp` releases its own Publication registration/reference even
after topic removal. Repeated Fini and post-Fini publication/count checks are
safe. The original upstream source remains unmodified for provenance.
A real native fixture recreates the same four-Publisher/TCP-subscription chain,
finalizes transport first, then nodes/publishers. Weak pointers expire and
Finish drains without removing the retained peer to make the test pass.

A separate real TCP-backlog fixture holds ConnectToRemoteHost in SYN_SENT until
Fini returns, then permits the real handshake. The old implementation inserts
that late connection into the cleared manager list. Incoming/outgoing admission
now shares the existing connection mutex with Stop; outgoing connections check
again after their blocking handshake, and rejected sockets close outside that
mutex. Lifecycle bool polling uses acquire/release builtins without changing the
pinned class layout. This separate defect was reproduced independently; the
captured bookstore core had an empty manager list and is attributed to the
Publication cycle, not to this admission race.
