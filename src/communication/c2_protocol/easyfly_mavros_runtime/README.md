# Owned MAVROS runtime

This private build uses the unmodified MAVROS 2.14.0 plugin ABI and installed
plugins. Upstream source and license notices are retained in `upstream/` with
archive and file hashes. The library is private to this executable through
`$ORIGIN/runtime`; it does not replace `/opt/ros` or export a global library path.

After the main executor stops, Router detaches its endpoint table under its
mutex, then closes and joins connections outside that mutex. UAS cancels
and joins its plugin executor before clearing subscriptions and plugin owners.
The executable verifies both Router and UAS ownership has expired before exit.
The installed library fingerprint pins the ABI consumed by existing plugins.

This addresses the captured shutdown path in F-016: the original process was
in shared-library finalization while a UDP callback still used MAVLink tables.
Normal message routing, estimator data, time policies and flight parameters are
unchanged. No exit status is hidden and no worker is detached.


The private MAVConn 2.14.0 library also retires serial I/O outside its callback
mutex. The original serial close path could hold that mutex while joining the
receiver, whose cancellation callback tried to acquire the same mutex. A captured
actual PTY/MAVROS thread stack confirms this cycle. Peer-error callbacks do not
join their own thread; the owning endpoint still joins an already-closed device.
Cancellation handlers drain before owners and MAVLink tables are released.

The library uses exact upstream source/header snapshots and unchanged object
layouts; only the serial close implementation and a required compatibility marker
are derived. The executable's private RPATH and required marker prevent silently
loading the system library. No `/opt/ros` library is replaced. Regressions cover
UDP, active serial cancellation, PTY peer hangup, SIGINT and SIGTERM. PTY tests do
not certify physical USB, baud-rate throughput or flight hardware.

SIGINT/SIGTERM now request executor cancellation while keeping the ROS context
valid. The main executor joins in-flight plugin construction before Router/UAS
retirement and final context shutdown. This also handles an interrupt during
startup: previously the default ROS signal handler could invalidate the context
while a plugin was creating a service, causing an uncaught RCLError/SIGABRT.
Regression tests stop the actual native executable before and during plugin
construction, in addition to active UDP/serial shutdown.
