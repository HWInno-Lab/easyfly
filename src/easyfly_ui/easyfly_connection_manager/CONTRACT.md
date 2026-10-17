# CM-0 and CM-1 connection contract

Schema and protocol version: 1. UiSnapshot ABI/bridge SONAME: 1. The desktop's
installed connection_protocol.json must declare matching protocol and ABI before
it can be selected. Only existing-platform observation is implemented.
FCU/SITL/QGC and remote backend mutation are later development batches. Static
peers and interface selection remain later CM-1 work. Existing six-package desktop
builds do not depend on this optional package.

Each profile has a UUID, name, observe mode, domain 0–232, local/LAN scope,
rmw_fastrtps_cpp/default discovery, boolean simulation time, UI profile label and
two absolute camera topics (empty disables a channel). Unknown or incorrectly
typed fields are rejected. SHA256 covers the compact JSON of the parsed profile.

The supervisor accepts newline-delimited JSON on a private Unix socket. Each
request includes integer v=1, a nonempty id (at most64 characters) and op. Requests
are at most16KiB; buffers are bounded, with at most8 local-user connections.
Every response returns v/id/ok plus status or an error. Mutations retain at most64
request IDs per supervisor instance; replaying the same ID/payload returns its
original response, and changing payload under an existing ID is rejected.

Operations: status, attach_manager(config_file), start(profile), close(session), workspace
(session/token/status), show_manager and exit_service. start accepts no executable,
shell fragment, environment overrides or PID. The desktop executable is resolved
from the installed ament package index. The workspace token correlates owned
sessions; private directory ownership and Unix peer credentials restrict access.
Tokens are not emitted in status/logging.

The manager's file selection is retained for Return to connection manager, even
if that window was closed. The service menu can reconnect the local manager or
exit an idle service; it cannot terminate a running workspace.

Phases: idle, starting, running, stopped, failed. A running OS process and a fresh
workspace heartbeat (under2.5s old) are independent. Missing heartbeat means
unknown observation status; it never triggers a second process or automatic kill.
start rejects any live old process. close requires the exact active session,
fresh status and unresolved=false; acceptance is only a request for the original
window's normal close handling, not proof of process exit. Only QProcess completion
establishes stopped/failed. No task cancellation is sent.

Changing a connection creates a new complete desktop process. All SDK/Observer,
mutable-port factories (disabled in this batch), RViz contexts and tools inherit
one environment. Stale inherited DDS profile/server variables are cleared for
the supported default-discovery preset. Security settings are not disabled.
Actual optional library dependencies are still those of the installed desktop.

The original launch session lock is not acquired in CM-1 because no backend is
started. The supervisor uses its own flock. GUI close does not release a live
workspace. A supervisor crash loses management ownership: new instances do not
adopt PIDs, and no old workspace is killed or reidentified from a PID file.

Pending/dirty local drafts use the old workspace confirmation flow. Task/import/
parameter unresolved states prevent managed close. The default CM-1 workspace
does not create these mutable ports; manually adding such CLI flags is rejected.
Default standalone desktop behavior remains available.
