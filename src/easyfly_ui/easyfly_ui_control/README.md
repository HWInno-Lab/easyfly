# Independent managed mission control

Qt-independent library with an owned ROS context/worker. It is separate from
the read-only Observer/Source and CAN submit reviewed start/exact cancel RPCs.
No SDK flight actions, legacy-start fallback, setpoint publishers, mode/arm
commands, cancel-all or fault-clear interface exist. Desktop links this library
but creates a controller only with explicit managed-simulation opt-in and a
persistent journal directory; ordinary startup remains read-only.

Construct MissionControl with an explicit dedicated absolute journal directory
and response timeout (default2s, .05..30). No temporary storage default, implicit
home path or recursive directory creation; trailing separator/dot paths are
rejected to keep the durable parent-directory operation unambiguous.
The directory must be owned/private
(0700), with a nonblocking exclusive lock; one controller per directory. Domain
ID and localhost scope are bound into the journal; changing scope blocks restore.
This does not discover other controllers using different directories.

Directory creation/ownership and exclusive-lock acquisition are synchronous:
construction throws before any worker or ROS context on failure. The CLI prepares
one controller before opening a window, keeps that same held lock, and exits2
with a diagnostic for invalid/occupied startup directories. It never chmods,
clears or replaces an existing record. Let the library create mode0700 or
pre-create a private directory. Record decoding, scope/remote-state and runtime
failures remain conservative Blocked states with the existing unresolved exit
confirmation; a storage error is not remote-release evidence.

Construction sends reads only. A fresh session and reviewed valid_snapshot
(<=2 steady seconds, UTF-8 <=64KiB, exact SHA-256, shared full static parser) are
required for start_confirmed. The caller must supply the explicitly confirmed
server instance. The desktop additionally checks simulation/profile evidence and
requires explicit confirmation/exit choices; this library is NOT live flight admission.

Before start or cancel, the local intent is durably committed: owner-only
bounded strict schema-v1 JSON, integrity checksum, fsync(file), atomic rename,
fsync(directory). Open/read use dirfd, O_NOFOLLOW and regular-file checks. Lock,
schema, checksum, scope or write failures block control. The checksum detects
accidental corruption, not authentication or malicious edits by the local owner.
Only one unresolved intent is retained. Confirmed resolved records can be
replaced by a new explicitly confirmed intent; there is no unresolved-record
eviction, automatic clearing or public force-unlock operation.

Lost replies and restart recovery use only lookup of the original instance and
request. Never resend start. A cancel is journaled as dispatched before sending;
no automatic retry, including across restart. A queued-but-unsent operation may
therefore conservatively remain unresolved if shutdown/write/transport fails.
New-instance/unknown/malformed replies retain a fault lock; no inference from
empty IDs, backend disappearance, telemetry or public terminal status.
Unavailable/timeout reads may retry at <=1Hz; wrong-identity faults do not.
Known original admission refusal or exact terminal plus ownership release can
resolve the intent. Cancel ACK is not terminal; execution_pending=true keeps
the lock even on succeeded/failed/canceled. Original admission is not overwritten
by cancel responses. cancel_confirmed targets only the recorded execution ID.

stop is nonblocking and only stops local communication. Poll snapshot()->stopped
before destroying on a GUI thread; the destructor joins the owned worker.
Stopping, closing or crashing never cancels remote actions or deletes intent.
Restore does not adopt someone else's execution or automatically change instance.
No cross-restart server recovery database is added.

The GUI supplies a random 32-hex confirmation ID for each currently open start
dialog. Its decision callback is one-shot and checks the originating dialog.
The controller consumes an accepted supplied ID at most once in this process,
even after the intent resolves. Up to256 IDs are retained without eviction;
new attributed starts are refused at the limit. A refused enqueue does not
consume its ID. Fresh explicit review/confirmation is required after failure;
there is still no automatic write retry. The three-argument library API remains
available to explicit non-GUI callers, recorded with confirmation=none.
Confirmation IDs are not journal/schema fields, wire credentials, or replay
authorization across restarts. Restore remains reads-only and never restores a
GUI confirmation or permit.

INFO logs under easyfly_managed_audit correlate dialog result/recheck, enqueue,
durable intent identity and transport attempt using confirmation ID, PID, steady
time, server instance, request ID and content hash. Transport sequence/node name
is recorded separately. Mission services log mutating entry points with domain,
request writer GUID/sequence, new-vs-known identity, execution and Action UUID;
session/lookup polling does not generate per-poll audit lines. No mission body,
catalog/journal paths, credentials or arbitrary upstream message text is logged.
Use an explicitly retained ROS_LOG_DIR when collecting evidence. These are
diagnostic logs, not an authenticated or fsync-backed audit database. A recorded
dialog Yes result does not itself prove a human gesture, successful transport
does not prove admission, and admission does not prove terminal ownership release.
Missing historical logs cannot be reconstructed from these additions.

Tests use temporary journals and mock reviewed services. The service package's
test_reviewed separately verifies the actual mission service with mock Actions,
including exact cancel identity, late admission and ownership drainage.
The desktop's test_managed_stack covers real Qt/controller/service integration
with mocked action ownership and fabricated observation evidence. None of these
checks establish SITL/PX4/Gazebo or real-flight acceptance.

## Independent, opt-in metadata writer

`parameters.hpp` adds a separate `ParameterControl`; constructing MissionControl
does not construct it. Desktop enables it only with `--enable-parameter-edit`.
It owns its own context/worker and one fixed QueryParams client, not a flight
SDK, publisher, arbitrary node parameter client or mission journal.

Only enum keys OperatorNote and ExportPretty exist. Values are UTF-8/no-NUL
text <=512bytes or a boolean, respectively. Strict scalar JSON decoding rejects
other types and trailing content. JSON `""` means an empty-note write; empty
request text means read. `export_pretty` affects platform JSON config export,
not YAML drafts. The frontend no-NUL restriction is stricter than the backend.

`read` is explicit, serial and clears stale review permission while pending.
`write_confirmed` requires the exact current review ID/key/canonical JSON and a
new32-hex confirmation ID. Before one application write, it independently reads
and compares old JSON. After the write reply it independently reads again.
Verified requires success=true acknowledgement and a matching typed readback.
Rejected acknowledgements are not promoted to success even if current==desired.
No request retry/rollback occurs. At most256 confirmation IDs are retained
without eviction; empty IDs are not accepted by this API.

Fault locks are sticky across reads/reconnect, including matching late values.
Post-dispatch timeout/transport failure, unresolved observation disconnect or
invalid/different readback keeps the lock. `inhibit(true)` stops new operations
and prevents an unfinished precheck from dispatching a write; if a write was
already dispatched, it keeps uncertainty even when later replies match. It
does not revoke or compensate a request already received by the backend.
`stop` is nonblocking; poll stopped before destroying on the GUI thread.

QueryParams has no server-instance/request identity or CAS. This is not atomic
concurrent editing, authenticated identity, exactly-once transport, a durable
intent store or cross-restart recovery. Service replacement and writes by other
clients can race the checks. The process-local fault/confirmation set is not
restored after restart; do not use restart to establish remote outcome.
Parameter tests use dedicated domain123. The desktop real-service test uses
domain124 without flight nodes; this is not production or flight acceptance.

## Durable create-only mission importer

`MissionImporter` is another separate opt-in metadata/file client, not mission
execution. It owns only ImportMission and GetMissionDetails clients, a context,
a worker and a dedicated import journal. There are no SDK/action/setpoint/mode
operations, directory paths supplied to a write endpoint, overwrite, cancel,
rollback, replay or force-unlock APIs. It requires ROS_LOCALHOST_ONLY=1 and an
explicit persistent private journal path. Localhost discovery is not authentication.

ImportJournal reuses the existing private-directory/exclusive-lock acquisition
only, not flight-intent serialization. Its file is `import.json`, with strict
schema_version1/kind=catalog_import, scope/domain, instance, catalog, request,
name/hash, outcome and integrity checksum. It rejects an existing flight
intent.json and never loads/modifies it. Files are private/bounded/regular and
no-follow, writes fsync+atomic rename+directory fsync. A checksum is accidental
corruption detection, not a signature. Different journal paths do not provide
mutual exclusion; never change directories to bypass an unresolved import.

Fresh startup loads the local record but sends no automatic create. An unresolved
record restores by original lookup, never by adopting a new session. Wrong ROS
scope blocks before remote calls. Instance/catalog/request mismatch, not_found,
malformed outcome or mismatching file readback keeps a fault and unresolved record.
Unavailable/time-out reads poll at most1Hz; mutating create is submitted at most
once by the application. A retained original successful decision plus independent
details matching exact SHA-256, followed by the same live instance/catalog check,
is needed before durably resolving success. A matching retained refusal resolves
failure, not success. Resolved history is not a fresh current file observation.

New import needs an explicitly read destination and confirmed name/bytes/hash,
bound to one32-hex confirmation ID. Up to256 confirmation IDs are retained without
eviction in-process. After human confirmation, session is independently rechecked,
the shared v1 validator runs again, and intent is durably recorded before one
create. Inhibit/stop observed before dispatch prevents create; a durably prepared
but locally proven unsent intent can be durably resolved as not_dispatched.
After dispatch, closing stops local communication only and retains uncertainty.
Startup locking is synchronous before GUI construction; normal validation and
journal synchronization run outside the mutex used by GUI polls/inhibit/stop.

This is not exactly-once transport, server persistence, authentication, multi-user
editing, cross-restart server recovery, atomic readback/CAS, or a permanent catalog
integrity guarantee. Files may subsequently change; later execution must separately
review and bind current bytes. `stop` is nonblocking; wait for stopped before GUI
destruction. A hung filesystem can delay local worker completion, not authorize
forced rollback or infer remote outcome. Tests use domain128 mocks and isolated
temporary storage; GUI integration uses domain129 without flight nodes.
