# Mission inspection and reviewed execution protocol

`/platform/mission/manage` retains its existing list/validate/start/query/cancel
contract. The additive `/platform/mission/details` endpoint uses
`easyfly_msgs/srv/GetMissionDetails` and cannot execute or modify a task.

Supply a basename within the configured mission_directory, at most 128 bytes.
The existing canonical catalog/extension checks, regular-file limit (64KiB),
O_NOFOLLOW/O_NONBLOCK reading and shared static validator are reused. Success
returns valid_snapshot, the same requested name, original content and its
SHA-256. Failure returns a bounded error and empty content/digest. Reading does
not require action-server readiness or create a private execution snapshot.

The digest identifies returned bytes, not execution identity, authentication,
flight permission or a guarantee that a later start will read the same bytes.
The original text does not contain injected defaults; clients should use the
shared parser for normalized steps. Formal YAML schema v1 and legacy flat
content detection remain unchanged. No task profile field is introduced.

The existing start request does not carry an expected review hash. A client must
not claim this additive preview makes later execution atomically identical.
Likewise, a lost start reply may lose the service-generated execution_id;
inspection, empty catalog or telemetry cannot recover it or justify retry.

## Additive reviewed protocol

`/platform/mission/reviewed` uses `ManageReviewedMission`; the legacy interface
and read-only details interface are unchanged. This new endpoint CAN execute
missions. The observation bridge remains read-only. The desktop's separate
control facade calls it only after explicit managed-simulation opt-in; default
desktop startup remains read-only. There is no fallback to legacy start.

| Operation | Required identity | Behavior |
| --- | --- | --- |
| session | none | Read-only current server_instance_id and ledger capacity; no flight admission |
| start | current instance, client_request_id, basename, expected_content_sha256 | Bind a new intent or report the already retained intent; never resubmit a known request |
| lookup | current instance, client_request_id | Read-only recovery of the original intent and current execution state |
| cancel | current instance, request ID, original basename/hash, exact execution_id | Cancel only the execution bound to this retained request; never guess UUID or cancel-all |

Both IDs are 32 lowercase hex characters; callers generate a new request ID per
explicit intent. Expected SHA-256 is 64 lowercase hex characters from the
reviewed original bytes, not normalized YAML. The same validated in-memory
bytes are compared and copied into the private execution snapshot. A mismatch
returns content_changed before creating an execution or sending an Action.

The ledger reserves an intent before catalog validation/action submission.
Same key and same name/hash return the retained intent, even if the file later
changes, admission fails, the backend rejects, or the execution finishes.
Reuse with different name/hash returns request_conflict. Static errors, busy,
unsupported, content_changed and backend results are retained; fixing the
cause requires a new explicitly confirmed intent, not retrying the old key.
Malformed identity/field syntax, wrong instance and capacity refusal do not
reserve a record and never send a goal. A busy intent does not expose or bind
the execution ID of the task occupying the slot.

Default/max capacity is 256, optionally reduced to 1..256 by the read-only
startup parameter reviewed_request_capacity. No record is evicted during the
node lifetime. At capacity, new starts fail closed; session, lookup and known
duplicates remain available. Rejected requests also consume capacity.
Restart changes server_instance_id; old-instance requests are refused, not
replayed. There is no cross-restart recovery database. A mismatch, not_found,
node disappearance or empty execution ID is NOT evidence that an old child
has stopped. Never erase the caller's unresolved state or silently change the
instance on an old intent. Retained terminal executions are still queryable by
request ID even after the legacy 32-entry execution history is pruned.

The initial start reply reports admission; lookup and duplicate replies report
current execution state. success on a known-execution lookup only means the
query worked, not that the task succeeded. admission_resolved/success/status
preserve the original admission decision separately. The child action_goal_id
is empty until accepted and is distinct from execution_id, request ID and
server instance. feedback_valid uses the existing freshness checks.
execution_pending remains true after an Action result until exact ownership
RELEASED is confirmed. Admission timeout retains the existing late exact
cancellation policy; loss of a client reply does not change that policy.

Clients should send once and use lookup on a lost/uncertain response, not
automatically resend start. cancel verifies instance, request, original name/hash
and execution ID, then uses the existing exact child cancellation mechanism.
It cannot cancel a legacy execution lacking this managed request association.
Cancellation replies never overwrite the original admission decision; ACK is
not terminal, and late admission cancellation keeps the original timeout policy.
The legacy exact cancellation behavior is unchanged.

The independent easyfly_ui_control library now journals unresolved client intent
before start/cancel, restores by lookup only, and stops local communication
without cancelling remote tasks. The desktop now has opt-in simulation controls,
confirmation dialogs, profile/preflight evidence and unresolved-close prompts.
It does not expose standalone flight actions or real-hardware operation.
Server persistence across restart remains
unimplemented; a new instance never silently adopts an old client intent.

Run the isolated test_details fixture for exact-byte/digest checks, catalog
mutation, YAML and invalid/oversized/path/symlink/FIFO rejection. It starts the
real service with a temporary catalog and a rejecting mock action server only;
inspection must produce zero goal submissions. Existing service regressions
also use mock action/ownership providers, not PX4 or a flight simulation.
The test_reviewed fixture exercises the real reviewed service with mock Actions
and ownership: review binding, duplicates, discarded reply recovery, timeout
exact cancellation, rejection/failure retention, terminal ownership drain,
capacity, restart identity, invalid input, cancel identity and late admission
cancellation without rewriting the admission record. It does not run PX4/Gazebo.

## Request-source diagnostics

Mutating reviewed start/cancel and legacy start/cancel entries emit INFO identity
metadata, including server instance, PID/domain, request writer GUID/sequence,
request/content hash and (when available) execution/Action UUID. New reviewed
request registration and known-request reuse are distinguished. Session/lookup
polls do not emit per-poll audit records. Untrusted names, bodies, paths and
arbitrary messages are excluded; malformed identifiers are rendered as none.
GUIDs distinguish transport writers, not authenticated users; diagnostic logs
are not a durable authorization database. The wire contract, record capacity,
admission, cancellation and ownership rules are unchanged.

## Opt-in create-only task import

The additive `/platform/mission/import` (`ImportMission`) is a file/metadata
operation, not execution. Existing manage/details/reviewed contracts are unchanged.
Import defaults off. Explicit server startup uses:

```bash
ros2 run easyfly_mission_srvs mission_services --ros-args \
  -p enable_mission_import:=true -p mission_directory:=/absolute/private/catalog
```

The destination must already exist, be owned/private (no group/other bits), be
absolute with no symlink components, and be outside the source/install/package
catalog trees. No parent/directory creation or permission changes occur. Only
an empty directory or a previously marked catalog is accepted, never an existing
ordinary directory containing user files. An empty directory receives the private
`.easyfly-mission-catalog-v1` marker. The importer holds an exclusive marker lock;
a second importer of that same directory fails startup. Root/marker identity and
ownership are rechecked. This is a trusted same-user workflow, not authentication
or a hostile-owner filesystem sandbox.

`session` reads current instance/catalog IDs, enabled state, configured server
directory and capacity. `create` binds current IDs, a new32-lowercase-hex request
ID, a YAML basename, exact original SHA-256 and UTF-8 bytes <=64KiB. Only formal
v1 YAML approved by the shared validator is accepted, not legacy `.mission` data.
The same in-memory bytes are validated and written to a private0600 staging file,
fsynced, and atomically published with Linux renameat2(RENAME_NOREPLACE), then
the directory is fsynced. Existing files, symlinks, hardlinks and FIFOs are never
overwritten. No link/unlink replacement fallback exists. A post-publish durability
or catalog-identity failure remains `unconfirmed`, without rollback/removal.

`lookup` queries only the original instance/catalog/request. Successful duplicate
creates return the original retained decision without rewriting the file, even
if the file later changes. Reuse with a different name/hash/body conflicts.
`published/resolved` describe the original decision, not current file contents;
clients independently read details/hash and recheck the original instance before
confirming success. Imports create no Action/SDK goals and do not require flight
readiness. Existing reviewed execution still requires its own separate confirmation.

The in-memory import ledger is separate from execution intents. Startup-only
import_request_capacity is1..256, default256; records are never evicted and
recorded rejections also consume capacity. Syntactic/identity/direct capacity
refusals need not reserve a record; a delivered matching refusal may prove no
publication for that request, but a missing reply/unknown lookup does not.
Restart changes both instance and instance-bound catalog identity; old requests
are refused, never adopted/replayed. Server-side ledger persistence is not added.
The existing total directory-entry bound of256 also applies (including marker
and hidden leftovers). Interrupted staging files are not silently deleted on
restart; inspect them manually rather than guessing their history.

One import worker handles one active plus eight queued jobs. ROS callbacks only
enqueue/send replies; validation and filesystem synchronization do not run on
the execution/ownership/cancel executor. Queue overflow returns busy; clients must
not retry create automatically. Shutdown finishes an active handler and abandons
unstarted jobs, leaving missing outcomes uncertain. File I/O completion/shutdown
duration is not guaranteed under a hung disk. The desktop separately journals
intent before one create and recovers by lookup only; see its README.

Tests use a temporary private catalog in domain127, with a rejecting ExecuteMission
spy that must see zero goals. They verify bytes/hash/readback/list, no overwrite,
duplicates/conflicts, invalid input/startup roots, root replacement, capacity and
restart refusal. The import worker's blocked-handler test verifies bounded,
nonblocking enqueue independently of any flight node. None is SITL/flight acceptance.
