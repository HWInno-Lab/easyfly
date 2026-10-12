# Optional local QGroundControl AppImage

EasyFly does not vendor QGC source or the binary. The user-supplied file is
`src/third_party/QGC.AppImage`, ignored by Git (180816376 bytes).
Use the desktop file picker or `--qgc-path /absolute/path/QGC.AppImage`.
No automatic download, startup, update or vehicle connection occurs.

The supplied file was inspected locally: its embedded AppStream metadata and
binary identify **5.0.8**, with bundled Qt **6.8.3**. This is not a signature
or provenance verification. The inspected SHA-256 is:

`06969c67ef58ea063def0a8271447a1cc385438c4a7df36813315b4475146737`

Other files selected by the user are not assumed to have this version or hash.
Only trusted Type-2 AppImages are supported; the launcher checks the static
format and executable path before invoking bubblewrap.
QGC remains a separately licensed upstream program; its embedded project
metadata declares GPL-3.0. This does not change the license of EasyFly's own
code, and this repository does not redistribute the supplied binary.
Upstream: [QGroundControl](https://github.com/mavlink/qgroundcontrol).

The first EasyFly integration is **offline isolated preview**, not read-only
connected QGC. The sandbox has a private network, minimal devices, fresh
temporary settings and contained descendants. It uses the local X11 display
with software rendering and `--appimage-extract-and-run`, without FUSE.
Missing bubblewrap or unsupported namespaces produce a visible failure,
never a normal-network fallback. No real device or PX4/SITL is started.

Ordinary QGC networking, MAVROS/GCS endpoint coexistence, heartbeats and flight
failsafes require a separate approved integration stage. Starting QGC directly
outside EasyFly is not subject to this isolation.
