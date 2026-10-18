# Optional local QGroundControl AppImage

EasyFly does not vendor QGC source or the binary. The user-supplied file is
`src/third_party/QGC.AppImage`, ignored by Git (180816376 bytes).
Use the desktop file picker or `--qgc-path /absolute/path/QGC.AppImage`.
This directory contains integration documentation, not an executable lookup default.
Moving these documents does not move, download, update or start the selected program.

The supplied file was inspected locally: its embedded AppStream metadata and
binary identify **5.0.8**, with bundled Qt **6.8.3**. This is not a signature
or provenance verification. The inspected SHA-256 is:

`06969c67ef58ea063def0a8271447a1cc385438c4a7df36813315b4475146737`

Other files selected by the user are not assumed to have this version or hash.
The desktop's offline preview supports trusted Type-2 AppImages; its launcher
checks the static format and executable path before invoking bubblewrap.
QGC remains a separately licensed upstream program; its embedded project
metadata declares GPL-3.0. This does not change the license of EasyFly's own
code, and this repository does not redistribute the supplied binary.
Upstream: [QGroundControl](https://github.com/mavlink/qgroundcontrol).

The desktop advanced tool provides **offline isolated preview**. It is separate
from the connection manager's networked maintenance entry. Its sandbox has a private network, minimal devices, fresh
temporary settings and contained descendants. It uses the local X11 display
with software rendering and `--appimage-extract-and-run`, without FUSE.
Missing bubblewrap or unsupported namespaces produce a visible failure,
never a normal-network fallback. No real device or PX4/SITL is started.

The connection manager also provides explicitly enabled **networked maintenance**,
using the saved executable path and per-run settings. It can send vehicle commands
and does not have the offline tool's private-network isolation. Selecting a file
does not enable forwarding or start QGC. See the current
[connection workflow](../../../easyfly_ui/easyfly_connection_manager/README.md)
and [process contract](../../../easyfly_ui/easyfly_connection_manager/CONTRACT.md)
for entry conditions, endpoint selection and normal window close behavior.
Those runtime implementations remain in EasyFly's UI packages.
Starting QGC directly outside EasyFly is not subject to the offline isolation.

ALG-8.4 only relocates integration documents and updates references. Local binary
identity and the limits of existing evidence are recorded in
[provenance.json](provenance.json); relocation validation is documented in
[ALG-8.4](../../../algorithms/alg8-4-validation.md).
