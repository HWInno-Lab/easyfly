# Geographic map and aircraft observation

空间视图 now contains a 地理底图 tab. It is a read-only display and supplies no
navigation targets, flight commands or TF. It works without the optional RViz
and media features. SQLite3 and GeographicLib are required build dependencies;
exported CMake consumers discover them through `find_package(easyfly_desktop)`.

The bundled offline world land outline is Natural Earth 5.1.2, public domain,
1:110 million scale. It provides global context, not a survey or obstacle map.
The installed `assets/geography/NOTICE.md` and `provenance.json` retain the exact
source, license and SHA-256. No network request or online tile provider is used.

Use 导入离线底图 to copy a raster MBTiles file into the application cache.
Accepted files contain indexed `tiles` and `metadata` tables, PNG or JPEG tiles
of 256 or 512 pixels, zoom 0–19 and TMS row order. Supply `name`, `attribution`,
`format`, `minzoom` and `maxzoom`; optional `scheme` must be `tms`. Views, vector
PBF, absent attribution, duplicate tiles and oversized/invalid images are refused.
Files are limited to 512 MiB, cache total to 1 GiB. Import is asynchronous and
atomic; filenames contain SHA-256, and reimport verifies cached bytes. Reopen a
cached MBTiles file to use it after restarting. The import button tooltip shows
the cache path. Remove an unused cached file explicitly when the cache is full.
Missing tiles have a visible pattern; they are never fetched from the internet.
A single bounded worker reads at most 64 tiles for the current view.

GPS is shown only with valid, fresh WGS84 latitude/longitude. Arrow keys pan,
+/− and the wheel zoom, dragging pans, and 居中 GPS recenters the view. Web
Mercator cannot display positions beyond ±85.05112878° latitude.

局部坐标基准 is explicit and applies to the current observation session only.
Enter a surveyed correspondence: WGS84 latitude/longitude/ellipsoid height,
local XYZ, local +X counterclockwise angle from true east, and a short evidence
note. The local frame is taken from the current state. GeographicLib converts
WGS84 ECEF/ENU and the declared yaw/translation, with a 5 km local radius limit.
GPS altitude above mean sea level is never substituted for ellipsoid height.
No datum is inferred from one GPS fix, estimator position or vehicle heading.
The yellow local marker is distinct from the GPS marker. A connection/session
change, clock epoch change or disconnect clears the datum; stale data removes
live markers. This is a projection of the declared datum, not automatic alignment.

In an RViz-enabled build, manually enable 完整机模 in 内嵌 RViz. The project-authored
Apache-2.0 asset contains 23 components: body, arms, motors, propellers, landing
legs, camera and LiDAR. Geometry is a display approximation in metres and FLU
(+X nose, +Y left, +Z up), not a collision model or rotating-propeller simulation.
The original `/mavros/local_position/odom` or `/odom` pose (selected by the existing
preset) must identify `base_link`, have a finite unit quaternion and be no more
than 0.5 seconds old by both source and steady receipt clocks. Repeating the
same timestamp cannot renew freshness. Missing/competing sources, timestamp
rollback, conflicting samples, future samples and clock jumps hide the model.
Camera and LiDAR components additionally require `camera_link` / `livox_frame`
TF relative to `base_link` at the original pose timestamp, composed with that
original odometry body pose; a separate global TF body estimate is not used.
Missing transforms hide those components and
report the missing count. The display publishes no ROS topics or transforms.
Hardware geometry/calibration and flight accuracy are separate acceptance work.
