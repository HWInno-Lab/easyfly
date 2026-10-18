# Point cloud filtering

The node filters Livox samples by range and voxel size, then transforms them to
the configured output frame at the original measurement timestamp. It never
substitutes the latest vehicle pose for a missing transform.

A queue of at most ten scans waits for TF without blocking the executor. Scans
expire after one wall-clock second, even when simulation time is frozen. Zero
timestamps, duplicates, data older than one ROS second, and timestamps more than
0.2 ROS seconds in the future are rejected. A backward ROS clock jump clears
pending scans and permits the new epoch. Static transforms remain valid through
the normal timestamped TF lookup. Output retains the measurement timestamp.

Missing or expired TF produces no cloud; downstream map freshness checks remain
responsible for invalidating unavailable input. Filtering distances, voxel size,
obstacle inflation, navigation admission and arrival thresholds are unchanged.

The ordinary ROS regression starts only this node and publishes synthetic clock,
TF and cloud messages. It checks delayed rotation, frozen-clock expiry, recovery,
invalid timestamps and clock rewind; it never starts a simulator or sends flight
goals.

ALG-4 moves range/voxel computation into the standard-library-only
`easyfly_pointcloud_filters` package. The node directly consumes its `Pt` and
filter functions; the old `filter_core.hpp` contains forwarding aliases only.
Parsing, source-time TF, queue/watermark/clock handling and PointCloud2 layout
remain in this node. Filtering happens in the source frame before the original
measurement-time transform. No PCL, Eigen or ROS runtime dependency is added to
the algorithm package; Eigen/TF remain node dependencies.
