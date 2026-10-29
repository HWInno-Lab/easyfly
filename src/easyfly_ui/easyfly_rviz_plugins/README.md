# EasyFly RViz odometry lifetime

This read-only plugin subclasses the installed RViz 11.2.26 odometry display.
It retains the original topic/QoS, frame transforms, properties, Keep limit,
arrows, axes and covariance behavior. No system RViz library is replaced.

That version creates an additional parent scene node for each arrow, axes and
covariance visual. The visual creates and destroys its own child, leaving the
extra empty parent in the display scene. Covariance retirement also leaves its
empty orientation root detached from the scene tree. The plugin observes node
lifetimes in its own subtree during native update/reset, then destroys only surviving empty
nodes that became detached or remain direct children of its own scene node.
Temporary Ogre listeners invalidate deleted pointers, forward any existing
listener callbacks, and restore those listeners on surviving nodes. No node name
or raw pointer is assumed to survive native deletion. It never scans or deletes
another display's detached nodes. Parents containing
live visuals remain intact. Both embedded and external EasyFly
views use `easyfly_rviz_plugins/Odometry`.

The native ownership contract is pinned to
[OdometryDisplay 11.2.26](https://github.com/ros2/rviz/blob/11.2.26/rviz_default_plugins/src/rviz_default_plugins/displays/odometry/odometry_display.cpp),
[Axes](https://github.com/ros2/rviz/blob/11.2.26/rviz_rendering/src/rviz_rendering/objects/axes.cpp),
[Arrow](https://github.com/ros2/rviz/blob/11.2.26/rviz_rendering/src/rviz_rendering/objects/arrow.cpp)
and [CovarianceVisual](https://github.com/ros2/rviz/blob/11.2.26/rviz_rendering/src/rviz_rendering/objects/covariance_visual.cpp).
The graphics regression in `easyfly_desktop/test/test_rviz_view.cpp` measures the
actual Ogre scene tree across repeated ROS odometry updates. Separate native
plugin loading, heap diagnosis and whole-product hour evidence are retained under
`artifacts/v01-v16-20261025/`; source preparation alone is not acceptance.
