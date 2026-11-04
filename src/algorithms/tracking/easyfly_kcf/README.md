<!--
  SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
  SPDX-FileCopyrightText: 2026 贾国源
  License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_kcf/licenses/.
  This revision offers no Apache-2.0 option for the listed original code.
-->
# easyfly_kcf

Project-maintained adapter for OpenCV 4 `cv::TrackerKCF`, implementing
`easyfly::single_tracking::Tracker`. Installed target: `easyfly_kcf::easyfly_kcf`;
header: `easyfly_kcf/tracker.hpp`. No ROS, Python, model weights or GPU required.

`Parameters::detect_thresh` defaults to 0.5 and accepts finite values in [0, 1].
It maps to OpenCV's correlation response threshold; it is not a calibrated
probability. Other native settings retain OpenCV defaults. Results deliberately
leave `calibrated_confidence` empty.

Initialize with BGR8 and an in-bounds pixel ROI at least 8x8. KCF's first native
update trains the model, so initialization performs that update on the exact
initialization image. Later frames must keep the same image dimensions. Invalid
updates, native exceptions, out-of-bounds output and tracking loss return an
invalid empty result and release history; recovery requires explicit initialize.
Reset or failed reinitialization also releases all previous history. Calls are
serial and synchronous; no borrowed image is retained by this adapter.

The algorithm is supplied by the installed OpenCV tracking module; upstream
source is not copied into this package. The wrapper is Apache-2.0. OpenCV and
its components retain their upstream notices and licences. Reference API:
[OpenCV 4.5.4 KCF](https://docs.opencv.org/4.5.4/d2/dff/classcv_1_1TrackerKCF.html);
reviewed implementation:
[opencv_contrib 4.5.4 trackerKCF.cpp](https://github.com/opencv/opencv_contrib/blob/4.5.4/modules/tracking/src/trackerKCF.cpp).
Other OpenCV versions require their own regression run.

## Platform use

Load an installation containing KCF and the platform factory. CSRT remains the
default; select KCF explicitly:

```bash
ros2 launch easyfly_single_object_tracking single_object_tracking.launch.py \
  tracker_backend:=kcf 'backend_parameters:={"detect_thresh":0.5}' \
  image_topic:=/camera/color/image_raw perception_version:=2
```

Provide fresh images and initialize an ROI through the matching V2 service using
that displayed image's header. Starting a node does not initialize a target.
The node converts mono8/RGB8 to BGR8; the pure adapter accepts BGR8 only.
The existing desktop single-tracker selector obtains KCF from the common registry.

See the [integration guide](../../docs/integration-guide.md) for registration,
configuration and acceptance, and [build conventions](../../docs/build-conventions.md)
for exported targets and installed consumption.
