# Configuration provenance

`gazebo_mono/estimator_config.yaml`, `estimator_config_navigation.yaml` and
`kalibr_imu_chain.yaml` derive from the
OpenVINS EuRoC examples at official revision
`69488123ed9362dd44b6f28e7f4680abbff1442b`, with EasyFly simulation values and
guarded ROS topics. Upstream GPLv3-or-later notices apply to these derived
configurations; the original license is retained in `OPENVINS-LICENSE`.
`kalibr_imucam_chain.yaml` contains project SDF-derived simulation calibration.
EasyFly Python code and `guard.yaml` use Apache-2.0. No D435i device calibration
is inferred from these simulation parameters.

The observation profile shares the calibrated Gazebo gravity (9.8 m/s²),
initialization disparity (3 px), feature budget (200), and `NONE` histogram
processing with the navigation profile. This keeps sensor initialization
consistent; it does not enable OpenVINS dynamic initialization or certify
navigation accuracy under CPU pressure.
