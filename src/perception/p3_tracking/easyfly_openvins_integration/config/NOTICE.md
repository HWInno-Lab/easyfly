# Configuration provenance

`gazebo_mono/estimator_config.yaml` and `kalibr_imu_chain.yaml` derive from the
OpenVINS EuRoC examples at official revision
`69488123ed9362dd44b6f28e7f4680abbff1442b`, with EasyFly simulation values and
guarded ROS topics. Upstream GPLv3-or-later notices apply to these derived
configurations; the original license is retained in `OPENVINS-LICENSE`.
`kalibr_imucam_chain.yaml` contains project SDF-derived simulation calibration.
EasyFly Python code and `guard.yaml` use Apache-2.0. No D435i device calibration
is inferred from these simulation parameters.
