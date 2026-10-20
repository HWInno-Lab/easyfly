# OpenVINS 接入

本包提供显式启动入口、实时相机/IMU 输入守卫及定位状态。上游计算保持在
`third_party/algorithms/localization/open_vins`，通过已安装的 ROS 节点消费。
默认 FAST-LIO 与现有飞行链不自动切换。

```bash
ros2 launch easyfly_openvins_integration openvins.launch.py sensor_profile:=gazebo_mono
```

输入为 `/camera/color/image_raw`、对应 CameraInfo 和 `/livox/imu`，必须使用
仿真源时间和本仓库修正后的 `livox_imu_frame`。已有 rosbag 可通过
`ros2 bag play BAG --clock 100 --rate 0.5 --delay 2` 回放；应在两个节点订阅就绪后启动。
发布端应提供 RELIABLE QoS。本入口不启动 Gazebo、不发送飞行指令。

输出为 `/localization/openvins/pose` 和 `/localization/openvins/status`。
位姿位于 `openvins_odom` 的局部惯性参考系，跟踪点为 IMU 原点；本包不发布 `map`、
`odom`、`base_link` 的 TF，也不覆盖飞控 `/odom`。速度和视觉跟踪质量未在此适配器中提供。
状态与时间契约见 [CONTRACT.md](CONTRACT.md)。

Ceres 为必需依赖，构建需与 ROS `cv_bridge` 使用同一 OpenCV。可使用系统
`libceres-dev`，或显式传入私有前缀并加载其 `usr/lib` 与 `usr/lib/x86_64-linux-gnu`。
禁止依赖本机旧的 OpenVINS 安装替代本次构建结果。原始许可证随三个上游包安装。
配置来源及不同文件的许可见 `config/NOTICE.md`；本包的双许可证元数据不改变各文件原通知。

当前安装配置仅用于 Gazebo 单目。D435i 的采集参数另见 `easyfly_params/config/sensors/`；
设备专属标定、双目输入守卫和实机精度验证完成后才能增加对应配置。
生产导航选择、界面接线和飞行准入仍需独立验证。


## Managed simulation selection

The connection UI now offers `OpenVINS（Gazebo 单目，仅观测）`. The equivalent
managed profile argument is `algorithms_json:='{"slam":"openvins"}'` with
`profile:=full_perception` (or `navigation_dev` / `landing_dev`). The selected
branch starts this launch instead of FAST-LIO. It requires installed optional
`ov_msckf` and this package; the default FAST-LIO branch has no OpenVINS lookup.

This is observation-only integration: the public navigation coordinator rejects
goals, private planners and the tracker are omitted, and the desktop blocks
navigation/tracking/search mission confirmation. Fresh ESTIMATING status does
not remove this restriction. Body conversion and alignment observations are
available. Local full-stack input/lifecycle regressions are recorded in
`docs/development/openvins-transport-fix.md`; map integration and complete
managed-flight validation are still pending.

受管 OpenVINS 分支会自动选择同步渲染、本机共享内存 DDS 策略及较低优先级的
检测/跟踪/像素转换进程。调用者显式提供的 DDS XML 和 `LP_NUM_THREADS` 优先。
FAST-LIO 默认分支不启用这些 OpenVINS 策略。

受管启动先运行只读 `openvins_startup`：至少观察 14 秒（覆盖剩余定时启动组），
要求节点图连续 5 秒稳定且输入按原守卫门限连续 2 秒健康，总等待上限 60 秒。
它复用守卫的布局、标定、源时间与依赖检查，不发布传感器或定位输出；启动期坏窗口
会记录原因，永久配置错误或等待超时会终止启动。就绪后仅启动一次守卫、算法和对齐器，
运行期故障仍按原契约锁存，不自动重建会话。独立回放入口默认不等待；
`wait_for_inputs:=true` 可启用相同检查。受管分层诊断使用
`openvins_wait_for_inputs:=false` 保持原始启动时间，不能将它当成就绪检查通过。


## Body and source-time aligned observations

Managed Gazebo now includes `openvins_alignment` in the same process lifetime.
Independent callers opt in with `start_alignment:=true`. It publishes local body
pose/covariance, an observation-only `odom` pose and a separate alignment status;
see CONTRACT.md for the JPL covariance conversion, temporal pairing, bounded
initialization and session failure rules. No global TF or flight source changes.

Gazebo OpenVINS selects MAVROS PASSTHROUGH source stamps. The observer verifies
that policy through read-only parameters before accepting references. Simulation
NONE mode uses ROS receipt time and is explicitly insufficient for alignment;
FAST-LIO keeps its original policy. The profile is not a D435i real-device config.

For raw/guarded input evidence, run `tools/algorithms/probes/openvins_input_trace.py`
in the same ROS domain with `--output DIRECTORY --seconds 60`. It observes only,
records source/receipt gaps, image/IMU field digests, CameraInfo and guard status,
and does not launch or command the stack. DDS reader histories are bounded;
graph-reported depth 0 is unknown, not the actual configured writer depth.
The probe also has an independent best-effort IMU observer to distinguish
delivery delays from lost samples; the guard's reliable subscription is unchanged.
Producer-side timing and the experimental native Fast DDS policy verifier are
documented under `easyfly_gazebo_plugins/safe_imu` and
`tools/algorithms/probes/fastdds_imu_profile/README.md`.
