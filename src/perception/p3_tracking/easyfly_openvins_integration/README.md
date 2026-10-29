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

The default managed integration is observation-only: the public navigation coordinator rejects
goals, private planners and the tracker are omitted, and the desktop blocks
navigation/tracking/search mission confirmation. Fresh ESTIMATING status does
not remove this restriction. Body conversion and alignment observations are
available. Local full-stack input/lifecycle regressions are recorded in
`docs/development/openvins-transport-fix.md`; explicit navigation, map integration and qualified managed flights have since passed;
the desktop task entry is covered by the separate I-01 acceptance.

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

显式导航的新守卫会先收到新鲜、有效的 IMU 和相机标定，再创建图像订阅。
这是本进程的依赖就绪检查：先前只读观察进程的健康状态不能保证新 DDS 读者的
发现顺序。两项依赖到齐前不接收或缓存图像；订阅建立后的 0.5 秒超时、源时间、
配对和故障锁存规则不变，运行期失联不会重新进入启动等待。


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

## 显式 Gazebo 仿真导航

正常受管 `profile.launch.py` 的 `navigation_dev` 方案可显式传入
`openvins_navigation:=true`，独立入口对应
`start_alignment:=true enable_navigation:=true`。默认仍为观测模式，桌面尚不开放
导航任务；本轮约定仿真验收已通过，场景条件与完整证据见 `docs/development/openvins-simulation-acceptance.md`。
导航入口使用独立的 `estimator_config_navigation.yaml`，没有修改原观测配置。

导航在真实初始化、源时间对齐、视觉跟踪诊断与地图均就绪后才接纳请求。
`openvins_navigation` 发布 `/localization/openvins/navigation_state`、`/odom`，以及
在同一实测位姿时间发布的 `map→odom→base_link`。前一条边是本次局部地图的固定
坐标约定，不是 Gazebo 世界真值、全局地图复用或回环结果。规划/地图接收 VIO 状态；
PX4 内部控制继续使用自身估计，本实现没有接入 PX4 外部视觉融合。

原始状态、对齐状态、导航状态分别表示不同检查阶段，不能互相替代。上游新增的
`/openvins/raw/visual_tracking` 是带图像源时间的实际保留特征数；导航检查初始化、
至少 8 个保留特征的最近 150 ms 图像源时间窗口以及原有 500 ms 新鲜度。运行中零
特征立即锁存；持续弱跟踪或停流停止导航输出，协调器取消实际子动作并等待排空。
定位进程退出只结束本地 VIO 组，保留 PX4/控制链执行悬停和降落。

首次导航前先执行真实运动初始化和正常降落，在未解锁状态建立可信地面后再次起飞。
原始低纹理办公室不列为正向通过，办公室正向案例显式使用物理纹理板；森林和几何场景
使用原场景，绕障正向案例显式保持约 45° 观测朝向，未宣称任意视角通过。
实际时钟回退后须重启整套仿真；没有自动恢复旧状态或切换定位。
本轮没有验收 D435i 实机、目标机性能、PX4 外部视觉融合或桌面导航放行。

## 2026-10-28 桌面导航接入

连接管理选择 `navigation_dev` 与 OpenVINS 时显式传入
`openvins_navigation:=true`；其他方案保持观测模式。桌面导航、跟踪和搜索
共同要求原始状态与导航状态新鲜且属于同一 session/generation，后端继续检查
地图、飞控和任务所有权。实际 Qt 两段导航、独立真值评估、落地上锁和完整退出
已通过 I-01 验收。初始化仍需真实输入与必要的起飞／落地建图流程。
这更新了上文早期验收的“桌面未开放”边界；不扩大为 D435i 或任意场景验收。
