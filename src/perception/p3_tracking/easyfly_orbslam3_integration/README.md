# ORB-SLAM3 平台适配

本包使用固定源码构建的 `easyfly_orbslam3_core`，提供 Gazebo RGB-D 定位、
机体坐标对齐、定位状态，以及显式启用的受管仿真导航。默认只观察。
RGB-D+IMU 支持显式选择、经标定核验的旧图合并和条件协方差；E-06 的声明仿真范围已验收。
两种安装方式完成新地图保存、冷启动复用、地面激活、导航、实际相机中断和正常退出。
详见[签收及边界](../../../../artifacts/stages1-3-20261028/orb-inertial-signoff.json)：
辅助建图采用显式球面扫描仿真配置；条件协方差不包含全局地图和标定误差。
D435i、其他相机标定和真实机载环境仍需硬件验收。

## 安装依赖

运行需要 `easyfly_orbslam3_core`、本包和 `easyfly_openvins_integration` 的安装前缀，
以及固定 Pangolin、OpenCV、Eigen、Boost、ROS 2 Humble 依赖。引擎安装包含词典与许可文本。
Pangolin 使用源码构建的独立安装前缀；构建时将它加入 `CMAKE_PREFIX_PATH`，
运行时使其 `lib` 位于 `LD_LIBRARY_PATH`。不要用来源未知的预编译引擎替换。

源码来源及补丁：`src/third_party/algorithms/localization/orb_slam3.provenance.json`。
纯安装消费者检查：`tools/algorithms/verify_orbslam3_install.py --help`。
这些源码工具可选；正常启动使用安装文件，不需要本机历史 artifacts 目录。

## 显式启动

独立启动只接入已有传感器和参考状态，不启动飞控或 Gazebo。为每次会话创建新的私有目录：

```sh
orb_session_dir="$(mktemp -d /tmp/easyfly-orb.XXXXXX)"
ros2 launch easyfly_orbslam3_integration orbslam3.launch.py \
  runtime_directory:="$orb_session_dir"
```

需要保存时显式指定 `save_atlas:="$orb_session_dir/atlas"`；输出前缀必须为绝对路径，
不得覆盖已有 `.osa` 文件。加载已有 RGB-D 图谱时传 `load_atlas:=/absolute/path/atlas`
（不带 `.osa`），同时使用新的运行目录。惯性模式另要求配套 `.easyfly.yaml`，
核对文件、词典、配置摘要和真实旧关键帧标定，并完成新会话实际惯性初始化及原生地图合并。
正常退出后等待全部算法线程结束；保存完成之前不要删除运行目录。

桌面连接管理选择 `navigation_dev`、`orbslam3`，并显式开启该后端导航仿真。
管理器创建会话目录；进入任务执行还需健康且同一会话的原始／导航定位状态、地图和飞行准入。
受管 CLI 使用 `easyfly_sim_launcher profile.launch.py`，显式设置
`orbslam3_navigation:=true`、新的 `orbslam3_runtime_directory`，以及
`algorithms_json:='{"slam":"orbslam3","strategy":"combined","provider":"cpu"}'`。
其他必填 PX4／场景输入见启动器的安装 README；配置或连接成功不会自动提交飞行任务。

## 输入和失效处理

RGB：`/camera/color/image_raw`；数值深度：
`/camera/aligned_depth_to_color/depth/image_raw`，必须为米制 `32FC1`。
`/camera/aligned_depth_to_color/image_raw` 是渲染图像，不能替代数值深度。
RGB／深度原时间戳配对差至多 1 ms；标定、光学轴、机体外参与队列有界性均受检查。

输出主题位于 `/localization/orbslam3/`，包含 `pose`、`status`、对齐状态和可选导航状态。
RGB-D 的 `covariance_available=false` 表示协方差未知。惯性原始位姿提供真实原生优化的条件
协方差，排除地图／全局规范自由度／标定不确定性；对齐输出仍标记未知，不直接供外部视觉融合。
重定位、回环／合并或地图重置改变已经激活的坐标代次后，
公开位姿锁存失效，导航不再沿用旧会话；先结束旧会话、核查输入，再显式建立新会话。
不自动重启任务或沿用旧导航控制权。

惯性观测显式使用 `sensor_mode:=rgbd_imu`，自动选择本包 `config/gazebo_rgbd_imu.yaml`，
输入为原始 `/livox/imu`。完整惯性初始化包含需要实际运动的多个阶段，初始建图不等于定位可用。
该配置与 `IMU.fastInit` 策略仅对已验 Gazebo 初始化航段成立，不能套用为实机标定。

受管 CLI 对应 `orbslam3_sensor_mode:=rgbd_imu`。配合 `orbslam3_navigation:=true` 后，
公开导航位姿还需显式激活：完成初始化运动，等待状态原因为
`ready_for_explicit_inertial_navigation_activation`，正常落地上锁后调用
`/localization/orbslam3/activate`（`std_srvs/srv/Trigger`）。等待地面模型和地图就绪后
才重新提交起飞／导航任务；服务不发飞行命令，空中首次激活会被拒绝。
任何激活后的地图修正或输入失效都会锁存停止；恢复输入不自动续飞。

## 验证范围

RGB-D 已完成实际建图、旧图重定位、A*／EGO 导航与真实 Qt 受管任务；
RGB-D+IMU 已完成指定初始化航段后的现场回环／位姿冻结，低激励失败记录仍保留。
全量验收索引在 `easyfly_sim_launcher` 安装的 `ACCEPTANCE.md`；原始证据作为独立归档交付。
构建／本包软件回归：

```sh
colcon build --base-paths src --packages-up-to easyfly_orbslam3_integration
colcon test --packages-select easyfly_orbslam3_integration easyfly_openvins_integration
colcon test-result --verbose
```
