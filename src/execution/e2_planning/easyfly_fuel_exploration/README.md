# FUEL 探索任务

`/explore` 为显式、有界的探索任务。默认使用 FUEL 原生前沿、LKH 访问顺序和视点选择，再由 GCOPTER 执行导航；详细历史验收见 `docs/development/fuel-simulation-acceptance.md`。

## 扩展仿真模式

2026-10-29：扩展模式通过声明范围内的软件／原生仿真验收。长距离任务完成 8 个相机视点、90.63% 覆盖率、返航和落地，实测峰值 1.018 m/s，源时间对齐的 PX4 状态最大跟踪误差 0.175 m。动态障碍任务实际重规划 1 次、覆盖率 99.29%；相机冻结、精确取消、所有权释放和正常退出通过。默认 FUEL 完整任务回归通过。签收及失败记录见 `artifacts/stages1-3-20261028/fuel-extended-signoff.json`。

```bash
ros2 launch easyfly_sim_launcher profile.launch.py \
  profile:=navigation_dev headless:=true rviz:=false \
  'algorithms_json:={"slam":"fast_lio","strategy":"gcopter","exploration":"fuel","provider":"cpu"}' \
  fuel_extended_profile:=true fuel_native_max_speed:=1.2 \
  fuel_evidence_directory:=/absolute/new/fuel \
  gcopter_evidence_directory:=/absolute/new/motion
```

启动后保持空闲，起飞、Explore 和最终降落仍由调用方明确提交。`fuel_extended_profile` 默认关闭；`native_max_speed` 是 0.1–2.0 m/s 的请求上限，轨迹还受已有的加速度、加加速度、跟踪误差和地图检查约束。实际速度以飞行测量为准。FUEL 扩展使用固定地图，不与独立 GCOPTER 的滚动地图参数叠加。

扩展模式执行固定版本 FUEL 的 `BsplineOptimizer` 与 `NonUniformBspline`。原生三次 B-spline 通过精确代数转换进入现有多项式消息与 FollowTrajectory 通道；保存原始控制点、结点和候选轨迹，独立比较 de Boor 的位置、速度和加速度。固定起止边界参加原生求解，保留实测初速度。每条曲线均须通过既有连续碰撞、C2、终态及动力学检查。

动态地图仅由实际新射线更新。移走的障碍需要连续 3 个不同源扫描的有效空闲观测才清除；未观测、无效或重复数据不能清除障碍。未知空间按完整机体／跟踪包络保守扩展，FUEL 视点选择和轨迹执行使用同一规则。新增障碍阻断剩余轨迹时，先停止原子动作并确认 STOP，再在同一父任务期限和重规划预算内重新求解。

扩展模式需要新鲜的 `/camera/color/image_raw`、对应时间戳的 `/camera/color/camera_info`、`base_link <- camera_color_optical_frame` 与 `odom <- camera_color_optical_frame`。支持固定前视光学坐标；实际平移由 TF 读取。相机内参、外参和 DDS 来源固定到本次运行；变更、失效、时钟回退或停流停止任务。视场由实际内参推导并预留角度余量。地图仍来自原生雷达观测，覆盖率仍按真实已观测体素计算。

每个视点通过 `/planning/fuel_viewpoint` 精确绑定父任务 UUID、子导航 UUID、位置与航向；绑定本身不产生运动。航向平滑地包含在同一个 FollowTrajectory 控制流中。到位后需要实际机体航向、源时间相机帧以及原生可见前沿在真实光学 TF 下的投影检查全部通过，才计为完成视点。原始 Image／CameraInfo CDR 和校验值保留在任务证据目录。

超时未决的绑定或导航请求持续保留所有权；确认原请求、取消相应动作、确认 RELEASED 后才清除绑定和父租约。恢复输入后需要显式提交新任务。B-01 的历史取消后重启原因仍按用户要求暂缓。

受管扩展模式在调用方未指定 DDS 策略时启用已验证的大图像共享内存配置；显式 DDS 配置保持调用方选择。同机原始图像生产者和消费者都需要足够的消息／共享内存容量。

软件飞行验收使用 Gazebo 真实物理射线与 PX4 动力学，显式配置球面扫描、3.5 m 量程和 0.6 m 机体附近排除区，以验证完整未知空间包络。它不代表 MID360 实物覆盖范围。普通稀疏或有盲区的扫描可能被保守包络拒绝；此时需提供有效观测，不应放宽未知空间或碰撞门限。实机不在此轮验收内。

## 构建依赖

新增原生轨迹计算使用 NLopt 2.7.1；Ubuntu 22.04 对应开发包为 `libnlopt-cxx-dev`，运行共享库为 `libnlopt0`。本轮私有构建记录、原版权说明和 LGPL 文本位于 `src/third_party/algorithms/planning/fuel.notices/NLopt-*`。FUEL 固定上游原件、零模糊补丁与文件指纹位于相邻的 `fuel.provenance.json`；既有 D-04 许可材料缺口保持暂缓。
