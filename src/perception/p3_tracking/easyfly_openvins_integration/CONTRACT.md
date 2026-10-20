# OpenVINS 定位输入与状态契约 v0.3

`LocalizationStatus` 是 ROS 接入契约。`backend_id=openvins`，每次启动生成新的
`session_id`；不可恢复故障使 `generation` 递增并锁存失效。接纳消费者必须同时核对
会话、代次、源时间、参考系及 `pose_valid`，不能因最近收到过一个位姿就继续认为有效。

## 输入接纳

- 只接纳已知图像布局、光学/IMU 坐标系及与安装标定一致的 CameraInfo。
- 原图像、IMU 消息及源时间戳原样转发，不用接收时间或 `/clock` 替换测量时间。
- IMU 必须严格递增且无超限源间隔；初始化前也不把 NaN、错系或零时间发送上游。
- 图像暂存到观察到更大的图像源时间，且对应 CameraInfo 与前后 IMU 时间范围可用。
  暂存期间的重复时间整组拒绝。最多保留四个待定图像、64 个 CameraInfo 时间及
  1000 个 IMU 时间点；DDS 队列另由固定 QoS 限制。
- 四帧待定队列满时，用独立回调组暂缓从图像 DDS 队列取消息，让 IMU、CameraInfo
  和时钟回调继续运行；不直接把可恢复的调度积压判作队列溢出。生产入口显式使用
  单线程执行器，保持守卫状态及图像组的串行访问。图像 DDS 队列上限仍为 32。
  已封组图像在缺少依赖时，也由单调时钟定时器在原有 0.5 秒期限后锁存
  `pending_camera_expired`，不依赖后续新消息来触发故障。最后未封组帧停流仍报 STALE。
  仿真 RGB 与 CameraInfo 发布历史均为 keep_last(32)，不改变采样或时间戳。
- 最后一张图像保持待定，直到更大源时间到来，不因停流强行放行。已发图像再出现
  迟到重复/倒退时，无法撤回已发观测，必须锁存 `RESET_REQUIRED`。
- 大间隔、IMU 重复/倒退、时钟回退或队列溢出均锁存失效。守卫停止输入及公共位姿输出，
  不能只清空守卫而沿用上游缓冲。必须重启整个 launch，创建新算法实例和新会话。
  尚无 IMU 放行、也未接纳图像时间时，允许建立仿真时钟的启动重基准；此时算法尚无测量历史。
- 仅接收时间超时会使状态 `STALE`。源时间仍连续时，恢复后的新输入和新输出可恢复有效；
  若源时间有超限缺口，必须整会话重启。所有配置启动后只读。

## 输出语义

`/localization/openvins/pose` 为 IMU 在该后端局部惯性参考系中的位姿，
`header.frame_id=openvins_odom`，源时间及上游有效协方差保留。
这只是给上游 `global` 参考系命名以隔离后端，没有进行新的地图对齐。
不将 IMU 原点伪装为机体原点，不发布默认 `map/odom/base_link` TF。

守卫核验输出时间、有限数、单位四元数及协方差对称/半正定性后才发布。
`velocity_valid=false`、`tracking_quality_available=false`：没有从输出流存在推断速度或视觉跟踪质量。

| 状态 | 含义 |
| --- | --- |
| WAITING_INPUT | 尚无有效时钟及可配对输入 |
| INITIALIZING | 输入有效，尚未观察到算法有效位姿 |
| ESTIMATING | 输入与有效位姿均新鲜；不等于视觉跟踪质量已知或飞行可用 |
| STALE | 输入或输出接收/源时间超时，`pose_valid=false` |
| RESET_REQUIRED | 时间/队列连续性失效，必须重启整会话 |
| FAILED | 标定、布局或算法输出数值不合法，修复配置后重启 |

状态以单调时钟驱动的定时器发布，仿真 `/clock` 暂停时也能报停流。
状态话题可靠、保留最后一条；位姿不保留。订阅者仍须自查状态接收超时以发现整个进程退出。
`pose_stamp` 对应最近接纳的位姿，不能将状态 header 当成新的测量时间。
计数器与 `last_rejection` 保留歧义帧等输入问题，正常输出不会隐藏剔除记录。

任一子进程退出会关闭另一个子进程。没有自动回退、热切换、重定位跳变消除或飞行准入。
此接口为后续导航/界面消费者提供状态；当前默认控制链仍使用既有数据源。

受管 Gazebo 在会话创建前运行有界只读就绪检查。此时没有守卫 session_id、
状态发布、传感器转发或算法实例；不应将启动观察当作有效定位会话。
启动观察使用同一 Gate/标定/数值检查，坏窗口记录后可重新观察；配置失败直接退出，
总等待超过 60 秒也退出。至少观察 14 秒、节点图静默 5 秒且输入连续健康 2 秒后，
退出检查器并创建一次完整会话。此后所有故障规则保持原样，不因故障返回启动检查。
等待期间被中断不会启动子进程。单独 openvins.launch.py 默认不启用此仿真启动协调。


## 可选机体与 odom 观测（非导航准入）

独立入口默认为两进程；`start_alignment:=true` 将对齐器加入同一启动/退出会话。
受管 Gazebo 的 OpenVINS 分支显式启用它。三个进程任意一个退出均关闭整组。

- `/localization/openvins/base_pose`：`openvins_odom` 中 base_link 原点的
  `PoseWithCovarianceStamped`，保留原测量源时间。
- `/localization/openvins/aligned_pose`：`odom` 中机体的 `PoseStamped`。
- `/localization/openvins/alignment_status`：backend_id 为 `openvins_alignment`，
  tracked_frame 为 `base_link`，session_id 绑定本次守卫会话。消费者须核对这条状态
  的会话/代次/pose_stamp，不能用原始 IMU 状态代替对齐状态。
  base_pose 是中间诊断数据，初始化期间出现它不表示 aligned_pose 已有效。

上游姿态数值按 Hamilton 解释为 IMU 到局部世界旋转，但协方差直接来自 OpenVINS
`[delta_position_G, delta_theta_I]` 左乘 JPL 误差。不是已经处于世界轴的 ROS 姿态误差。
令 R=R_GI，t=T_IB.translation，机体位姿为 T_GB=T_GI*T_IB，协方差变换使用
J=[[I,-R*[t]x],[0,R]]。外参按固定标定值处理。机体局部协方差包含杆臂交叉项。

参考位置取 `/mavros/local_position/pose`，姿态单独取 `/mavros/imu/data`。
MAVROS local_position 消息中的姿态为缓存值，不能当成位置来源时刻的姿态；
对齐器先用 IMU 姿态的前后源时间插值到位置时刻，再建立参考位姿历史。
不使用按发布定时器刷新 header 的 `/odom` 或 `/uav_state`。MAVROS 的局部 `map` 数值与 EasyFly `odom`
沿用相同局部 ENU 数值约定；没有将 Gazebo 世界或其他 SLAM 的 map 设为恒等。
`/uav_state` 只提供连接/位置/姿态健康检查，不提供插值测量时间。

仅在本项目受管 Gazebo OpenVINS 分支选择 MAVROS `PASSTHROUGH`，保留 FCU 测量时间；
FAST-LIO 保留原 NONE 策略。对齐器以有界异步只读请求核实 `/mavros/time` 的
`timesync_mode=PASSTHROUGH` 且 `use_sim_time=true`；验证缺失/超时不输出，已核实的
时间策略改变则锁存重置。此策略不适用于未经时基验证的实机/D435i 配置。

机体位姿只在守卫明确确认相同 pose_stamp、参考源时间具有前后端点且健康状态新鲜时
接纳。参考位置线性插值、姿态 SLERP，不外推，不使用最新参考帧替代来源时刻。
收集至少 5 对且跨越 0.2 秒的一致样本，估计一次尺度为 1 的偏航和平移并冻结。
初始不一致、参考时间/位姿冲突或跳变、UUID/代次变化、时钟回退均锁存失效。
停流、暂停、健康或参数验证超时使输出无效；不会因旧位姿仍在缓存而继续发布。

PX4 参考与对齐参数的不确定度和相关性未知，因此 aligned_pose **不发布协方差**，
对齐状态 `covariance_available=false`；其 ESTIMATING 也仅指观测输出新鲜。
不发布 map/odom/base_link TF，不覆盖 /odom，不提供速度或导航准入。
导航仍被拒绝，地图会话与执行取消排空尚未接入。所有设置启动后只读。
