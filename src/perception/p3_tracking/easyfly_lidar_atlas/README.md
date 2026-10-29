# 原生激光关键帧地图与独立全局定位

E-07 已完成明确范围内的原生仿真验收。支持显式连接运行中的 FAST-LIO 或
FAST-LIVO2。地图使用原始注册点云和同时间戳 IMU 位姿，保留每个关键帧的原始
局部坐标、时间、会话、点云完整字段哈希以及里程计和实测回环约束。
哈希包含全部语义字段和原始 `PointCloud2.data` 字节；ROS CDR 对齐填充不属于
消息内容，跨进程重序列化可能不同，因此不用于源身份或参数标定身份。
地图格式 v2 明确记录该规则，拒绝早期试验版 v1。

FAST-LIO 使用其已有的完整去畸变 IMU 坐标点云 `/cloud_registered_body`，与
`/Odometry` 精确配对。默认 `/cloud_registered` 是 0.5 m 滤波后的显示点云，
不能作为独立留出配准的完整扫描；该显示输出及其既有消费者保持原配置。

FAST-LIVO2 前端需显式启用 `fastlivo.launch.py enable_atlas:=true`，或
`perception_sitl.launch.py fastlivo_atlas:=true`。该选项默认关闭；启用时增加
`/fastlivo/raw/atlas_cloud` 和 `atlas_odometry`，在原生视觉更新后使用同一个状态
投影完整去畸变激光扫描。原彩色可视化云仅覆盖相机视野，不作为 atlas 几何输入。

```bash
ros2 launch easyfly_lidar_atlas atlas.launch.py backend:=fast_lio \
  save_atlas:=/absolute/new/atlas-directory
ros2 service call /localization/lidar_atlas/save std_srvs/srv/Trigger '{}'
# 下一次原生前端冷启动后，显式启动新的 atlas 会话：
ros2 launch easyfly_lidar_atlas atlas.launch.py backend:=fast_lio \
  load_atlas:=/absolute/new/atlas-directory
```

参数均只读。启动时读取实际前端参数服务，将完整参数值与后端身份绑定到地图；
设置变化、来源切换、时间回退或已定位后的输入过期均锁存，需显式启动新会话。
Humble 通过唯一原生节点的 DDS 发现端点固定来源，不宣称逐消息 GID 认证。
当前地图不能跨 FAST-LIO／FAST-LIVO2 或不同参数配置混用。

地点描述子仅提出候选。原始点云配准必须通过独立留出样本、六自由度可观测性和
歧义检查；同一会话回环再使用固定首帧规范的 SE(3) 图优化。
几何残差不作为位姿协方差。图优化与注册分别有时间和规模限制；单工作线程，
最多保留一个等待点云。容量为 256 个关键帧、每帧 4,096 个真实采样点、
最多 768 条边和 128 MiB 存储，满额时拒绝继续扩张。

只发布 `/localization/lidar_atlas/pose`、`local_to_atlas` 和 `status`。
位姿表示 `lidar_atlas` 中的原生 IMU；修正变换通过独立消息话题发布。
该节点不发布 `/tf`、PX4 外部视觉、局部里程计或控制命令，也不恢复任务。
前端自身的局部定位与既有导航控制保持独立。

保存目录必须全新且为绝对路径。数值数组无 pickle，固定字段、尺寸和哈希校验，
manifest 最后提交并 fsync；已有地图不能覆盖。服务成功表示文件已完成持久化。
保存需至少两个本次会话关键帧、新鲜输入和空闲几何工作线程。

源时钟和系统独立计时器共同检查新鲜度，`/clock` 冻结也会锁存；恢复输入不能
自动解除锁存。该回归在旧 ROS 时钟计时器上确定失败，修复后的两种时钟测试通过。

验收覆盖普通安装建图和符号链接安装冷启动，在不同初始位置／朝向使用同一地图，
旧地图到物理世界的基准只在首次设置，冷启动不重新拟合。FAST-LIO 两轮全部位姿
RMSE 为 4.1／2.4 cm，FAST-LIVO2 为 5.8／5.7 mm；每轮都有实测回环和保存成功。
全部新关键帧与原始录包的完整字段、真实测量点逐一匹配，复用帧的原始几何不变。
FAST-LIVO2 每帧另有原始相机与有效视觉跟踪证据，原生浮点秒转换误差最多 1 ns。
实际暂停原生前端后约 0.84–1.09 s 锁存，输入恢复后不再输出；控制悬停、落地上锁
及所有原生进程正常退出通过。

这组验收使用 20,000 条球面原生激光射线、0.6–6 m 距离、实体纹理墙和不对称柱体；
相机与 IMU 输入保持原始值。它不代表 MID360 真实视场或实机效果。过长进程暂停
触发原有 32 帧图像队列拒绝的失败案例被保留，未扩大队列；恢复测试在已触发锁存
并排空 0.1 s 消息后恢复前端，再观察 3 个源时间秒。完整证据及所有失败见工作空间
`artifacts/stages1-3-20261028/lidar-atlas-signoff.json`。
