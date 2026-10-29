# EasyFly 多机受管仿真

一个 Gazebo Classic 物理世界中运行 2–4 架独立 PX4 Iris。每架飞机使用独立
PX4 实例、系统 ID、端口、私有 rootfs、DDS 域和控制服务。共用域只接收带唯一
`/fleet/<id>` 名称的观测，以及带 `<id>/` 前缀的 TF；控制命令不会跨这个边界转发。

## 配置

以 `config/fleet.example.json` 为模板，选本机实际可用的 CPU ID。Gazebo 和每架飞机
分别分配至少两个不相交的 CPU；明确协调域、每车域、Gazebo 端口与出生位置。
实例 0–3 分别对应系统 ID 1–4；TCP `4560+i`、FCU `14540+i`、offboard `14580+i`、
payload `14280+i`、gimbal `13030+i` 和 GCS `18570+i` 在启动前全部检查。
初始位置至少间隔 3 m；相机更新率显式设为 5–30 Hz，动力学和传感器噪声参数保留。

```bash
ros2 run easyfly_fleet_sim fleet_sim --config /absolute/fleet.json \
  --output /absolute/new-session
```

使用已验证的 PX4 兼容构建，可通过 `--px4-dir` 明确指定。输出必须是新的私有目录，
保存配置、会话身份、输入来源哈希、派生模型、各车私有 rootfs、原生日志和退出状态。
该入口仍持有原 `single-vehicle.lock`，与旧单机入口互斥；不删除锁文件或复用旧仿真。
每车内部沿用现有根话题，隔离在各自 DDS 域中，保留原有 FSM/SDK/飞控控制所有权。
跨车辆工具必须同时使用 manifest 中的车辆会话、域、系统 ID 和 namespace，不能仅凭名称猜测。

## 坐标与传感器

共同域发布 `/fleet/<id>/uav_state`、`odom`、`flight_status`、`identity` 和
`boundary_status`。原始相机/Livox 输出在各自 `/fleet/<id>/camera` 和 `livox` 下；
原生 C++ `fleet_sensor_boundary` 把属于该车的传感器与时钟送入其内部域，并移除
精确匹配的车辆 frame 前缀；Python 边界负责状态、身份和测量坐标，不转换高频点云。
原生统计发布到 `/fleet/<id>/sensor_boundary_status`，退出时保存
`<id>-native-sensor-statistics.json`；IMU 使用 256 帧有界接收队列。
原始时间戳、图像/点云内容保持，超时、重复源、时钟回退会拒绝继续刷新有效观测。
米制深度使用 `/camera/aligned_depth_to_color/depth/image_raw`（32FC1），
相机内参来自对应 `/depth/camera_info`。本机进程采用本包
`config/raw_image_transport.xml` 的 64 MiB 共享内存段；独立观测工具也应在启动前
显式设置 `FASTRTPS_DEFAULT_PROFILES_FILE` 指向该文件，避免大图像回退到 UDP 分片。

共同 TF 中每车的 `odom → base_link` 和传感器变换均带唯一前缀。
`world → <id>/odom` 必须由三次稳定地面状态和真实 Gazebo 实体位姿查询测得，
不能把出生点坐标直接当成 PX4 的局部原点。测量记录写入 `<id>-world-alignment.json`；
未建立该变换时，协同任务不得使用世界坐标。每次新仿真重新测量，不沿用旧会话。

## 生命周期与范围

启动任何关键进程失败都会结束该次舰队会话。收到停止信号后，先结束控制与任务服务，
再停止边界、各 PX4 和 Gazebo；全部进程组真正退出后才释放锁。原生遥测用所属
PX4 实例的命令客户端设置并读回，另一个实例的成功输出不能作为本车证据。
停止信号只发送给受管启动器，由 ROS launch 转发一次；原生子进程异常退出或
升级至 SIGTERM／SIGKILL 都会记为失败，即使启动器本身返回 0。
进程存活状态不表示飞行准入；起飞、降落与所有权继续由原生飞控服务判定。

本入口仅用于软件仿真，不连接物理设备。共用 GCS 接收端可按 MAVLink 系统 ID 区分
各机；默认不配置额外 MAVROS GCS 转发。实际射频、硬件时钟和机群实飞属于阶段 4。
