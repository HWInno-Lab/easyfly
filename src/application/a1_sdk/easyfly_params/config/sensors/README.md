# D435i 双目＋IMU 采集配置

`d435i_stereo_imu.yaml` 是实机传感器采集预设，用于后续 OpenVINS 双目惯性后端。
配置按工作区内 RealSense ROS2 驱动核对；尚未连接用户的 D435i 验证，也不包含该设备的实测标定。
它随 `easyfly_params` 的 `config/` 自动安装，不属于平台任务 profile。

## 启动

加载 ROS2 和已构建的工作区，在工作区根目录执行：

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=sensors camera_name:=d435i \
  config_file:=/home/jia/easyfly_ws/src/application/a1_sdk/easyfly_params/config/sensors/d435i_stereo_imu.yaml
```

安装后也可将 `config_file` 指向
`<easyfly_params安装前缀>/share/easyfly_params/config/sensors/d435i_stereo_imu.yaml`。
这是上游 launch 接收的扁平 YAML，不是供 `ros2 run --params-file` 使用的节点参数文件。
YAML 中的值在此上游 launch 中覆盖同名参数的 launch 值；修改采样参数时应使用自己的配置副本。

多台 RealSense 并存时额外指定 `serial_no:=_设备序列号`。驱动会去除序列号前导下划线，
用于避免纯数字被 launch 转成整数。设备型号、序列号、USB 模式、固件、SDK 与驱动版本应随采集记录保存。
此启动入口仅运行相机驱动；后续由独立 OpenVINS 接入消费数据。

## 数据与默认参数

| 项目 | 配置/预期输出 |
| --- | --- |
| 左右红外 | 640×480、30 Hz，设备格式 Y8；`/sensors/d435i/infra1/image_rect_raw`、`/sensors/d435i/infra2/image_rect_raw` |
| 双目内参 | `/sensors/d435i/infra1/camera_info`、`/sensors/d435i/infra2/camera_info` |
| 原始惯性流 | `/sensors/d435i/gyro/sample`、`/sensors/d435i/accel/sample`；请求 200 Hz gyro、250 Hz accel |
| 合并 IMU | `/sensors/d435i/imu`；`unite_imu_method: 2`，加速度插值到陀螺仪时间，输出频率以实际 gyro 流为准 |
| 内部标定信息 | `/sensors/d435i/gyro/imu_info`、`/sensors/d435i/accel/imu_info`；与 `/tf_static` 一起保存 |
| 发射器 | `depth_module.emitter_enabled: 0`，避免跟踪随相机移动的投射纹理 |
| RGB/深度/点云 | 默认关闭，先降低双目惯性采集的 USB 与计算负载；后续导航深度流另行配置与验证 |
| 时间 | `use_sim_time: false`；深度/红外模块与运动模块均开启 global time 选项 |
| TF | 相机名 `d435i`，使用 `d435i_*` 内部 frame，避免与 Gazebo 的 `camera_*` frame 混淆 |

支持的采样组合与动态选项仍需在目标设备上通过驱动日志及参数查询确认。
请求频率不代表设备已经按该频率出流；发现 profile 回退或选项不支持应记录并调整。

`enable_sync` 用于视频帧分组，`unite_imu_method` 用于合并不同频率的惯性流，
global time 用于设备/主机时间映射，三者均不能替代相机到 IMU 的时间偏移标定。
必须检查左右帧源时间、IMU 连续性和跨流偏移，保留源时间戳。

## OpenVINS 标定与接入要求

1. 从该设备采集左右相机内参、畸变、双目外参以及相机到 IMU 的外参。
   先核对驱动内部标定，再按需要进行 Kalibr 等离线标定；不使用其他设备的标定数值。
2. 驱动话题名包含 `image_rect_raw` 不足以证明送入算法的像素已经完成哪一种去畸变。
   需核对驱动实际像素和相应 CameraInfo 的 K/D/R/P。若使用原始像素，应配置原始内参/畸变；
   若显式整流后再送入算法，应使用整流后的模型与外参，避免重复去畸变。
3. OpenVINS 中使用 `max_cameras: 2`、`use_stereo: true`，映射左右红外与合并 IMU。
   `T_imu_cam` 表示 `p_I = R_CtoI * p_C + p_CinI`；核对合并 IMU 实际 frame 和传感器原点，
   不能直接把 `d435i_link` 或光学坐标系当作 IMU 坐标系。
4. 校验相机/IMU 时间偏移、IMU 噪声密度和随机游走。合并 IMU 中的插值会改变噪声的时间相关性；
   ROS 消息协方差与 OpenVINS 噪声密度也不是同一个量。
5. 单独标定无人机机体到 `d435i_link` 的安装外参，再接入平台定位接口。
   Gazebo 单目 RGB＋Livox IMU 的外参、噪声与仿真内参均不适用于这台实机。

在实际标定齐全前不提供带猜测值的 OpenVINS 可运行标定文件。
采集预设、驱动出流和 VIO 定位分别记录验证状态。

## 采集与验收

出流后先查询实际话题、QoS 和频率，并检查一条 CameraInfo 和 IMU：

```bash
ros2 topic info -v /sensors/d435i/infra1/image_rect_raw
ros2 topic hz /sensors/d435i/infra1/image_rect_raw
ros2 topic hz /sensors/d435i/infra2/image_rect_raw
ros2 topic hz /sensors/d435i/imu
ros2 topic echo --once /sensors/d435i/infra1/camera_info
ros2 topic echo --once /sensors/d435i/imu
```

采集 rosbag 时同时保存双目、内参、原始 gyro/accel、合并 IMU 与静态 TF：

```bash
ros2 bag record -o /tmp/d435i_vio_capture \
  /sensors/d435i/infra1/image_rect_raw /sensors/d435i/infra2/image_rect_raw \
  /sensors/d435i/infra1/camera_info /sensors/d435i/infra2/camera_info \
  /sensors/d435i/gyro/sample /sensors/d435i/accel/sample /sensors/d435i/imu \
  /sensors/d435i/gyro/imu_info /sensors/d435i/accel/imu_info /tf_static
```

每次采集使用新的输出路径。记录静止段和有纹理的缓慢运动段，检查双目时间配对、
丢帧、IMU 单位与偏置、曝光/模糊、TF 和标定一致性。回放时由 rosbag 发布 `/clock`，
估计器使用 `use_sim_time: true`；不要同时启用实机驱动，以免同名话题有多个来源。

参考：[RealSense ROS2 驱动说明](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md)、
[OpenVINS 标定说明](https://docs.openvins.com/gs-calibration.html)。
