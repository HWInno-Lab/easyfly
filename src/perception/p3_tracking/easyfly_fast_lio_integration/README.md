# FAST-LIO 定位接入

Gazebo MID360 使用独立传感器配置：

```bash
ros2 launch easyfly_fast_lio_integration fastlio.launch.py sensor_profile:=gazebo_mid360
```

`perception_sitl.launch.py` 已显式选择此配置。单独启动时默认 `sensor_profile:=native`，
保留上游所选 YAML 的外参；`use_sim_time` 本身不选择传感器型号或标定。
`fastlio_config_file` 和旧 `config_file` 别名继续有效，界面参数在文件之后应用。

Gazebo 的激光原点相对 `base_link` 为 `(0, 0, 0.13)`，IMU 原点为
`(-0.011, -0.02329, 0.12412)`，两者坐标轴平行。对应关系为：

- `/livox/imu` 的消息坐标系为 `livox_imu_frame`；统一标定发布
  `livox_frame → livox_imu_frame`，平移 `(-0.011, -0.02329, -0.00588)`。
- FAST-LIO 计算使用 `p_imu = R_imu_lidar * p_lidar + t_lidar_in_imu`，
  因此仿真覆盖文件中的 `mapping.extrinsic_T` 为 `(0.011, 0.02329, 0.00588)`。
- FAST-LIO `/Odometry` 中的 `body` 是 IMU 原点。TF 桥接在计算 `map → odom`
  前应用 `body_to_base_translation = (0.011, 0.02329, -0.12412)`。
  该参数只支持坐标轴平行的平移外参，启动后只读；默认零平移保留既有行为。

Gazebo 覆盖文件只修正安装几何，不将滤波噪声参数当作实机标定，也不覆盖原始硬件 YAML。
变更后的飞行与导航精度仍需单独验收。启动检查与 TF 合成回归不能替代运动定位评测。
