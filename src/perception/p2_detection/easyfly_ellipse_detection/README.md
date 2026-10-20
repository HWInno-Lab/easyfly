# 椭圆检测

本包负责 ROS 图像接纳和结果发布，计算实现在
[`easyfly_ellipse`](../../../algorithms/detection/easyfly_ellipse/README.md)。

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to easyfly_ellipse_detection
source install/setup.bash
ros2 launch easyfly_ellipse_detection detection.launch.py
```

启动参数 `config_file` 可指定完整参数文件，`use_sim_time` 默认 false；
`input_topic` 默认 `/camera/image_raw`，`output_topic` 默认 `/ellipse_observations`。
其余参数见 [config/detection.yaml](config/detection.yaml)，所有参数在启动时校验且只读。
参数文件节点名为 `ellipse_detection`。更换相机参考系需设置 `optical_frame`。

- 输入：sensor_msgs/Image；BGR8/RGB8，也支持 mono8。
- 输出：`/ellipse_observations`，easyfly_msgs/EllipseObservationArray，使用源图像时间和像素坐标。
- 能力：`/platform/ellipse/capabilities`，easyfly_msgs/GetCapabilities。
- `valid=true` 且空列表是有效的无目标结果；`valid=false` 是坏图像、时间/参考系不匹配、
  重复/乱序、超时或算法失败。停流和冻结仿真时钟会主动发布失效；消费者也应检查源时间。

默认 max_age_seconds=0.5，最大 2073600 像素，支持行尾 padding。启动无图像时 ready=false；
最近有效空图也可以 ready=true。不会产生世界坐标、Action 或飞控命令。

可直接通过 `ros2 run easyfly_ellipse_detection ellipse_detection_node --ros-args --params-file ...`
启动；节点层的 input_topic/output_topic/capabilities_service 可按命名空间需求覆盖。
测试包含真实 ROS 进程和合成图像，以及坏输入、过期、时钟暂停/回退、参数拒绝与干净退出。
