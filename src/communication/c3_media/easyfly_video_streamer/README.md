# 视频网络推流

独立 `video_sender` 将 ROS `sensor_msgs/Image` 编码为 MJPEG，经 HTTP multipart 传输；`video_receiver` 校验并解码为新的图像话题，保留源时间与坐标系。接收端不会重复发布上一帧，不提供飞行控制接口。

```bash
ros2 run easyfly_video_streamer video_sender --ros-args --params-file \
  "$(ros2 pkg prefix easyfly_video_streamer)/share/easyfly_video_streamer/config/streamer.yaml"
ros2 run easyfly_video_streamer video_receiver
```

原始 720p BGR 图像每帧约 2.64 MiB，超过 Fast DDS 默认共享内存段容量时会回退到易丢片的 UDP。对同机图像生产者、发送者、接收者和显示消费者，在各自启动前显式设置 `FASTRTPS_DEFAULT_PROFILES_FILE` 为本包 `config/raw_image_transport.xml`，提供 64 MiB 段和 4 MiB 消息容量；不修改全局 DDS 配置。该设置只影响两端本机原始图像传递，跨机视频仍走 HTTPS。

默认仅监听回环地址。跨机需发送端设置 `bind`、`certificate`、`private_key`、`token_file`，接收端设置 `https://主机:端口/stream.mjpg`、相同令牌文件及证书颁发者 `ca_file`（或系统可信 CA）。证书主机名必须匹配；没有跳过验证开关。令牌是拥有者权限 0600 的普通文件，32–128 位 URL 安全字符；不放入 URL、ROS 参数值或日志。两端系统时间需同步；仿真时两端须显式启用同一仿真时钟。

默认预算为 10 fps、1280×720、JPEG 质量 80、每客户端 2 MiB/s、最多 2 个客户端。编码前至多缓存一帧，接收后至多缓存一帧，过载时保留最新观测；单张压缩图像不超过 1 MiB，解码前检查尺寸不超过 1920×1080。发送采用每客户端令牌桶，初始突发量等于一秒预算；统计包含 multipart 元数据，不包含 TCP/TLS 开销。状态话题 `~/status` 提供实际编码／传输／接收／发布／丢弃计数和发送排队最大耗时。

默认源时间超过 2 秒即拒绝，源中断会关闭流；接收端以 0.5 秒退避重连。源时间倒退或坐标系改变产生新 stream_id，接收端在同一身份内拒绝重复／倒退帧。该处理允许仿真重启，不能替代跨机时钟同步。接收端源时间与本地 ROS 时钟衡量总延迟；发送者单调时钟只用于发送端排队统计，不能在另一台机器直接相减。

退出会停止 ROS 输入、关闭监听与连接、等待实际网络／编码线程完成。请求、TLS 握手和写入均有超时；超出连接数直接关闭 socket。实测延迟／带宽依据验收报告中的图像尺寸与主机条件，默认预算不是对任意网络的延迟保证。
