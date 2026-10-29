# 独立录像、轨迹与原始数据

`video_recorder` 是独立 ROS 2 节点，订阅原始 `sensor_msgs/Image` 和
`nav_msgs/Odometry`。显式开始后异步写入分段 MJPEG/AVI、源时间索引和原生
轨迹；可同时写入 SQLite3 rosbag2。关闭与输入失效均真实收尾，不自动重启。

桌面原有录像仍保存 UI 抽样图像和任务事件，使用原 schema 1；本包采用独立
schema 2，以原始 ROS 输入、轨迹和 rosbag2 为用途。两者均使用可独立解码的
MJPEG/AVI 容器，不将 UI 抽样或录像帧率冒充原始输入频率。旧记录继续由原桌面
播放器打开。本包不依赖 Qt 桌面、飞控或任务服务，也不发布控制命令。

```bash
ros2 run easyfly_video_recorder video_recorder --ros-args \
  -p directory:=/absolute/new-record-directory \
  -p image_topic:=/camera/color/image_raw \
  -p odom_topic:=/mavros/local_position/odom \
  -p record_bag:=true
ros2 service call /video_recorder/start std_srvs/srv/Trigger '{}'
ros2 service call /video_recorder/query std_srvs/srv/Trigger '{}'
ros2 service call /video_recorder/finish std_srvs/srv/Trigger '{}'
```

Gazebo 使用者额外指定 `-p use_sim_time:=true`。录制根目录需要是用户所有的
0700 私有目录，父目录须已存在；每次创建全新 UUID 子目录，不覆盖现有记录。
使用系统 `/usr/bin/python3 -s` 和发行版 OpenCV/NumPy，避免用户 site-packages
中的不兼容二进制依赖影响启动；不修改用户的 Python 包。

开始／停止服务只表示请求被接受，`~/status` 和 `~/query` 返回实际阶段。
只有 `complete`、文件全部落盘且 SHA-256 校验通过的会话可用于回放。
`failed` 和未完成的目录保留原状，不自动删除，也不当成成功。

默认视频 10 fps、最大 1280×720、每段 300 帧，限制 512 MiB 并保留 256 MiB
空闲空间；原始 rosbag2 与视频共用本会话配额。图像队列最多 2 帧，轨迹队列
最多 128 条，所有替换／降频／无效输入计数公开。编码和读回校验在独立工作线程。
原始包保留被节点选中样本的 CDR 与源时间，不宣称录下上游每一帧。

输入时间重复／倒退、坐标系变化、编码／尺寸无效、空间不足或写入错误会
使会话失败；无有效图像达到 3 秒会停止录制。源失效看墙钟，ROS 时间暂停
不会阻止失联检测。恢复后必须显式开始新会话。轨迹保留原始坐标系、四元数和
协方差，未经配准，不跨会话拼接。

验收包括独立 AVI 全帧解码及颜色／位置比较、CDR 原样读回、时间／坐标／布局
拒绝、真实空间保留检查、损坏校验和已安装 ROS 节点的服务／输入／退出流程。

## 历史窗口与隔离回放

```bash
ros2 run easyfly_video_recorder easyfly_history
ros2 run easyfly_video_recorder easyfly_history --inspect /absolute/records/SESSION_ID
ros2 run easyfly_video_recorder easyfly_history --replay /absolute/records/SESSION_ID --auto-play
```

桌面“记录与回放”页面提供“原始记录与历史轨迹”入口。新窗口可以显式启动自己
所有的独立录制进程，输入图像／定位话题、保存目录和仿真时间选择；停止或关闭
窗口时等待实际收尾。每次录制使用唯一服务命名空间，不控制其他录像实例。
历史读取、哈希校验和视频解码在工作线程，Qt 不初始化 ROS；轨迹为原坐标系 XY
等比例显示，可按源时间查看 XYZ、逐帧前后移动。

原始数据回放需要 `bubblewrap`。程序强制创建独立网络、IPC、PID namespace 和
独立 `/dev/shm`，禁用外部 DDS discovery/profile 配置；从验证过的 sqlite3
rosbag2 仅读取 Image／Odometry，发布到 `/history/image`、`/history/odom` 和隔离
环境中的 `/clock`。不会回放控制命令、服务、Action、TF 或参数；缺少隔离能力时
拒绝启动，不退回当前 ROS 图。默认暂停，提供播放／暂停／源时间跳转、0.25–4 倍
速和正常结束。命令行 `--auto-play` 为显式自动播放。

视频为固定帧率容器，ROS 回放按记录的源时间间隔调度，两者显示含义不同。
轨迹绘图按原始源时间选择对应点，未配准、未重建丢失样本，也不将协方差当作精度
真值。源坐标系改变会结束当前记录，用户需要新会话。
