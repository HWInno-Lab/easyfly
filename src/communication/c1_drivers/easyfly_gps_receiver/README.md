# 独立串口 GPS 接收器

`gps_receiver` 直接读取用户指定字符设备，与 MAVROS GPS 接入独立。支持原始 8N1 串口的 NMEA GGA + RMC 配对，发布 `sensor_msgs/NavSatFix`、`sensor_msgs/TimeReference` 和私有 `~/status` JSON。

```bash
ros2 run easyfly_gps_receiver gps_receiver --ros-args -p device:=/dev/ttyUSB0 -p baud:=9600
```

必须显式指定绝对设备路径；拒绝普通文件和符号链接。支持 4800、9600、19200、38400、57600、115200、230400 baud。打开后使用 TIOCEXCL 排除后续读者；先前已打开的外部读取程序应由操作者退出。节点不写接收机配置，不更改系统时间或飞控参数。

协议依据 [GPSD 维护者的 NMEA 字段说明](https://gpsd.gitlab.io/gpsd/NMEA.html#_gga_global_positioning_system_fix_data)。每行必须有正确 XOR 校验；限制 128 字节，增量帧缓存有界。GGA 与 RMC 的源时刻必须相同、接收差不超过 1 秒，经纬度差不超过 0.0001°。采用 RMC 的 UTC 日期与小数秒；二位年份按 1980–2079 展开，并拒绝 ROS 有符号秒字段无法表达的日期。拒绝秒值 60，不将其错误转换为翌日。

GGA 的海拔与大地水准面差相加形成 WGS84 椭球高；缺少单位或大地水准面差时不发布完整定位。仅接受真实定位质量 1–5、至少 4 颗卫星、RMC 有效状态，拒绝估算、手动和模拟模式。`NavSatFix.header` 是本机 ROS 收件时间；原始 GNSS UTC 在 `TimeReference.time_ref` 中。HDOP 单独报告，不冒充米制协方差；协方差类型为 UNKNOWN。GN 混合星座语句不臆造所用星座，service=0；GP/GL/GA/GB/BD 按相应星座标记。

同一 UTC 观测只接受一次；回退时间不能刷新旧定位。串口重连创建新解析／时间代次；校验坏包计数后等待后续新观测，显式无效定位、断线或超时发布 NO_FIX 和 NaN 坐标，停止发布时间参考。默认成对源超时 1.5 秒，使用单调时钟，仿真时钟暂停也能触发。默认 GNSS UTC 与主机 UTC 差超过 60 秒时拒绝定位，可按明确部署条件配置为 1–86400 秒。软件解析和伪终端协议验收不能替代真实设备波特率、电气接口、PPS 或定位精度验收。
