# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# 无人机全状态消息
std_msgs/Header header

# 聚合状态有效性。数值字段为默认值时不能据此推断传感器已经就绪。
bool connected              # MAVROS/PX4 连接是否建立
bool pose_valid             # 局部位置在 freshness 窗口内且数值有效
bool velocity_valid         # 局部速度在 freshness 窗口内且数值有效
bool attitude_valid         # 姿态/角速度在 freshness 窗口内且数值有效
bool battery_valid          # 电池数据在 freshness 窗口内且 percentage 在 [0,1]
bool gps_valid              # GPS 数据在 freshness 窗口内且经纬度有效

# 飞行状态
bool   armed                 # 是否解锁
uint8  flight_mode           # 飞行模式 (0=Manual, 1=Altitude, 2=Position, 3=Offboard, 4=Stabilized, 5=Acro, 6=RTL, 7=Land)
string flight_mode_name      # 飞行模式名称

# 姿态 (ROS ENU 语义；yaw=0 指向 +X，逆时针为正)
float64 roll                 # 横滚角 [rad]
float64 pitch                # 俯仰角 [rad]
float64 yaw                  # 偏航角 [rad]
float64 rollspeed            # 横滚角速度 [rad/s]
float64 pitchspeed           # 俯仰角速度 [rad/s]
float64 yawspeed             # 偏航角速度 [rad/s]

# 位置 (header.frame_id 坐标系；EasyFly 局部链路固定为 odom/ENU)
float64 position_x           # x [m]
float64 position_y           # y [m]
float64 position_z           # z [m]，正值表示高于局部原点

# 速度 (ENU坐标系)
float64 velocity_x           # vx [m/s]
float64 velocity_y           # vy [m/s]
float64 velocity_z           # vz [m/s]

# GPS
float64 latitude             # 纬度 [度]
float64 longitude            # 经度 [度]
float64 altitude             # 海拔高度 [m]
float64 relative_altitude    # 相对起飞点高度 [m]
uint8   satellites_visible   # 可见卫星数
float64 eph                  # 水平位置精度因子
float64 epv                  # 垂直位置精度因子

# 电池
float64 battery_voltage      # 电池电压 [V]
float64 battery_remaining    # 剩余电量 [0.0-1.0]

# 遥控器
bool   rc_connected          # 遥控器是否连接
