# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# 无人机控制指令
std_msgs/Header header

# 控制模式
uint8 POSITION=0
uint8 VELOCITY=1
uint8 ATTITUDE=2
uint8 ATTITUDE_THRUST=3
uint8 ANGULAR_RATE=4
uint8 TRAJECTORY=5                # M4.3: 位置+速度前馈轨迹 (复用 position/velocity/yaw 字段)
uint8 control_mode

# 位置设置点 (header.frame_id；EasyFly 局部链路要求 odom/ENU, m)
float64 position_x
float64 position_y
float64 position_z
float64 yaw

# 速度设置点 (header.frame_id；EasyFly 局部链路要求 odom/ENU, m/s)
float64 velocity_x
float64 velocity_y
float64 velocity_z
float64 yaw_rate

# 姿态设置点
float64 roll
float64 pitch
float64 yaw_att
float64 thrust                 # 归一化推力 [0.0-1.0]

# 角速率设置点 (rad/s)
float64 roll_rate
float64 pitch_rate
float64 yaw_rate_cmd
