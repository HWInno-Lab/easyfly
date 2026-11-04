# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# 任务执行状态
std_msgs/Header header

uint8 IDLE=0
uint8 TAKEOFF=1
uint8 HOVER=2
uint8 WAYPOINT=3
uint8 SEARCH=4
uint8 TRACK=5
uint8 LAND=6
uint8 RTL=7
uint8 EMERGENCY=8
uint8 mission_phase         # 当前任务阶段

string phase_name           # 阶段名称
int32  current_waypoint     # 当前航点编号 (-1表示无)
int32  total_waypoints      # 总航点数

float32 progress            # 任务进度 [0.0-1.0]
float32 estimated_time_remaining  # 预计剩余时间 [s]

bool   mission_success      # 任务是否成功
string message              # 附加信息 (错误/警告/状态描述)
