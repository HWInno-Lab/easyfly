# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# 集群中单个无人机信息
std_msgs/Header header

uint8   drone_id             # 无人机编号
string  drone_name           # 无人机名称

float64 position_x           # 相对领机的x偏移 [m]
float64 position_y           # 相对领机的y偏移 [m]
float64 position_z           # 相对领机的z偏移 [m]

float64 velocity_x           # 速度x [m/s]
float64 velocity_y           # 速度y [m/s]
float64 velocity_z           # 速度z [m/s]

bool    is_leader            # 是否为领机
bool    is_connected         # 是否在线

uint8   formation_role       # 编队角色 (0=领机, 1=左翼, 2=右翼, 3=后卫, 4=自定义)
