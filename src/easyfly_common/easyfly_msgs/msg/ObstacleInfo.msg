# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# 单个障碍物信息
std_msgs/Header header

int32   obstacle_id          # 障碍物ID
string  obstacle_type        # 障碍物类型 (tree/building/person/vehicle/unknown)

# 三维包围盒
float32 center_x             # 中心x [m]
float32 center_y             # 中心y [m]
float32 center_z             # 中心z [m]
float32 size_x               # 尺寸x [m]
float32 size_y               # 尺寸y [m]
float32 size_z               # 尺寸z [m]

bool    is_dynamic           # 是否动态障碍物
float32 velocity_x           # 速度x [m/s]
float32 velocity_y           # 速度y [m/s]
float32 velocity_z           # 速度z [m/s]
