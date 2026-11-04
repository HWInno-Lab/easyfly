# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# 单个目标跟踪结果
std_msgs/Header header

int32   track_id             # 跟踪ID
string  class_name           # 类别名称
float32 confidence           # 置信度

# 预测轨迹 (最多30个历史点)
float64[] trajectory_x       # 历史轨迹x [m]
float64[] trajectory_y       # 历史轨迹y [m]
float64[] trajectory_z       # 历史轨迹z [m]

# 预测速度
float32 velocity_x           # 估计速度x [m/s]
float32 velocity_y           # 估计速度y [m/s]
float32 velocity_z           # 估计速度z [m/s]

# 当前边界框
float32 bbox_center_x
float32 bbox_center_y
float32 bbox_width
float32 bbox_height
