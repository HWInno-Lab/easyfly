# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# 单个目标的 3D 定位结果 (M5 pixel_to_3d 输出)
std_msgs/Header header

int32   track_id             # 跟踪ID (-1 = 未跟踪)
string  class_name           # 类别名称
float32 confidence           # 置信度 [0.0-1.0]

# 世界坐标 (frame_id 指明坐标系, 通常为 map 或 odom)
float32 position_x           # 目标x [m]
float32 position_y           # 目标y [m]
float32 position_z           # 目标z [m]
bool    position_valid       # 深度无效/超范围时为 false

# 相机系观测信息
float32 distance             # 目标距相机距离 [m] (无效为 0)
int32   pixel_u              # bbox中心像素u
int32   pixel_v              # bbox中心像素v
