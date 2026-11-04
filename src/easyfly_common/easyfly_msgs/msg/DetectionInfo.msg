# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# 单个目标检测结果
std_msgs/Header header

int32   track_id             # 跟踪ID (-1表示未跟踪)
string  class_name           # 类别名称
float32 confidence           # 置信度 [0.0-1.0]

# 图像坐标 (归一化)
float32 bbox_center_x        # 边界框中心x [0.0-1.0]
float32 bbox_center_y        # 边界框中心y [0.0-1.0]
float32 bbox_width           # 边界框宽度  [0.0-1.0]
float32 bbox_height          # 边界框高度  [0.0-1.0]

# 世界坐标 (ENU, 如果有融合信息则填充)
float32 position_x           # 目标x [m]
float32 position_y           # 目标y [m]
float32 position_z           # 目标z [m]
bool    position_valid       # 世界坐标是否有效
