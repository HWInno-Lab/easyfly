# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# 云台状态
std_msgs/Header header

float32 pitch_angle          # 俯仰角 [度]
float32 yaw_angle            # 偏航角 [度]
float32 roll_angle           # 横滚角 [度]

bool    is_tracking          # 是否正在跟踪目标
int32   tracking_target_id   # 跟踪目标ID (-1表示无)

string  gimbal_model         # 云台型号
bool    is_connected         # 是否连接
