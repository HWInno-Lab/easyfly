# easyfly_obstacle_association

Python 标准库模块 `easyfly_obstacle_association.core`；普通 frozen dataclass 描述 AABB、目标和关联结果。
associate 接受已接纳的有限几何/类别快照与 margin/minimum_confidence，返回 obstacle_index、target_index、
target_id、class_name、confidence。输入须为唯一非负目标 ID、三维有限值和非负尺寸；由平台接纳快照。

保留 expanded-AABB（含边界）和双向唯一匹配；即使某个框本身有多个候选，它对目标的占用也计入歧义。
低置信度、空类别和 unknown 不参与匹配，不任选候选。无关联返回空列表，不能据此判定几何无效。
计算不修改输入，不持有历史，不构造消息、不管理 TTL、不参与占据图或飞行控制。

原 easyfly_obstacle_classification/core.py 保留 header/odom、geometry_valid、快照预算/唯一性/源时间校验、
独立障碍深拷贝、observed_at/expires_at 与原因文案。分类结果仍走独立话题，原障碍/栅格不被修改。
模块实例无状态；调用者在同步调用期间拥有输入且不并发修改。
