# 多目标跟踪接入（ALG-2）

节点负责 ROS 数据转换、类别过滤、配置、ID 整合和发布；SORT 核心已迁入
`easyfly_sort`，世界坐标 ID 整合已迁入 `easyfly_track_id_fusion`。
`src/adapters/sort_adapter.cpp` 实现统一 `easyfly::tracking::Tracker` 调用与输入/输出检查。

## 后端与配置

当前只实现 `sort`。`tracking.yaml` 增加 `tracking.tracker_backend: sort`；亦可在启动时
通过只读 ROS 参数 `tracker_backend` 覆写。未知后端明确启动失败，不回退到其他算法。
原有平铺的 SORT 参数、类别列表、话题、QoS 和默认阈值保持，速度仍为 px/frame。
`tracking.world_frame` 默认 odom，须与 pixel_to_3d 的 target_frame 一致。

输入源时间须为正且在 epoch 内严格递增；无效几何/分数、重复/回退时间及超过 1024 个
输入检测会在 raw/stable 通道发布空数组，不推进 SORT 历史。世界回流拒绝错参考系、
无效时间、重复 ID 和非有限有效坐标。没有新增世界观测时效认证，不将旧缓存声称为新观测。

## 数据链和重置

```text
/detections → 节点转换 → Tracker/SORT → /tracks_raw → pixel_to_3d → /target_positions
                                        ↓                         ↓
                                   ID 整合 ← 原始 ID + 世界坐标回流
                                        ↓
                                     /tracks
```

首次新轨迹通常先发布 raw ID，下一帧才取得三维位置。ID 整合器现在允许尚无首个世界观测的
临时自身映射在回流后继承墓园 ID；已有真实身份或已继承的轨迹不再改分配。类别、距离与
TTL 条件保持。该项修复与纯移动分开验证，原实现此延迟场景无法继承。

新增 `~/reset`（默认 `/easyfly_multi_object_tracking/reset`，std_srvs/Trigger）：
清空算法、ID 整合、世界坐标和图像缓存，增加内部 epoch，并发布空 raw/stable 数组。
因现有消息没有 epoch，适配器在同一节点进程内保持原始 ID 不复用；ID 空间耗尽明确拒绝。
后端数值结果无效时也清除相关历史。普通输入拒绝不会消耗成功时间戳或 KF 步长。
进程重启与外部消息重放不由该内部 epoch 认证，仍需保持可信且唯一的上游发布链。

旧 `sort.hpp`、`kalman.hpp`、`hungarian.hpp`、`id_consolidator.hpp` 只保留兼容 using 别名。
调用方通过导出 target/依赖包消费，不保留第二份算法实现。

## 验证

库内保留原 SORT/ID 回归，并增加延迟回流与 reset 用例；适配器对照原生输出及失败状态。
`test_tracking_feedback_ros.py` 实际启动本节点和 pixel_to_3d，使用合成深度、内参和静态 TF
验证真实三维反投影、raw ID 回流、稳定 ID 继承、重置、重复帧拒绝和未知后端失败。
这是软件集成回归，不启动飞控或仿真，不认证实际相机/目标跟踪精度或飞行行为。
