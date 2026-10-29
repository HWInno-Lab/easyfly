# 近拐点回归数据

`short_corner_recorded.txt` 是实际 `forest-obstacle-4/navigation-failure-input.json`
栅格的列 `[240,275)`、行 `[235,265)` 裁剪；该失败记录位于
`artifacts/new-algorithms/openvins-sim-acceptance-20261020/`。
源间距、原点与实测 VIO 起点保留。按组合规划器既有规则，将未知或占据值转换为 100，
其余转为 0。没有清理障碍或修改位姿。

回归中 `(0.700000755, -1.099999271)` 拐点来自同一案例的
`navigation-input-1.json` 原生 A* 返回路线；EGO 在距它不足 20 cm 时反复拒绝新轨迹。
回归要求外侧余量不清空任何栅格、所有新线段可见、终点完全保留，并遵守原搜索时间预算。

`tracking_offset_recorded.txt` 和 `tracking_offset_updated.txt` 分别来自
`forest-obstacle-7/navigation-input-1.json` 与 `navigation-failure-input.json`，
保留同样的栅格几何信息及未知格保守转换规则。前者用于重建实际返航路线，后者用于检查
执行偏移后的路线是否仍安全。测试从失败时的 VIO 位置计算相对原路线的横向偏移，
复现两格余量不足，并验证 1 m 跟踪余量加一格后仍保留可通行空间。
该偏移仅用于离线回归，不注入运行时的传感器、定位或飞行输出。

Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0.
