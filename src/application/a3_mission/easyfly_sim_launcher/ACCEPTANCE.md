# 软件与仿真验收范围

本文件随安装交付，说明当前支持范围与可执行验证入口。原始录包、逐帧数据、截图、
失败日志和独立物理真值作为单独证据归档保存；启动平台不依赖这些历史记录。
本轮日期：2026-10-28。源码包含开发改动，不能仅以 Git HEAD 表示整份受测源码。
最终交付应同时携带物料清单、源码指纹、构建配置和对应验收摘要。

## 已验证范围

| 能力 | 验证内容与边界 |
| --- | --- |
| 桌面导航 | OpenVINS、FAST-LIVO2、ORB-SLAM3 的显式 Gazebo 导航，经真实 Qt → Mission → SDK → 原生导航 → PX4 SITL 完整任务；原始／导航状态须属于相同后端及会话，状态过期即阻止新任务。 |
| Explore 编辑与执行 | YAML v1 有界区域、禁入框与返航参数；真实 Qt 提交 FUEL 探索、返航、降落和控制权排空。B-01 取消后重启探索的历史地图失效仍按用户要求暂缓。 |
| 专用观测 | AprilTag、精准降落反馈、目标预测及语义分类副本的 Qt／RViz 显示，验证源时间、代次、失效清除和图形输出；显示不会授予控制权。 |
| CSRT 与云台 | 明确选择已绘制图像的同帧 ROI、重置和查询；模拟云台显式使能、实际状态与失联停止。没有签收真实云台，也不把 CSRT 未知置信度解释成概率。 |
| CPU 渲染 | 四核竞争负载下，完整地面感知栈使用 Mesa 加 `gazebo_lockstep:=true`，两次冷启动通过原传感器质量及退出门槛；约 0.16 倍实时速度，不是实时飞行或纯 CPU 赛道资格。 |
| 稀疏场景 FAST-LIO | 显式使用安装包 `easyfly_fast_lio_integration/config/gazebo_mid360_sparse.json`，表面与地图滤波均为 0.1 m；三次冷启动各两轮原任务通过落地上锁、0.15 m 恢复门槛及退出检查。默认值与其他冻结会话未改变，此资格不外推到其他场景或硬件。 |
| 精准降落 | 静止标志板、固定下视相机与专用 D5 夹具；需按 `easyfly_precision_landing` 安装 README 选择已验配置。移动板、机载时延和硬件仍未签收。 |
| 场景 | 20 个清单、14 个登记；六个草稿的保留／替代决策见 `easyfly_gazebo_worlds/SCENE-LIFECYCLE.md`。已登记场景仍需每次启动的资源及实时准入。 |
| 来源与模型 | RealSense 驱动可通过源码随附 bundle 精确离线恢复；YOLO 权重官方来源、许可、类别顺序及等价导出已验证。V-09 近距离卡车车尾漏检仍保留。 |

## 当前尚未签收的部分

历史 Gazebo／独立观察器退出异常仍缺少因果修复。全量规范检查 368 个入口中
360 个通过，剩余 8 个仅涉及旧 EGO 导入文件的版权／许可诊断。
EGO 统一许可、VIKIT 分叉许可冲突、旧 ikd-Tree 与 LKH 授权范围仍有缺口；
当前来源重建与文件指纹不能替代缺失的授权。离线交付采用独立 Ubuntu 用户空间，
安装、配置迁移、仿真输入和卸载的实际结果以该归档对应的验收摘要为准；
本机隔离用户空间不等于另一台实体机器，也不扩大以上飞行资格。
它们必须按各自结果关闭，不能以其他任务成功替代。硬件当前不可用，
本轮不产生 D435i／Livox、实际飞控／遥控器、机载计算机或实体远端机器的签收。
九个骨架模块和明确列出的可选扩展也不在本轮实现范围。

## 安装后的检查入口

```sh
ros2 run easyfly_sim_launcher scene_catalog list --json
ros2 run easyfly_sim_launcher scene_catalog validate primitives_sparse_seed42_v1 --json
ros2 run easyfly_px4_sitl check_graphics --help
ros2 run easyfly_sim_launcher scene_sitl --help
ros2 run easyfly_sim_launcher d5_simulation --help
```

`scene_sitl` 和 D5 飞行分支会向其创建的模拟平台提交真实 Action；先阅读帮助及对应说明，
提供独立 session／DDS domain／端口、审核后的 PX4 兼容输入和新的证据目录。
离线 `scene_catalog` 校验只读、不执行飞行，也不代替传感器和实时状态验收。

源码软件检查入口：

```sh
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --verbose
```

构建时需启用 `BUILD_TESTING`。测试开关、disabled／skipped 计数与图形、真实 DDS、
故障注入和长稳的显式执行范围都应随结果记录；仅有 CTest 返回成功不能代表所有验收分支已执行。
`tools/algorithms/verify_*_install.py --help` 提供独立计算库的安装消费者检查，
需要源码工具和明确的安装前缀；它们不是应用启动的运行时依赖。

## 证据归档约定

归档包含原失败、修复对照、执行命令、受测源码／配置／二进制 SHA、环境和原始结果。
每条关闭记录必须说明实际门槛、退出状态和未覆盖范围；原失败不因后续成功而删除。
原始证据独立存放，公开摘要不携带用户凭据、会话密钥或个人配置。
