# EasyFly

自主无人机开源平台，基于 ROS2 Humble + PX4 1.16 + Gazebo Classic。

## 架构

| 层 | 子层 | 说明 |
|---|------|------|
| ① 通信层 | C1 驱动 / C2 协议 / C3 媒体 | 传感器驱动、MAVLink桥接、视频推流 |
| ② 感知层 | P1 预处理 / P2 检测 / P3 跟踪 / P4 融合 | 目标检测跟踪、SLAM、深度估计 |
| ③ 执行层 | E1 状态机 / E2 规划 / E3 控制 | 任务调度、路径规划、飞行控制 |
| ④ 应用接口层 | A1 SDK / A2 服务 / A3 任务 | 对外API、ROS2服务、任务编排 |

## 编译

```bash
source /opt/ros/humble/setup.bash
source easyfly_env.bash
build_all.bash
```

## 算法结构与验收

项目维护的 4 个算法接口包和 17 个计算组件位于 `src/algorithms/`；外部算法工程、
运行库及设备 SDK 按类别保存在 `src/third_party/`。新增后端仍需接口适配、参数及运行验收。

新增[椭圆与彩线检测](src/algorithms/classical-detection.md)包含独立算法库、ROS 节点、
消息、配置及启动入口；算法相关预留包剩车辆重识别、深度估计、集群规划和独立轨迹优化四个。

ALG-9 基线的完整构建、两种安装、受管仿真和未通过项见
[ALG-9 验收报告](src/algorithms/alg9-validation.md)，组件归属见
[算法索引](src/algorithms/README.md)。ALG-9验证已执行，整体验收尚未签收。

## 仿真验收

统一导航、取消、地图失效、任务编排和区域搜索的真实 PX4/Gazebo 验收入口及
电池采样修复及仿真默认兼容版见[电池 SITL 说明](docs/development/battery-sitl.md)。

安全边界见[导航 SITL 说明](docs/development/navigation-sitl.md)。该入口需要显式
调用，会发送仿真飞行动作，不由普通 `colcon test` 自动执行。

`semantic_map` 退出崩溃尚未定位，当前回归与默认关闭的专用取栈入口见
[semantic 退出专项](docs/development/semantic-shutdown.md)；正常复跑不等于修复。

## 许可证

EasyFly 自有代码采用 [Apache-2.0](LICENSE)，署名沿用 `EasyFly Project`。

第三方源码、模型、资源及其本地移植保留各自原许可；根许可证不替代这些声明。
`easyfly_gazebo_plugins` 的 Livox 移植和 `easyfly_gazebo_models` 的模型资源仍需逐项
核对来源，暂未覆盖其原有包许可元数据。详情见[许可说明](docs/development/copyright-options.md)
与[第三方来源登记](docs/development/third-party-provenance.md)。
