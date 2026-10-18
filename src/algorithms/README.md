# EasyFly 算法目录、接口与迁移基线

本目录管理 EasyFly 自有的可复用算法实现及接口。2026-10-18 已完成 ALG-0 盘点与
ALG-1 两个 API 包、ALG-2 的 SORT/世界坐标 ID 整合迁移，以及 ALG-3 的 A*/覆盖路线迁移，
并完成 ALG-4 预测/点云过滤、ALG-5 视觉包装和 ALG-6 投影/障碍类别关联迁移。
ALG-7 建图拆分与 ALG-8.1/8.2 第三方 FAST-LIO/EGO 分类已完成；ALG-8.3 SDK 分类也已完成；下一步 ALG-8.4 QGC 资料分类。

外部算法的目标目录是 `../third_party/algorithms/`，其功能分类与本目录一致：
`preprocessing`、`detection`、`tracking`、`localization`、`fusion_mapping`、`planning`。
按实际实现建立目录；分类父目录不放 `package.xml`。

## 当前交付

- [ALG-0 冻结清单](migration-inventory.json)：历史来源、提取范围和迁移目标。
- [当前迁移进度](migration-progress.json)与 [ALG-8.3 验收](alg8-3-validation.md)：实际位置和验证结果。
- [接口与兼容契约](interface-contracts.md)：当前真实行为与拟建立的调用接口。
- [基线验收](baseline.md)：实际构建/测试结果、证据及未覆盖项。
- [构建规范](build-conventions.md)与 [ALG-1 验收](alg1-validation.md)：API 导出和两种安装验证。
- [跟踪接口](tracking/easyfly_tracking_api/README.md)与
  [二维规划接口](planning/easyfly_grid_planner_api/README.md)：实际公共类型和调用契约。
- `baseline/`：源码和配置指纹、外部路径引用、包依赖和测试结果摘要。
- `../../tools/algorithms/capture_inventory.py`：ALG-0 历史布局的只读盘点工具，不构建、不启动节点、不移动源码；当前布局以迁移进度为准。
- `../../tools/algorithms/run_baseline.py`：串行运行清单内既有软件回归，保存命令、日志与 JUnit；
  默认使用 DDS 域 212，各测试自带的非零隔离域继续生效。
- [ALG-2 验收](alg2-validation.md)：跟踪实现与真实世界坐标回流。
- [ALG-3 验收](alg3-validation.md)：规划、覆盖路线与控制增强。
- [ALG-5 验收](alg5-validation.md)：真实 CPU/CUDA 检测、标志位姿、CSRT 与 SDK 安装责任。
- [ALG-6 验收](alg6-validation.md)：中位深度/针孔、普通投影和双向类别关联；平台保留时间与消息。
- [ALG-7.1 验收](alg7-1-validation.md)：实际 PCL 候选、可信地面与显式状态/变换证据。
- [ALG-7.2 验收](alg7-2-validation.md)：当前帧聚类/AABB、Hungarian/EMA 和完整前向扫掠。
- [ALG-7.3 验收](alg7-3-validation.md)：observed-cells/cluster-boxes、膨胀和占据持久化。
- [ALG-8.1 验收](alg8-1-validation.md)：FAST-LIO 完整工程分类、来源、启动/TF 与安装消息消费。
- [ALG-8.2 验收](alg8-2-validation.md)：EGO 完整8包分类、导航/退出回归、安装库与消息消费。
- [ALG-8.3 验收](alg8-3-validation.md)：Livox/RealSense SDK 分类、新目录编译、真实驱动头/库解析及搬迁消费。
- `../../tools/algorithms/verify_sensor_sdks.py`：默认系统SDK与显式私有SDK、软件几何/消息及实际组件加载。
- `../../tools/algorithms/verify_ego_install.py`：两种安装、目录搬迁、200组样条差分及既有安全/导航回归。
- `../../tools/algorithms/verify_fastlio_install.py`：两种安装、目录搬迁、实际 FAST-LIO 进程和原保护验证。
- `../../tools/algorithms/verify_occupancy_install.py`：逐格原版差分、实际释放/reset、两种安装与无 ROS/PCL 消费。
- `../../tools/algorithms/verify_obstacle_install.py`：原版几何/状态差分、实际移动障碍、两种安装与旧头消费。
- `../../tools/algorithms/verify_ground_install.py`：原版拟合/状态差分、实际地面生命周期、安装与旧头消费。
- `../../tools/algorithms/verify_fusion_install.py`：完整消息差分、原 ROS 回归、无 ROS C++/Python 消费。
- `../../tools/algorithms/verify_vision_install.py`：视觉差分、原 ROS 入口、两种安装与无 ROS 消费。
- `../../tools/algorithms/verify_prediction_filter_install.py`：预测/过滤、真实时序、旧头与独立消费。
- `../../tools/algorithms/verify_planning_install.py`：规划/搜索回归、两种安装及真实库消费。
- `../../tools/algorithms/verify_api_install.py`：只构建新 API，验证安装及独立 CMake 消费，不运行节点。

自有实现表示本项目维护实现代码，不表示 SORT、A* 等算法理论为原创。
OpenCV/PCL/ONNX 等外部库保持各自来源和许可。新包装库只承载本项目包装实现。

## 依赖边界

```text
EasyFly 节点 → 适配器/工厂 → 算法库 → 能力接口/外部计算库
```

算法公开接口不依赖 EasyFly 消息、ROS 节点、TF 查询、设备 SDK、任务或飞控。
计算实现可私有依赖推理 SDK，例如 YOLO 包的 ONNX Runtime；不能向公开接口泄漏 SDK 类型/路径。可采用 ament 导出
普通 C++/Python 库；消费者使用安装后的 target/module，不跨包引用源码树。
时钟、参考系和车辆状态证据显式传入；算法自身数值检查和可信模型守卫仍然保留。

节点保留消息转换、QoS、时间/校准/TF 接纳、worker、能力状态和发布。
执行层保留飞行准入、Action 所有权、超时/取消排空、接管、悬停和降落交接。
FAST-LIO/EGO 保留完整 ROS2 工程，通过现有话题和会话接入，无须改造成普通库。

## 批次与变更规则

ALG-0 清单/基线 → ALG-1 接口/导出 → ALG-2 SORT/ID → ALG-3 A*/覆盖路线 →
ALG-4 预测/滤波 → ALG-5 检测/标志/CSRT → ALG-6 投影/类别关联 → ALG-7 建图 →
ALG-8 第三方分类/接入收敛 → ALG-9 整体回归。

移动实现与改进算法分别提交。每批同时迁移库、消费者和必要测试，保留原公开节点接口
和默认配置。需要新增的契约能力标为待实现，不能以接口文档代替运行验证。
已有未解决问题保留记录，不以一次重跑成功宣布修复。

完整实施计划和进度分别在工作区根目录的本地 `plan-v5-algorithms.md` 与
`schedule-v5-algorithms.md`；运行和扩展必需信息以本目录受追踪文档为准。
