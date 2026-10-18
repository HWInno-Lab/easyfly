# EasyFly 算法目录、接口与迁移基线

本目录管理 EasyFly 自有的可复用算法实现及接口。2026-10-18 已完成 ALG-0 盘点与
ALG-1 两个 API 包，以及 ALG-2 的 SORT/世界坐标 ID 整合迁移；A* 等其余算法尚未迁移。

外部算法的目标目录是 `../third_party/algorithms/`，其功能分类与本目录一致：
`preprocessing`、`detection`、`tracking`、`localization`、`fusion_mapping`、`planning`。
按实际实现建立目录；分类父目录不放 `package.xml`。

## 当前交付

- [ALG-0 冻结清单](migration-inventory.json)：历史来源、提取范围和迁移目标。
- [当前迁移进度](migration-progress.json)与 [ALG-2 验收](alg2-validation.md)：实际位置和验证结果。
- [接口与兼容契约](interface-contracts.md)：当前真实行为与拟建立的调用接口。
- [基线验收](baseline.md)：实际构建/测试结果、证据及未覆盖项。
- [构建规范](build-conventions.md)与 [ALG-1 验收](alg1-validation.md)：API 导出和两种安装验证。
- [跟踪接口](tracking/easyfly_tracking_api/README.md)与
  [二维规划接口](planning/easyfly_grid_planner_api/README.md)：实际公共类型和调用契约。
- `baseline/`：源码和配置指纹、外部路径引用、包依赖和测试结果摘要。
- `../../tools/algorithms/capture_inventory.py`：只读盘点工具，不构建、不启动节点、不移动源码。
- `../../tools/algorithms/run_baseline.py`：串行运行清单内既有软件回归，保存命令、日志与 JUnit；
  默认使用 DDS 域 212，各测试自带的非零隔离域继续生效。
- `../../tools/algorithms/verify_api_install.py`：只构建新 API，验证安装及独立 CMake 消费，不运行节点。

自有实现表示本项目维护实现代码，不表示 SORT、A* 等算法理论为原创。
OpenCV/PCL/ONNX 等外部库保持各自来源和许可。新包装库只承载本项目包装实现。

## 依赖边界

```text
EasyFly 节点 → 适配器/工厂 → 算法库 → 能力接口/外部计算库
```

算法库不依赖 EasyFly 消息、ROS 节点、TF 查询、SDK、任务或飞控。可采用 ament 导出
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
