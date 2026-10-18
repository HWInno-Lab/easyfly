# ALG-3 A* 与覆盖路线迁移验收

日期：2026-10-18；原生产基线 `bdbc34c`，ALG-0～ALG-2 检查点 `22b13aa`。
等价迁移单独提交为 `d57383c`；预算/中断与运行接纳增强在后续独立提交。

## 1. 实际结构

| 部分 | 归属与职责 |
| --- | --- |
| 二维 A* | `planning/easyfly_astar`：八邻域搜索、5 格起点脱困、原视线剪枝 |
| 覆盖几何 | `planning/easyfly_coverage_planning`：矩形蛇形、边界校验、条带完成面积 |
| 公共接口 | `easyfly_grid_planner_api`：普通栅格/地图身份/控制/结果类型 |
| 规划适配 | 原全局规划包的 `src/adapters/astar_adapter.cpp`、工厂与导出共享库 |
| 平台运行 | 全局规划节点、导航协调器与搜索节点继续负责接纳、消息和执行 |

两个实现包不链接 ROS 运行库。A* 依赖纯 C++ 规划 API；覆盖包仅依赖标准库。
旧 astar.hpp 与 search_geometry.hpp 只含新头和兼容别名，没有第二份算法实现。
原 10 条 A* 与 2 条覆盖几何单测随实现迁入。全局规划与 combined 协调器均调用同一
GridPlanner 工厂；搜索节点直接使用新覆盖几何函数。
combined 保持原策略：未知格视为障碍、脱困半径 0、地图最多 1,000,000 格、局部目标最多 256。

## 2. 等价迁移与显式增强

迁移前保留原始两个头文件，分别独立编译对照程序。200 个固定种子的栅格场景和
100 组覆盖几何输入，239,012 字节输出完全相同，SHA-256 为
`71e6fccc246a08cae8d614680c6837de4ef5cb2cbc33ffa54e49021b55fffb24`。
加入控制后，不请求控制的相同输入仍逐字节一致。适配器另对照 30 条原生路径及地图身份。

后续增强明确包括：

- A* 核心支持展开上限、steady-clock deadline 和 atomic 取消；失败无可执行部分路径。
- 展开数计入每个首次弹出的未关闭格（包括目标）；0 禁止展开，恰好够用的限额仍可成功。
- 取消优先于 deadline，deadline 优先于展开上限；回溯/剪枝只检查取消和时间。
- 检查点覆盖输入转换、脱困、open 弹出、展开、回溯、反转、剪枝和 LOS 采样。
- 布局/非有限输入拒绝，非法占据字节视为障碍，上边界采用半开区间，避免非法整数转换/索引。
- 全局节点新增只读启动参数 planner_backend、max_grid_cells、planning_max_expansions、
  planning_timeout；默认 astar、4,000,000 格、4,000,000 展开、5 秒，未知后端启动失败。
- 地图失效中断搜索；执行前和等待期间重新接纳地图。取消/失效与最后一次发送检查互斥，
  清理当前子航点到终态后终止，不发送后续段。失效事件不会因后续有效消息恢复旧路线。

combined 的同步回调搜索受剩余 parent 时间和 input_freshness 窗口约束，最多 1,000,000 展开。
它仍采用原默认互斥回调组，取消消息在同步搜索返回后处理，不能据此宣称其能并发响应取消。
主 A* 节点仍在 worker 中运行，取消可通过 atomic 在核心检查点生效。

这是协作式中断；标准库分配/初始化、容器单次操作与操作系统调度没有硬实时上界。
原半格采样 LOS、起点脱困后的真实起点保留规则和已有对角门限不变，未增加连续碰撞证明。
有效地图更新不会自动让全局节点重规划，原静态模式不变；combined 原有重规划职责保留。

## 3. 验证证据

证据根：`artifacts/algorithms/alg3-20261018/`。
迁移阶段摘要：[alg3-migration-results.json](baseline/alg3-migration-results.json)。
最终摘要：[alg3-results.json](baseline/alg3-results.json)。
最终矩阵 `install-validation-final/` 的两种安装全部通过，验证期间源码指纹保持一致。

每种安装通过 50 条 GTest、46 条 Python 回归用例、1 项旧安装头/格式消费、
1 项独立真实算法消费，共 98 次；两种安装合计 196 次，按 CTest 注册项计为 40 次，
无失败或跳过。包发现为 78，纯规划依赖闭包为 API 与两个实现包。
新增代码的 5 项定向 lint 调用通过。ALG-1 的 13 个和 ALG-2 的 36 个源码指纹均不变。
ALG-0 的 465 个冻结文件中 441 个不变、20 个修改、4 个测试迁出；修改仅在跟踪、
全局规划和任务包内，不涉及其他生产包。

核心与适配器覆盖绕障、无路、起终点、边界、未知格/阈值、非法输入、资源额度、
展开上限、取消/deadline 优先级和无部分路径。真实节点专项覆盖零额度、已过 deadline
不发子航点，以及地图失效/用户取消后排空当前子任务且不发送后续段。

既有协调器、局部协调器、晚会话、真实软件导航链、deadline/endpoint、地图/目标接纳、
航点生命周期、段接纳、任务 prevalidation/executor、搜索 Action 与旧安装头消费均纳入回归。
真实软件链使用协调器、EGO、轨迹服务器与飞控软件节点，输入为合成软件 plant。
不启动 PX4/Gazebo、不连接真实飞行器，不据此认证实际飞行或传感器精度。

历史 ALG-0 的 FAST-LIO HRT 前提和冻结时钟停流失败未涉及本批实现，仍保留原结论。
ALG-2 的跟踪实现及既有两 API 文件均未修改，最终指纹核对单列。

初次 underlay 缺失与新测试 fixture 的 rclpy 属性错误已修正，历史日志保留。
第一轮两种安装的命令全部通过，但发送互斥锁在该轮中途补入，源稳定性检查正确拒绝
将其记为最终通过；随后冻结源码完整重跑，最终统计只取 `install-validation-final/`。
未放宽等待时间、算法阈值或断言以获取通过。

## 4. 复验

在本工作区加载 ROS、主安装与 scene-s3 兼容 underlay，先在独立 prefix 构建
规划 API、两个实现包和两个功能包，随后运行：

```bash
python3 tools/algorithms/verify_planning_install.py \
  --symlink-prefix artifacts/algorithms/alg3-20261018/controls/install \
  --output artifacts/algorithms/alg3-new-run
```

工具拒绝覆盖旧证据，分别验证 symlink/普通安装。包内测试使用明确 ROS underlay；
独立 CMake 消费不加载 ROS overlay、不使用源码 include、不链接 ROS 运行库。
普通安装根移走后检查导出 target、头和适配器库；不扩大为所有 shell setup 可重定位。
全局包既有大量未整理文件未在本轮扩大做全包 lint；新增适配器/专项 Python 与核心格式定向检查。
下一阶段为 ALG-4：预测与点云预处理。
