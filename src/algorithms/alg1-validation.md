# ALG-1 接口与安装验收

日期：2026-10-18。生产源码基线：`bdbc34c`。本批完成两个新 API 包及独立消费验证，
原算法、节点、配置和第三方源码尚未迁移，ALG-2 尚未开始。

## 1. 交付

- `tracking/easyfly_tracking_api`：像素 bbox、检测/可选图像/特征需求、源时间/epoch、
  原始 ID、观测/滤波框、生命周期、显式速度单位、输入校验、序列辅助及 Tracker 接口。
- `planning/easyfly_grid_planner_api`：轴对齐二维栅格、未知格策略、地图身份、XY 路径、
  分项错误、布局校验、取消/deadline/展开预算能力与检查辅助及 GridPlanner 接口。
- [构建规范](build-conventions.md)：包边界、namespaced target、依赖导出和验证方式。
- `tools/algorithms/consumer`：纯 CMake 安装消费示例，测试替身仅用于契约与替换验证。
- `tools/algorithms/verify_api_install.py`：两种安装方式及重定位后的安装消费矩阵。

## 2. 实际结果

最终原始证据：`artifacts/algorithms/alg1-20261018-v2/`。
受追踪摘要：[alg1-results.json](baseline/alg1-results.json)。

| 检查 | symlink-install | 普通复制安装 |
| --- | --- | --- |
| 2 个 API 包独立构建 | 通过 | 通过 |
| 跟踪契约 9 条 GTest 用例 | 9/9 | 9/9 |
| 规划契约 8 条 GTest 用例 | 8/8 | 8/8 |
| 安装后替换/控制 3 项消费测试 | 3/3 | 3/3 |
| include 和 C++17 从导出 target 获得 | 通过 | 通过 |
| 独立消费者不加载 ROS overlay | 通过 | 通过 |
| 消费者不链接 ROS 动态库 | 通过 | 通过 |
| 安装头为普通文件、安装根重定位后消费 | 不适用该模式 | 通过 |

每种安装执行 17 条包内用例和 3 项消费测试，共 40 次用例/消费检查执行全部通过。
按 CTest 注册项计为每种安装 2+3 项、两种安装合计 10 项，避免混淆用例与注册项数量。

包发现结果由原 72 增至 74；两个 API 的源码依赖闭包恰为自身两包，不引入 ROS 节点、
EasyFly 消息、PCL、OpenCV、ONNX 或 CUDA 包。公开头只包含 C++ 标准库。
独立消费者通过 -Werror 构建，并在普通安装根重定位后继续通过全部测试。

源文件 hash 随验证命令保存，验证前后相同。ALG-0 冻结的 465 个生产输入指纹亦未变化。

## 3. 首轮验证工具问题

首轮证据保留在 `artifacts/algorithms/alg1-20261018/`：两个包构建成功，但 CTest 的
ament 包装脚本在清理后的环境缺 `ament_cmake_test` Python 模块，单元二进制未被执行。
工具现为包内 CTest 加载 `/opt/ros/humble` 测试环境；独立消费者仍使用不含 ROS overlay
的环境。另补有限端点差值溢出的边界用例和验证期间源文件稳定性检查后，完整执行 v2。
首轮不计入最终通过结果，日志未覆盖；这是测试驱动环境修正，不是生产算法修复。

## 4. 覆盖与限制

用例覆盖无效/空输入、非有限几何与分数、图像 stride/字节数/溢出、特征需求、资源上限、
同 epoch 重放及 reset、网格形状/有限范围、未知格与阈值，以及取消/deadline/预算优先级。
两个跟踪替身和两个规划替身通过共同虚接口被独立消费者替换。

这些测试不运行 SORT、A* 或 EGO，不认证真实跟踪、地图正确性、轨迹安全或中断时延。
ALG-2/ALG-3 需接入真实算法并验证状态与控制；ALG-0 的 FAST-LIO 启动前提和冻结时钟
停流两项失败未在本批修改或重跑。未执行 PX4/Gazebo、实机、全工作区依赖重建或性能验收。
