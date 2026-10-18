# ALG-9 整体构建、安装消费与仿真验收

2026-10-18：验证矩阵已执行并归档，**整体验收未通过，ALG-9 尚未签收**。
两种安装各完成 91 包干净构建，独立消费全部通过；9 类受管仿真场景的最新结果为
8 通过、1 失败。搜索动作完成后的 semantic_map 退出崩溃、一次 DDS reader 原生崩溃和
全量规范检查欠账仍保留，不以复跑成功关闭故障。

机器可读结果见 [alg9-results.json](baseline/alg9-results.json)，最终归属见
[final-migration.json](final-migration.json)。本批没有新增算法后端，没有实现六个原有算法占位包。

## 1. 输入、隔离与实际构建

算法实现输入提交 `59b6158`。构建仅加载 `/opt/ros/humble`，不加载旧 EasyFly overlay，
使用全新的 build/install 目录。实际构建包括 19 个算法包、FAST-LIO、EGO、两份 SDK、
反向依赖及其余工作区包；91 是包发现数量，不是已实现功能数量。

| 安装方式 | 证据目录（相对工作区根） | 构建结果 |
| --- | --- | --- |
| symlink-install | `artifacts/algorithms/alg9-20261018/symlink-v2/` | 91/91 |
| 普通复制安装 | `artifacts/algorithms/alg9-20261018/copy-v1/` | 91/91 |

两份 SDK 从当前分类目录重新编译，未复用其旧源码内 build 缓存。RealSense 使用 ALG-8.3
登记的本地 JSON 输入，关闭联网、固件和设备测试选项；私有安装跳过系统 ldconfig。
驱动仍按其默认构建规则解析系统 SDK，不把源码目录搬迁等同于强制切换驱动编译依赖。
独立消费者另验证了本轮安装的 SDK 库、消息与组件加载。

原生 IMU 插件必须显式提供已审核 PX4 Classic 的 `Imu.pb.h` 和 `libsensor_msgs.so` 所在目录。
FAST-LIO 的输出根单独指定到证据目录。Fast DDS 使用固定 SHA 的上游归档。
系统 Gazebo/ROS、现有 PX4 HRT 兼容树、ORT 和模型仍是已登记的外部输入，不属于本轮从源码重建范围。

普通复制安装在全量软件检查后，实际移动为 `copy-v1/relocated-install/`；原 install 路径不存在。
随后才配置独立消费者、导入 Python 模块及运行仿真。搬迁后的环境显式选择各安装包，
不依赖旧 colcon shell 链可任意重定位。算法导出文件没有源码绝对 include 路径，复制头文件不是符号链接。
平台仿真仍使用既有外部 PX4/资源，Livox 仿真插件的扫描 CSV 日志仍指向其原有源码资源；
本轮不宣称整个平台可脱离全部源码和外部资产运行。

## 2. 软件检查与独立消费

下表为修复夹具之前的原始 CTest 注册项统计，GTest/pytest 内部子用例不重复计数：

| 矩阵 | 通过 | 失败 | 跳过 | 范围 |
| --- | --- | --- | --- | --- |
| copy | 500 | 72 | 0 | 572 项，含规范检查 |
| symlink | 266 | 3 | 0 | 269 项，复用同源码的 copy 规范结果 |

copy 中软件/契约检查为 266 通过、3 失败；规范检查为 234 通过、69 失败，涉及 18 个包。
原始软件失败为停流夹具、场景 launch 夹具和 `test_timing_queue_ros`。
symlink 的原始失败为停流夹具、场景 launch 夹具和 `test_task_chain`。
两轮原始矩阵执行期间受追踪输入均未变化。修复后只复验受影响检查，原始失败未覆盖为通过。

上游 RealSense 设备/rosbag 套件未执行：它包含数据下载和按进程名清理相机节点，不属于本轮
软件回归范围。两种安装分别通过既有 SDK 几何、消息往返、组件注册加载消费者，不构造设备节点。

两种安装各通过 12 组既有独立消费夹具，覆盖 4 个 API、15 个计算组件以及 FAST-LIO/EGO/SDK。
纯核心 C++ 消费不加载旧 ROS overlay、不链接 ROS 运行库；两个纯 Python 模块以 `python -I`
从显式安装前缀导入，未导入 ROS。ROS 消息消费者只加载系统 ROS 和本轮安装。

- EGO 两种安装的 200 组样条输出均与迁移前 trace 完全一致：241,285 字节，
  SHA256 `bda356b934954967efdf90365764a5796b10b3fbe290a39a0f437337227aa158`。
- SDK 两种安装的 512 组几何与 FOV 输出逐字节一致，消息往返和组件加载通过。
- 其余迁移前后差分结论沿用 ALG-2～8 的冻结证据，本轮补做全工作区构建、软件回归及安装消费；
  不把旧差分日志写成本轮重新执行的结果。

## 3. 夹具修复与复验

| 提交 | 问题与修正 | 实际验证 |
| --- | --- | --- |
| `556f2cc` | 停流测试在准备起飞时就冻结时钟，无法提供连续新状态；现在先准备，实际运动后冻结故障窗口，并验证冻结和恢复后的新源时间。场景描述测试隔离未执行的假 PX4 原生库检查，同时断言该检查仍被调用。 | 两种安装各停流、场景、原生 IMU 正反例，共 6 项通过 |
| `20fe92a` | 精降 world 的旧 250Hz 物理配置不满足原生 IMU 1000Hz 契约；只对齐物理步进，几何不变。精降入口先确认持续静止，再进入原有严格地面采样。 | 两种安装各 9 项 simulation helper 与 1 项 flake8，共 20 项通过；真实精降正式任务通过 |
| `71c9c31` | SDK demo 夹具只等待服务，尚未发现状态订阅者就发首个 Goal；现在等待三个读者匹配及新状态样本。 | 两种安装各完整任务链检查，共 2 项通过 |

以上合计 28 个定向 CTest 注册项通过。没有更改生产飞控、地图、时间、距离、速度、失标或取消边界。
静止准备要求与已有地面采样相同，采样仍拒绝新的运动/不可信状态；原 80s 准备预算未增加。
冻结时钟测试现在真正进入停流断言，通过不等同于实机停流资格。
新跟随探针另要求近期可见反馈、正在执行的子航点及实际 OFFBOARD 水平运动，再精确取消。

## 4. 显式 PX4/Gazebo 场景

全部使用本机编译的真实 PX4、项目私有 Gazebo server、独立 rootfs 和原有正式入口，串行运行。
导航使用 CPU 感知、原始地图与时效规则；精降保持三相机原频率、0.2m 板、原 0.6/0.5m 交接区间。
普通 CTest 不注册这些飞行动作。每次记录准确 UUID、真实结果、所有权、源时间及所属进程退出。

证据前缀为 `artifacts/algorithms/alg9-20261018/`：

| 场景 | 最新结果 | `result.json` 所在目录 |
| --- | --- | --- |
| A* 短程导航 | 通过 | `sitl-v1/navigation-astar-happy/` |
| EGO 短程导航 | 通过 | `sitl-v1/navigation-ego-happy/` |
| combined 短程导航 | 通过 | `sitl-v1/navigation-combined-happy/` |
| combined 运动中取消及恢复 | 通过 | `sitl-v1/navigation-combined-cancel/` |
| combined 地图生产者丢失 | 通过 | `sitl-v1/navigation-combined-map-loss/` |
| 正式起飞→导航→降落任务 | 通过 | `sitl-v1/navigation-combined-mission/` |
| 两条扫描带、四个子导航的搜索 | **失败：退出崩溃** | `sitl-v1/navigation-combined-search/` |
| 真实感知目标跟随、子航点与精确取消 | 通过 | `sitl-fixture-v2/follow-perception-cancel/` |
| 正式起飞→悬停→精准降落任务 | 通过 | `sitl-precision-v3/precision-mission/` |

搜索动作返回真实成功，几何覆盖 3.3m²，降落确认新鲜 disarmed；随后 semantic_map_node
退出 -11，只有 23/24 个进程正常退出，因此整场仍是失败。没有在清理判定中豁免该进程。
跟随输入来自未改写的真实 `/target_positions`，没有合成目标发布；检查真实水平运动、匹配父取消、
子航点 GetResult 和后续稳定悬停。精降观察到 align、descend、handoff_wait_mode、land_handoff，
正式任务成功、精确 UUID RELEASED、10 个独立地面参考样本及全部进程正常退出。

最初精降在 IMU 速率检查前提缺失时失败；对齐速率后的第二次因地面静止条件不满足而在起飞前拒绝。
持续静止准备修复后第三次通过，原始两次失败保留。第一版跟随探针及原始记录也保留；
增加活动子动作和反馈新鲜度检查后另跑一次通过。共 12 次场景尝试，不能把“最新 8/9”写成一次全绿矩阵。

## 5. 未关闭项与签收边界

- **ALG9-SEMANTIC-EXIT**：搜索退出时真实 SIGSEGV，与历史问题同属 semantic 退出故障类；
  本次没有原生栈，不能断言与旧事故同根因。普通退出/压力组件检查通过不关闭此故障。
- **ALG9-DDS-READER**：copy 的 `test_timing_queue_ros` 中一次 reader 重建后 Python 子进程退出 -11。
  对应观测节点与读者夹具源码未改。两种安装的隔离诊断复跑均通过，未捕获原生栈，故障仍未修复。
- **ALG9-LINT**：原始全量 69 项规范检查失败仍列于结果及 `lint-failures.json`；
  不将功能通过或修复后的定向 lint 写成全工作区规范全绿。
- 历史来源/模型许可、RealSense gitlink 缺少根 `.gitmodules` 映射及其他未关闭退出问题继续保留。

本轮保留了运行期时效与计算预算检查，记录了所属进程树的资源采样；没有做受控旧新硬件性能基准，
不以 RSS 采样或短程仿真推导长期资源稳定性、实机安全、任意障碍绕行或模型精度资格。
源码分类和接口交付完成，不等于上述质量门已经通过。

## 6. 可复放入口与证据保护

构建、串行软件检查和安装消费使用受追踪工具：

```bash
python3 tools/algorithms/verify_workspace.py --mode symlink --build-only \
  --px4-gazebo-build /home/jia/PX4-Autopilot-1.16.2/build/px4_sitl_default/build_gazebo-classic \
  --output /绝对路径/新目录/symlink
python3 tools/algorithms/verify_workspace.py --mode symlink --tests-only \
  --px4-gazebo-build /home/jia/PX4-Autopilot-1.16.2/build/px4_sitl_default/build_gazebo-classic \
  --output /绝对路径/新目录/symlink
python3 tools/algorithms/verify_workspace_consumers.py \
  --matrix /绝对路径/新目录/symlink --output /绝对路径/新目录/symlink-consumers
```

copy 改用 `--mode copy`、独立新目录；软件检查完成后，消费工具加 `--relocate` 实际移动安装根。
首次完整规范结果记录后，第二安装可显式使用 `--skip-source-lint`；不用于隐藏首轮失败。
SDK 离线缓存前提与原生构建细节见 [ALG-8.3](alg8-3-validation.md)。

下列入口会发出仿真飞行动作，必须显式调用，不是上述软件检查的隐式子步骤：

```bash
python3 tools/algorithms/run_workspace_sitl.py \
  --installation /绝对路径/copy/relocated-install \
  --px4-dir /home/jia/PX4-Autopilot-1.16.2 \
  --hrt-compat /home/jia/easyfly_ws/build/easyfly_px4_hrt_compat \
  --output /绝对路径/新的场景证据目录 --execute-sitl
```

原始命令、返回值、JUnit、输入指纹、环境和日志在各 summary.json；修复提交及工具 SHA 在机器摘要。
28 项定向检查分别位于 `fixture-recheck-v1/`、`precision-helper-checks/`、`mission-fixture-recheck-v1/`。
原始矩阵中的缺前提、调度冲突导致的 `symlink-v1` 重试结果已明确弃用，只统计干净的 `symlink-v2`。
copy 范围审查时只暂停了调度器；当时 desktop 的 17 项检查正常完成，保留退出码/JUnit 后继续，未中断或重复这些测试。

44 份既有外部/主安装受保护文件、33 项 ORT 载荷、21 份旧公开兼容头、PX4 protobuf 输入及嵌套驱动身份保持。
未重建或覆盖主工作区 build/install；本轮只安装到证据前缀。源码资源修复在 symlink 安装中正常可见。
旧兼容头及本地旧路径桥接继续保留，分类父目录没有变成额外包。
