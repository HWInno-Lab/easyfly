# EasyFly 全功能测试问题修复记录 v1

依据：[test-full-v1.md](test-full-v1.md)。修复日期：2026-10-24（Asia/Shanghai）。

第一轮保留原报告及其失败证据，在 `artifacts/test-full-debug-v1-20261024/` 独立构建、安装和回归。没有连接真机，没有替换系统 ROS/Gazebo/DDS 库，没有运行宿主全局清理。任务开始前已有大量工作区改动，基线保存在 `baseline.diff`、`baseline-status.txt`，本轮相对基线的已跟踪文件差异另存 `task-tracked.diff`。

**状态口径：已修复表示对应确定性复现通过；加固、配置修正与间歇问题关闭分别记录。原报告的 30 项不能全部标为已修复。** 第一轮 17 项确定性缺陷/测试入口或配置复现已修复验证，13 项保留为加固、缓解或未闭环。**下述第一轮历史结果予以保留；用户要求继续修复后的最新进度见文末“第二轮追加修复”。**

**最新结果（第二轮）：** 已追加修复 SUPER 备用轨迹、MAVROS 退出、雷达/TF 时间戳、定位对齐缓存及 ORB 重力校准，并完成原场景复测。最终 65 个 CTest 入口经一处夹具修正后全部通过，306 个 JUnit 用例和另行 110 个重复用例无失败；760 个已记录进程身份无遗留。F-014、F-007 及部分环境/历史间歇性边界仍未完全关闭，详见文末“第二轮追加修复与最终验证”。第二轮证据：`artifacts/test-full-debug-v1-round2-20261024/`。

## 第一轮逐项结果

| 编号 | 问题 | 本轮处理与状态 |
| --- | --- | --- |
| F-001 | GPS 高度 ENU 符号相反 | **已修复**。Up 改为 `alt-ref_alt`，原 15 项头文件契约全部通过，增加持久化坐标方向回归。 |
| F-002 | Sophus 默认构建引用未交付目录 | **已修复**。缺省关闭未随裁剪归档交付的上游示例/测试；默认配置实际构建成功，补丁及来源哈希同步并通过校验。 |
| F-003 | 里程计速度旋转误用半角 | **已修复**。复用完整 RPY 的 ENU→FLU 变换，ROS 接口测试验证 90° 航向等实际输出。 |
| F-004 | 无效状态继续产生有效外观 TF/odom | **已修复**。校验连接、姿态/位置/速度有效性、有限值、坐标系、时间戳及源/到达新鲜度；无效新消息立即覆盖旧缓存，停止输出。 |
| F-005 | 旧多项式头文件不能独立编译 | **已修复**。修正 jerk 循环索引类型并补齐平均速度返回值；独立编译及 UBSan 通过。范围是保留的旧头文件。 |
| F-006 | EGO 退出测试不能解析 launch 参数 | **已修复**。测试通过 LaunchContext 求值 LaunchConfiguration，并提供显式 float；原生 12 种初始化/就绪/等待、冻结/活动时钟、INT/TERM 组合通过。 |
| F-007 | 定位准入测试偶发 rclpy 段错误 | **生命周期加固，未关闭历史崩溃**。每例独立 Context/Executor、显式销毁 ActionClient，11/11 通过；原崩溃没有确定的充分根因证据。 |
| F-008 | 场景测试错误要求全部支持 base_sim | **已修复测试契约**。单列 3 个 navigation_dev 专用登记场景，保持 11 个基础场景和草稿不准入检查。 |
| F-009 | 测试在 DDS 激活前载入系统库 | **已修复入口**。场景 CTest 在进程启动时前置私有库；ORB 守卫脚本先 activate 再导入 rclpy/消息。普通构建环境下原入口验证通过。 |
| F-010 | MID360 等价性测试仍比较旧 IMU 配方 | **已修复测试契约**。核对 ORIGINS 的输入/输出哈希，再应用声明的原生 IMU 转换配方比较物理/传感器语义。 |
| F-011 | 旧入口起飞后无法稳定悬停 | **候选诱因修正，尚不宣称完全关闭**。内置平地 empty/perception_demo 的默认出生高度从 0.83 m 改为 0.1 m，避免初始化时大幅自由落体；两次独立冷启动 EGO 导航/降落通过；仍保留历史间歇问题未关闭。 |
| F-012 | SUPER 地图变化后备用轨迹失败 | **未修复**。原始请求仍可重现；回退种子及备用优化重试候选未稳定通过，候选实现已撤回，保留失败日志，未减小安全半径。 |
| F-013 | Livox 安装仍依赖源码扫描 CSV | **已修复**。相对资源从安装后的 ament share 解析；私有挂载隐藏源码后实际收到 150 帧、每帧 20000 点，正常退出。 |
| F-014 | FUEL 取消后新探索 SOURCE_MAP_INVALID | **未关闭**。没有足够证据把取消逻辑与共享地图失效唯一关联；保留原失败及原报告冷启动成功对照。 |
| F-015 | 登记导航点云失效、EGO 保护停止 | **仍可复现，未修复**。增加源时刻 TF 诊断，实际捕获 map←odom 不能覆盖点云时刻；另一次冷启动完整任务通过，不能覆盖失败。 |
| F-016 | MAVROS 精准降落后退出 SIGSEGV | **未修复**。核对安装的 2.14.0 Router/UDP 生命周期及原报告 core 堆栈；未用退出码屏蔽、强制退出或未验证系统库替换绕过。 |
| F-017 | 清理脚本漏新节点、零残留计数错误 | **已修复**。统一精确进程名匹配、同 UID 清理与最终计数；私有 PID/挂载命名空间中 18 个旧类和 7 个新类均清理，2 个无关对照保留。 |
| F-018 | 旧桥接默认 drone_id 类型不符 | **已修复**。launch 显式 ParameterValue(str)，默认 launch 成功启动并正常退出。 |
| F-019 | 旧桥接首条轨迹前悬停崩溃 | **已修复**。时间对象使用节点时钟类型，未收到指令时短路时间差运算。 |
| F-020 | 旧桥接断流回到过期起点 | **已修复**。进入断流回退时锁存最新有效里程计位置/航向，避免飞回初始点。 |
| F-021 | 旧桥接转发 NaN/Inf 指令 | **已修复**。位置、速度、加速度、航向和航向率逐项拒绝非有限值；无效 odom 不作为悬停目标。F-018～021 的 13 项隔离 ROS 契约通过。 |
| F-022 | FAST-LIVO2 无注入 IMU 源间隔失效 | **未关闭**。保持原 IMU 间隔守卫；既有成功取消复测不等于源间隔原因消失。 |
| F-023 | FAST-LIVO2 对齐参考断档/输出过期 | **未关闭**。源断档与墙钟到达延迟仍需区分，未扩大对齐或导航超时。 |
| F-024 | OpenVINS 观察配置不能初始化 | **已修复观察配置复现**。观察模式使用已验证导航配置中的 Gazebo 重力、初始化特征/视差和直方图处理参数；保持静态初始化器；原默认运动复测完成初始化、飞行/落地及独立精度审计。 |
| F-025 | OpenVINS 四核下对齐/精度超限 | **未关闭**。没有改变 0.3 m RMSE / 1 m 最大位置误差门限，未用非压力运行替代四核压力验收。 |
| F-026 | 书店/住宅/赛道起飞前时序失败 | **配置改善但问题未关闭**。两场景原生 /clock 配置为 100 Hz，避免低实时率下 10 Hz 时钟使状态更新先天稀疏；不伪造时间戳。书店仍未满足连续就绪、住宅在心跳检查失败；赛道原本已是 100 Hz，不能套用同一结论。 |
| F-027 | 部分场景地图启动未就绪 | **未关闭**。保留源时刻 TF 约束；稀疏场景本轮有完整成功，也有飞行后失败，不推广为森林/仓库启动问题已解决。 |
| F-028 | 登记任务飞行中输入失效而中止 | **未关闭**。未把进程正常退出记为正常落地；仍需各原场景稳定飞行证据及生产链根因修复。 |
| F-029 | gz model 查询退出时线程 self-join | **查询路径已修复**。新增使用已固定私有 transport 的只读 easyfly_model_query，显式清理后才输出；6 次连续位姿查询、碰撞身份、缺失模型拒绝与退出均通过。 |
| F-030 | ORB 惯性观察公共位姿未通知的大跳变 | **输出保护已加固，估计精度未修复**。公共输出前检测源时间、位移及旋转连续性，失败锁存并报告 reset；保留原始诊断输出。原 10.11 m/33 ms 跳变回归被拒绝，不能据此声称原 16.496 m RMSE 已解决。 |

## 关键改动与验证边界

### 坐标和旧桥接接口

- `easyfly_utils/include/easyfly_utils/geometry.hpp` 修复 ENU Up 方向；新增 `test_geometry_contract` 检查东/北/上/下及 ENU→NED 方向。
- `easyfly_coord_transform/src/odom_tf_publisher.cpp` 统一使用坐标工具。默认状态超时 0.5 s，同时检查源年龄及 steady-clock 到达年龄，继续使用源时间戳。新增 `test_odom_tf_ros` 和独立探针检查各类无效输入、恢复及停止状态流后的 TF/odom 停止。
- `ego_to_mavros/src/bridge.cpp` 与 `launch/bridge.launch.py` 修复 F-018～021。该包是旧适配入口，不能把结果算作主执行链新增能力。回归辅助程序最初同时给 launch 和进程组发送 SIGINT，触发重复信号的误报；已修正为先给 launch 一次 SIGINT，并保留这次失败日志。
- 证据：`header-contracts.log`、`polynomial-contract.log`、`build/easyfly_coord_transform/ament_cmake_pytest/test_odom_tf_ros.txt`、`build/ego_to_mavros/ament_cmake_pytest/test_bridge_ros.txt`、`ego-shutdown-regression.log`、`localization-selection.xml`。

### 安装、构建与测试入口

- Sophus `CMakeLists.txt` 的缺省值及 `tools/algorithms/patches/fastlivo2/sophus-integration.patch`、`fast_livo2.provenance.json` 一起更新。来源校验没有禁用。
- Livox 保留显式绝对资源路径的语义；相对路径通过 `ament_index_cpp` 定位 `easyfly_gazebo_plugins` 的安装资源。源码隐藏验证在私有 user/mount 命名空间执行，不移动或删除真实源码。
- F-009 最初尝试追加库路径仍会先解析系统库，实际失败已保留；最终改为启动前**前置**私有路径。ORB 独立脚本也修正导入顺序，而不是要求用户额外手动修改系统环境。
- 场景和仓库针对性测试共 72 项通过；整个 `easyfly_sim_launcher` 功能测试 342 个 JUnit 用例通过。原生 IMU 两项第一次遇宿主 GLX 问题，独立 Xvfb 复测两项均通过，没有把环境失败删除。
- OpenVINS mock guard 首次包测试出现一次 `pending_camera_expired`（1/7 失败），独立复测 7/7 通过；原失败保存在 `fixed-test-log/`，未通过删除用例或增加门限掩盖。
- 证据：`fastlivo-source-verification.json`、`catalog-warehouse.xml`、`fixed-test-console.log`、`livox-install/hidden/result.json`、`orb-imu-guards/result.json`、`native-imu-regression.log`。

### 地图和规划器排查

本轮 `scene-sparse-tf-diagnostic` 在飞行中复现 EGO 保护停止。诊断明确指出：点云源戳 **36.421 s**，`map←odom` 最新变换仅 **36.399420 s**，等待到现有 1 s 墙钟期限仍不可用。该轮未完成 Land，不能用 24 个子进程正常退出代替落地。`scene-sparse-source-trace` 冷启动追加了只读源戳观察器，完整参考任务通过；所有原门限和任务均保留。两轮对照只能说明问题间歇，尚不能归因到单一处理节点。

SUPER 使用原失败 `job-2.request`，SHA-256 为 `2d4693f594aa1283b506abd7ba1a9b39f8d857442b2bd6a833ff7eab8db9bbac`。沿实际轨迹后退种子的候选绕过了原 CIRI 最近距离失败，但备用优化仍拒绝；使用第二次优化结果的候选也未稳定通过。因此撤回这两项候选，不接受未通过原生独立校验的轨迹，不减少体素包络或安全半径。证据在 `super-map-change-replay/`、`super-map-change-replay-2/`，候选源码留于 `super-candidate.cpp` 供复核。

### MAVROS 退出调查

核对的上游版本是 MAVROS 2.14.0：[节点 main](https://raw.githubusercontent.com/mavlink/mavros/2.14.0/mavros/src/mavros_node.cpp)、[Router](https://raw.githubusercontent.com/mavlink/mavros/2.14.0/mavros/src/lib/mavros_router.cpp)、[UDP 连接](https://raw.githubusercontent.com/mavlink/mavros/2.14.0/libmavconn/src/udp.cpp)。原报告 core 提示退出阶段仍有 I/O 工作，但目前没有可独立复现、验证的完整修复。不能简单在持有 Router 锁时关闭并 join 连接，接收回调可能仍需同一锁。未把这种未验证的改动写入系统安装。

### ORB 公共输出保护

新增 `pose_continuity.hpp`：仅在有效源戳、有限位置、可归一化四元数、递增时间且间隔不超过 0.5 s 时继续；位移界限为 `0.25 + 5*dt` m，角度界限为 `0.15 + 3*dt` rad。这是平台运动连续性保护，不替代定位精度审计。检测失败后进入 `unannounced_pose_discontinuity` 锁存状态，本进程不会自动把后续点重新当作连续有效输出。

4 个 gtest 覆盖原跳变、锁存、正常 3 m/s 运动、四元数符号等价、角跳变、重复/断档、NaN 和零四元数。ORB 包共 20 个 JUnit 用例及 6 个 ROS IMU 守卫用例通过。尚未重新完成原始完整 RGB-D/IMU 回环精度审计；F-030 的算法精度问题保持打开。

## 最终软件回归汇总

最后一轮 `final-build.log` 中 7 个重建包成功。`final-selected-test-console.log` 中 10 个所选包执行完毕，**62 个 CTest 入口全部通过，嵌套 JUnit 共 561 个用例、0 失败/错误/跳过**；两种层级有包含关系，不能相加。`easyfly_gazebo_worlds` 没有自身 CTest 入口，其场景契约由 sim_launcher 的测试覆盖。范围排除了原有格式/版权静态检查，未声称这些遗留全部修好。

| 包 | CTest 入口 | 嵌套 JUnit 用例 | 结果 |
| --- | ---: | ---: | --- |
| `easyfly_utils` | 1 | 0 | 通过 |
| `easyfly_super_planner` | 2 | 16 | 通过 |
| `easyfly_gazebo_plugins` | 23 | 50 | 通过 |
| `easyfly_gazebo_worlds` | 0 | 0 | 无独立入口 |
| `easyfly_sim_launcher` | 11 | 342 | 通过 |
| `easyfly_openvins_integration` | 10 | 75 | 通过 |
| `easyfly_coord_transform` | 1 | 1 | 通过 |
| `ego_to_mavros` | 1 | 1 | 通过 |
| `easyfly_orbslam3_integration` | 2 | 20 | 通过 |
| `easyfly_px4_sitl` | 11 | 56 | 通过 |

此外：原 15 项头文件探针、旧多项式 UBSan 编译/运行、EGO 12 种原生退出组合、定位准入 11 项、ORB IMU 守卫 6 项、隐藏源码的 Livox 安装运行、隔离清理验证均已执行。`git diff --check`、Python 语法、shell 语法及 FAST-LIVO2/SUPER 来源校验通过。最终结果保存在 `final-test-summary.json`、`sitl-summary.json`，本轮源码清单/哈希见 `task-files-sha256.json`。收尾按 PID 与进程启动时刻只读复核了 157 个本轮仿真进程身份，无同身份进程存活，见 `final-process-check.json`。

## 实际仿真回归

以下均是本机软件 Gazebo/PX4，使用独立显示环境且串行执行；结果路径为 `artifacts/test-full-debug-v1-20261024/sitl/<用例>/result.json`，命令及整体返回码为相邻的 `<用例>.json`。

| 用例 | 结果 | 说明 |
| --- | --- | --- |
| `scene-sparse-tf-diagnostic` | 失败 | 飞行中源时刻 TF 超时、EGO 保护停止；未记为落地成功。 |
| `scene-sparse-source-trace` | 通过 | 完整登记往返参考任务与清理；125.79 s。 |
| `legacy-spawn-recheck` | 通过 | 修改出生高度后的旧入口 EGO 导航与 Land；78.81 s。 |
| `legacy-spawn-cold-repeat` | 通过 | 第二次独立冷启动，EGO 导航、Land、新鲜上锁及清理通过；78.00 s。 |
| `openvins-observation-recheck` | 通过 | 原 forest 默认 dynamic 参数；352 个真值配对、RMSE 0.03469 m、最大 0.07070 m；90.30 s。 |
| `bookstore-clock-recheck` | 失败 | 时钟确为 100 Hz，状态中位到达间隔降至 0.0501 s，但 P95 0.3842 s、最大 1.3261 s，连续就绪仍失败；未飞行。 |
| `house-clock-recheck` | 失败 | 原生时钟 100 Hz 验证通过，心跳时序检查失败；未飞行，正常清理。 |

## 第一轮结束时尚未关闭的问题（历史状态）

F-007、F-011、F-012、F-014～016、F-022、F-023、F-025～028、F-030 仍有原崩溃、间歇性失败或算法精度边界需要验证，具体状态以上表为准。不能把测试入口修复、保护正确触发、单次冷启动成功或正常进程退出合并成“30 项全部修复”。后续应优先捕获失败轮的 FAST-LIO/Odometry/map→odom 与点云源戳/到达链，修复原生 MAVROS 退出生命周期，再分别验证受限 CPU 下定位误差与原始 ORB 惯性回环。

OpenVINS 本轮保持原森林场景、默认 0.3 m/s 起飞、12 s 持续观测、无视觉特征板和原精度门限。校准参数统一后已输出有效位姿，独立审计全部通过。该结论不覆盖 F-025 的四核压力条件。书店/住宅仍失败，时钟配置改善不能算作 F-026 完成修复；探针已补充失败前保存心跳样本及区分到达间隔与源戳不递增，供后续捕获具体分支。

## 复现方式与范围

```bash
source artifacts/test-full-debug-v1-20261024/environment.bash
python3 tools/algorithms/verify_fastlivo_source.py
python3 tools/algorithms/verify_super_source.py

# 软件/模拟接口回归使用本轮独立安装，包列表和命令见 final-selected-test-log/。
# 实际 SITL 使用 simulation-environment.bash，并由 private_display.py 提供独立 Xvfb。
# run_sitl.py 要求新的唯一用例名，禁止覆盖已有结果。
```

最终测试把 colcon 的包发现范围显式限定为 `--base-paths src`；首次未限定范围的命令在扫描证据目录阶段被主动中断，尚未执行任何用例，其日志仍保留。

本轮没有重新构建、执行原报告所有 111 包和全部飞行矩阵。只对改动包、原始确定性复现、关键仿真场景进行针对性回归；原报告的全量结果、静态检查遗留和未覆盖的真机行为不能被本报告的局部通过替代。

<!-- ROUND2_START -->

## 第二轮追加修复与最终验证

证据与独立安装：`artifacts/test-full-debug-v1-round2-20261024/`。第二轮基线为第一轮修复后的工作区，见 `baseline.diff`、`baseline-status.txt`、`baseline-report.md`。以下最新结果补充并更新上文第一轮历史状态；保留全部失败尝试，不改变原验收门限。

| 编号 | 第二轮处理与当前证据 |
| --- | --- |
| F-007 | **生命周期加固后 110/110 重复用例通过**。保留每例独立 Context/Executor 和显式 ActionClient 销毁；10 轮独立 pytest 进程均正常退出。历史 SIGSEGV 没有再次捕获，仍不宣称已证明其唯一根因。 |
| F-011 | 修正内置旧平地入口出生高度后，第一轮两次 EGO 冷启动、第二轮完整区域搜索均正常起飞、执行和落地。没有放宽悬停门限。 |
| F-012 | **已修复对应复现**。三个实际失败请求各重复三次通过，完全占据网格仍拒绝；原生 SUPER 完整地图变化、恢复、返航、降落及独立轨迹审计通过。 |
| F-014 | **仍可复现，未关闭**。`fuel-cancel-map-timing` 新探索返回 SOURCE_MAP_INVALID。随后两次带只读源流观察、三次仅内置诊断的冷启动均完成取消、新探索、返航、Land 和独立审计；未再捕获飞行中失效，不能用五次成功覆盖此前失败。 |
| F-015 | **修复已证实的源时间缺陷**。Gazebo 瞬时扫描移除伪造点偏移，FAST-LIO 源时间改为整体纳秒舍入；稀疏登记完整往返通过，原生 TF 对照从失败变为成功。 |
| F-016 | **已修复退出所有权和线程回收**。新增私有 MAVROS 2.14.0 运行包，六次实际 UDP 高流量 INT/TERM 退出，以及新运行库精准降落、后续多个飞行退出均通过。 |
| F-022 | **修复可靠 IMU 源的订阅 QoS**。保持原间隔门限；原取消、恢复和落地通过，289 个真值配对，RMSE 0.004347 m、最大 0.009095 m。 |
| F-023 | **修复策略查询等待造成的人工参考断档**。旧实现对连续参考序列失败，新实现通过，真实源断档负例仍拒绝；FAST-LIVO2 原绕障完整通过，1360 配对、RMSE 0.013587 m、最大 0.039581 m。 |
| F-025 | **原压力条件两次通过**。均限制四核并运行两个 75% 占空比竞争进程，原森林场景、0.3 m/s 起飞、原精度门限保持。两轮 RMSE 0.051291/0.058147 m、最大 0.090948/0.078814 m。 |
| F-026 | **原书店、住宅、赛道任务均有完整通过记录，图形环境范围见下文**。书店和住宅使用软件渲染；赛道使用隔离的匹配 NVIDIA 用户态库完成原往返、Land 和上锁。赛道软件渲染单线程仍未通过连续就绪；不将 GPU 结果写成纯软件渲染修复。没有降低帧率、分辨率、几何、物理步长或新鲜度门限。 |
| F-027 | **原地图启动场景复测通过**。森林、有屋顶仓库启动检查通过，稀疏场景完整任务通过；不将启动检查写成完整飞行。 |
| F-028 | **原医院和无屋顶仓库完整任务复测通过**。均正常往返、Land 和上锁。医院首次复测被起飞前电池反馈时序检查拦住，该失败保留；`hospital-timing-repeat` 完成原任务。 |
| F-030 | **原始录包回放、精度和真实回环审计通过**。原生重力与 Gazebo 统一为 9.8 m/s²，原始 IMU 不缩放。公共 1588 个配对 RMSE 0.035729 m、最大 0.054800 m；所有原生输出 1592 个配对 RMSE 0.035639 m、最大 0.054799 m、最大相邻位移 0.015247 m。完成 1 次原生回环，修正时锁存停止公共输出。范围是完整原录包回放，未重复现场仿真飞行。 |

### 关键原因和实现

1. **SUPER 备用轨迹**：单次静止输入快照的内部规划时刻不再随 CPU 求解耗时推进，真实工作期限仍用 steady clock。完整占据体素包络下有限回退 CIRI 种子，在新走廊中寻找第一段连续可用的轨迹时间区间；备用刹车时长独立于出发时间窗口，避免几毫秒初始时长产生巨大 jerk。保留原动态、走廊及连续碰撞校验；删除结果被丢弃的第二次诊断求解。最终 `super-braking-fixed` 的连续碰撞证明通过，最大跟踪误差约 0.06249 m，原生障碍最小间距 1.04347 m。两次中间返航失败均保留。
2. **点云/TF 源时间**：`rayShape->Update()` 在同一世界时刻取得整个扫描，CSV 只有射线顺序；旧 0～100 ms 偏移让 FAST-LIO 等待未来 IMU并错误去畸变。修改后十帧原生扫描各 20000 点，所有点偏移为零，消息戳与 timebase 一致。只读对照中 709 条 Odometry 均与原始雷达戳相差不超过 1 ns，旧实现 703 条中没有一条在 1 μs 内对应。另修复小数秒截断造成的 1 ns 提前：真实 tf2 BufferCore 对 36.421 s 查询，旧实现失败、整体纳秒舍入实现通过。见 `instant-scan-timing-comparison.json`、`tf-roundtrip-control.json`、`source-time-tests.log`。
3. **MAVROS 退出**：Router 在锁内移走端点表，锁外 close/join，避免 UDP 回调等待路由锁时死锁；UAS 先取消并 join 自有执行器线程，再拆除订阅、回调和插件。main 在共享库终结前检查 Router/UAS weak_ptr 已过期。私有库通过可执行文件 `$ORIGIN/runtime` RPATH 加载，保持已核对的 2.14.0 插件 ABI，未替换 `/opt/ros`。来源文件哈希及所用系统库 ABI 指纹在构建时验证，其他版本需重新核对。`precision-owned-runtime` 实为构建完成前旧运行库对照，不算新修复验收；`precision-runtime-installed` 才是实际新入口。
4. **定位参考缓存**：首次验证时钟策略前不收录参考；验证后允许参数服务等待期间保留有界、带原始源戳/接收时间的参考。策略租期失效时禁止配对与输出，恢复后才处理缓存；真实断档、时钟策略无效和消息过期仍按原门限拒绝。35 个针对性纯算法及 ROS 用例通过，旧/新对照见 `alignment-policy-control.log`。FAST-LIVO2 IMU 订阅使用与原生发布者匹配的 RELIABLE、深度 512，其他输入 QoS 不变。
5. **软件渲染资源**：两条 Gazebo 启动路径共用线程预算，最终缺省为 1；保留显式 `LP_NUM_THREADS`。中间的“可用核数四分之一、最多 4”方案仍在住宅和赛道失败，原代码保存为 `rendering-quarter-candidate.py`。依据 [Mesa 环境变量说明](https://docs.mesa3d.org/envvars.html#envvar-LP_NUM_THREADS)，原缺省线程数随 CPU 数增大。书店手动 4 线程、中间默认 4 线程以及最终默认 1 线程均完成完整任务（最终默认 162.59 s，实际 Gazebo 环境已核对）；住宅手动单线程与最终默认单线程各完整通过（最终默认 148.99 s），状态间隔 P95 从约 0.387 s 降至 0.166 s，源时间/墙钟比从约 0.256 提升到 0.497（不同任务阶段的整轮描述统计，非严格性能基准）；四核最终默认实际是 1 线程，进程亲和性和竞争负载证据保存在 `runtime-process-witness-pressure.json`。宿主内核 NVIDIA 模块 595.91.07、用户态库 595.99.02 不一致，系统 GLX/NVML 不能正常初始化。另从 NVIDIA 官方归档下载匹配的 595.91.07 包，仅解压到 `graphics-compat/userspace/`，以 `vendor-libs/` 隔离加载厂商库、保留系统 GLVND。没有运行安装器、改写系统驱动或重启。真实 GLX 上下文报告 RTX 5060 Ti 直接渲染，NVML 查询恢复；`racetrack-driver-compatible` 用这一环境在 214.77 s 内完成原参考任务。来源、SHA-256 与提取方式见 `graphics-compat/provenance.json`，实际进程加载库见该案例的 `runtime-process-witness.json`。复现先 source [graphics-environment.bash](artifacts/test-full-debug-v1-round2-20261024/graphics-environment.bash)，脚本核对内核版本，后续升级需重新匹配；这是本机图形环境修复，赛道纯软件路径仍有性能限制。
6. **ORB 惯性校准**：核心编译参数 `EASYFLY_ORB_GRAVITY=9.8` 随安装 target 导出，接入核对 YAML 与实际加载内核值，避免混用旧库。十秒静止原生预积分/惯性因子残差范数由旧库约 0.52545 降至约 0.01591；同一原生因子测试对旧库失败、新库通过。该因子使用 float 累积，新测试初始 0.01 界限不足以容纳自身舍入，最终设为 0.025；原飞行精度门限完全未改。完整录包回放与 `evaluation.json`、`correction-audit.json`、`native-output-audit.json` 均通过，原生输出审计没有筛除任何已输出位姿。此前关闭 fastInit 的候选处理了 5185 对图像但未完成惯性初始化、没有公共位姿，已撤回，完整失败回放保留。

### 验证口径与保留的入口错误

- 原始飞行门限、原始任务、原始真值匹配方式均保留；只做首次位姿 yaw/平移固定对齐，不拟合尺度、不重新对齐、不丢弃失误点。
- FAST-LIVO2 取消回归第一次独立审计因证据目录缺少 `acceptance-limits.json` 未运行；复制原文件后对同次飞行重审通过。初始返回码及 `evaluation_retry` 同时保留，未重飞掩盖失败。
- ORB 回放第一次证据 shell 在 ROS setup 的可选未定义变量处退出，未播放数据；移除该辅助 shell 的 `set -u` 后重新启动。未作为产品缺陷计数。
- MAVROS 初次测试暴露 RUNPATH 被环境库路径覆盖的问题，改为私有 RPATH；ORB 接入重建首次使用旧 CMake 查找缓存，清除该包路径缓存后使用本轮新内核。原构建失败日志保留。
- SUPER、ORB-SLAM3、FAST-LIVO2 来源和补丁校验均通过。最终软件汇总见 `final-test-summary.json`，SITL 与录包回放清单见 `sitl-summary.json`，任务差异与哈希见 `task-tracked.diff`、`task-files-sha256.json`，清理复核见 `final-process-check.json`。

### FUEL 间歇失效的诊断边界

失败轮在失效前后约三秒内仍收到 150 条 `/odom`（最大到达间隔约 0.0291 s）以及 23 帧非空点云/语义地图（最大间隔约 0.162 s）；不是所有定位/点云流共同停止。体素处理现已记录有效转无效时的源年龄、接收年龄、待处理数量、处理耗时和最近 TF 异常，并分别记录源发布者数量变化及仿真时钟回退；不修改原一秒失效门限、射线语义、占据膨胀或失效保护。`fuel-native-diagnostic-0..2` 按之前失败的四线程渲染配置执行，没有额外源流订阅者，三轮均通过。内置诊断捕获过起飞前/落地停流时的 TF 未来外推和接收超时，未捕获本轮飞行中失败；这些正常边界事件不能冒充 F-014 根因。

最终包级回归发现 `test_openvins_runtime_policy` 的 AST 求值夹具尚未引入新渲染环境函数，报 NameError；实际启动及原场景飞行正常。测试现使用真实 `renderer_environment`，同时核对新增线程预算和原六种后端/SLAM 开关组合。初次 CTest 日志及 `px4-runtime-policy-initial-failure.xml` 保留，修正后仅重跑该 CTest 项（2 个 pytest 用例），通过；原六种后端/SLAM 开关组合均保留。

### 最终回归汇总

以下是第二轮最终代码的功能回归，格式/lint 检查不混入功能统计。65 个独立 CTest 入口中，初次 64 个通过、1 个测试夹具失败；修正夹具后仅重跑该入口并通过。最终 306 个 JUnit 用例无失败、无错误、无跳过；CTest 与其内含 JUnit 数量不相加。原始 `final-checks-execution.json` 保留初次返回码，定向复测见 `final-px4-runtime-policy-retry.json`。

| 包 | CTest 入口 | 最终 JUnit 用例 |
| --- | ---: | ---: |
| `easyfly_mavros_runtime` | 1 | 6 |
| `easyfly_mavlink_bridge` | 4 | 19 |
| `easyfly_super_planner` | 3 | 20 |
| `easyfly_trajectory_optimizer` | 3 | 31 |
| `easyfly_gazebo_plugins` | 24 | 50 |
| `easyfly_px4_sitl` | 12 | 57 |
| `fast_lio` | 2 | 5 |
| `easyfly_fast_livo2_integration` | 2 | 19 |
| `easyfly_openvins_integration` | 11 | 77 |
| `easyfly_orbslam3_core` | 1 | 2 |
| `easyfly_orbslam3_integration` | 2 | 20 |

定位准入测试另行串行运行 10 轮，每轮 11 个用例，共 **110/110 通过**；不与包级 JUnit 重复合并。见 `localization-stress-summary.json`。

本轮共保存 **31 次 SITL 尝试和 2 次原录包回放**。SITL 初次 23 次通过、8 次失败；其中一次 FAST-LIVO2 飞行本身通过，补齐原审计配置后对同次数据重审通过，最终为 24 次通过、7 次失败。精降 `precision-owned-runtime` 是旧运行库对照，不计作新 MAVROS 验收。录包回放一轮因无惯性初始化/无输出不满足验收，另一轮重力修正后全部审计通过。以上总数包含开发中的失败候选，不作为最终缺陷关闭率。

所有案例原始结果均未被成功轮覆盖。复核 760 个已记录的 PID/启动时刻身份，无相同身份的活进程遗留；32 个私有 Xvfb 均已 wait 回收。赛道使用的宿主 `:1` 显示服务未关闭。SUPER、ORB、FAST-LIVO2 来源校验分别核对 246、633、116 个文件，均通过零模糊补丁重建校验。

### 复现与尚未关闭的边界

- 第二轮独立构建目录为 `artifacts/test-full-debug-v1-round2-20261024/build`，安装目录为其 `install`。普通软件仿真先 source `simulation-environment.bash`；赛道匹配 GPU 环境先 source `graphics-environment.bash`。实际命令、场景、原始任务与安装路径已记录在各案例 JSON。构建输出保留在各 `*-build.log` 与 colcon 日志目录。
- 最终场景复测使用 `run_single_default_scenes.py`；包级回归与十轮重复测试使用 `run_final_checks.py`，测试夹具修正后的定向复测使用 `retry_final_policy.py`。辅助脚本默认要求新的证据文件名，以免覆盖现有记录。原 acceptance-limits 未变更。
- **F-014 仍未关闭**：已有明确失败轮，新增五次冷启动成功及诊断尚未证明间歇根因。输入失效时仍触发中止；失败轮随后完成 Land、上锁与进程清理，不忽略失效消息。
- **F-007 保留历史崩溃未定因的标注**：生命周期加固和 110 次重复用例通过，不能代替原崩溃的充分因果证据。
- **F-011 与 F-026 按验证范围记录**：默认出生高度修正后，旧入口多轮实际飞行通过，但不把它认定为历史悬停失败的唯一诱因；书店/住宅最终默认软件渲染通过，赛道成功依赖本机隔离的匹配 GPU 运行库，纯软件赛道仍未达到原连续就绪要求。
- **F-030 的精度结论来自完整原录包回放**，包括所有已发出的原生位姿和真实回环审计；没有把回放写成新的一次现场飞行，也没有将 ORB 惯性观察模式接入控制。

因此，本记录提供已落地的修复、通过的原场景复测和明确的剩余问题，**不宣称原 30 项已经全部关闭**。


## 2026-10-26 软件／仿真补验结论

本轮 V 编号以 [未完成.md](未完成.md) 为准。V-15 十项范围、V-16 当前输入组合／完整栈与补充飞行、V-14 虚拟双机／两版 QGC／PTY 验收已按限定软件范围签收。当前源时间修复、全部原始失败、实际运行配置及仍未覆盖的硬件／纯 CPU 完整栈限制见[最终记录](artifacts/v01-v16-20261025/final-result.md)。此增量记录不把本文旧测试统计改写成当前全仓重跑，不关闭车尾漏检、许可或证据不足的历史异常。
