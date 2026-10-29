# EasyFly 全功能仿真测试报告 v1

状态：本轮测试执行与报告整理已完成；**整体未通过验收**。完成时间：2026-10-24T09:56:23+08:00。

当前工作区的已实现功能已按软件套件、模拟接口、安装消费、桌面与端到端仿真逐项检查；失败、阻塞及未实现部分均保留，不把“执行结束”写成“全部功能通过”。

- 构建：补建后 **111/111 包通过**；首次构建问题另列。
- 首轮 CTest 功能入口：**269 通过、5 失败**；JUnit、静态检查和补测使用各自口径，详见下文。
- 端到端仿真矩阵：**154 次执行尝试**，其中 110 通过、36 失败或审计失败、5 环境/前置阻塞、3 测试侧错误；另有 9 个登记入口变体因夹具阻塞未执行。部分在飞行前终止；首次失败与补测分别计数，不能视为互不重复的功能数。
- 共记录 **30 项问题与异常现象（F-001～F-030）**，包含确定缺陷、间歇性运行异常及测试/构建问题，并非都已证明同一层面的根因。
- 9 个骨架包明确标为 NOT_IMPLEMENTED。全部测试限于软件与仿真，未进行真机测试；产品实现未为取得通过而修改。

- 范围：当前工作区全部已实现功能的软件测试、模拟接口与 PX4/Gazebo SITL；不连接、不控制、不验证真机。
- 开始时间：2026-10-23，Asia/Shanghai。
- 源码基线：包含测试开始前已有的未提交及未跟踪改动，详见 `artifacts/test-full-v1-20261023/baseline/`。
- 包清单：当前发现 111 个包，数量不代表已实现功能数。
- 证据目录：`artifacts/test-full-v1-20261023/`。
- 执行方式：独立构建/安装目录；软件集成及 SITL 串行执行；保留首次失败。

本文分别记录构建、组件与接口、端到端仿真、异常恢复、覆盖缺口以及复现命令；历史验收结果不计入本轮通过数。

## 构建与软件测试结果

| 检查 | 当前结果 | 原始证据 |
| --- | --- | --- |
| Python 语法 | 455 文件通过 | `artifacts/test-full-v1-20261023/python-syntax.json` |
| 构建脚本与 B-spline 度量工具 | 7 项通过 | `artifacts/test-full-v1-20261023/standalone-pytest.xml` |
| FAST-LIVO2 / ORB-SLAM3 / GCOPTER / SUPER 来源重建 | 4 组通过 | `artifacts/test-full-v1-20261023/source-verification.json` |
| FUEL 来源重建 | 143 文件通过 | `artifacts/test-full-v1-20261023/fuel-source/result.json` |
| 公共数学/坐标/CRC/时钟头文件 | 15 项，14 通过、1 失败 | `artifacts/test-full-v1-20261023/header-contracts.console.log` |
| 补充 RC 转发及里程计接口 | 14 项，7 通过、7 失败 | `artifacts/test-full-v1-20261023/uncovered-interfaces/result.json` |
| ORB-SLAM3 安装后独立消费 | 通过：实际投影/特征提取、无源码 include、动态库解析 | `artifacts/test-full-v1-20261023/orbslam3-consumer/result.json` |
| 既有算法/API 安装后独立消费 | 13 组通过，包含 GCOPTER 实际优化；无源码 include，动态库均解析 | `artifacts/test-full-v1-20261023/consumers/summary.json` |
| 已安装场景离线校验 | 14 登记场景资源/许可/登记均通过；6 草稿正确保留不完整状态 | `artifacts/test-full-v1-20261023/scene-catalog.json` |

首轮构建完成 103 包（含 SDK 两包），`sophus`、`ov_init` 失败，6 包因依赖未处理。
`ov_init` 是本轮初始环境未加载私有 Ceres 所致；系统缺少 Ceres，而已有依赖位于
`artifacts/new-algorithms/openvins-build-20261019/dependencies/root/usr`。
已在本轮环境中登记该前缀并进行补建，未安装或升级系统依赖。
首轮机器摘要：`artifacts/test-full-v1-20261023/initial-build-summary.json`。

补建已经完成，111/111 包构建成功；首轮构建失败仍单独保留。
全量软件套件串行运行 20 分 44 秒，选择 106 个包（5 个设备 SDK/真机驱动包仅编译，不运行设备测试）。其中 89 包实际提供 CTest 入口，71 包提供功能入口；不能把选中包数当成功能测试包数。
CTest 功能入口 274 个：269 通过、5 失败；静态检查入口 328 个：249 通过、79 失败。
JUnit 功能记录为 2173 通过、4 断言失败、1 错误占位、13 跳过、4 禁用。
入口数与子用例记录数不能相加；原生 EGO unittest 失败只有 CTest 记录，崩溃测试的错误占位也不代表 11 个参数化用例全部执行。
汇总：`artifacts/test-full-v1-20261023/software-summary.json`。SITL 与补测的最终记录见后文。

## 按功能查看覆盖范围

| 功能 | 本轮实际验证 | 结论边界 |
| --- | --- | --- |
| 构建、来源、安装导出 | 111 包新构建，源补丁重建、安装后独立消费者与实际原生优化 | 构建通过不能替代功能通过；Sophus 首构缺目录单列 |
| 连接、参数、通信和时钟 | 连接管理、参数服务、串口模拟端点、MAVLink/RC 转发、时序/重放/失效接口 | Livox/RealSense 设备驱动只编译，不连接真机 |
| 图像感知 | YOLO CPU/CUDA 实际推理；颜色线、椭圆、AprilTag、标定/投影软件及 ROS 套件 | 仿真或测试图像，不代表任意光照/实物识别率 |
| 目标跟踪与预测 | SORT、CSRT、多目标、目标 ID 融合、运动预测及原生人物实际跟随/取消 | 人物 SITL 为静止目标；没有长期移动人物追踪验收 |
| 点云与地图 | 滤波、地面分割、障碍关联、语义/占据地图；实际导航和停流保护 | 坐标变换与无效输入发现 F-003/F-004；点云失效另列 |
| 全局/局部/三维规划 | A*、EGO、combined、GCOPTER、SUPER、FUEL 实际算法与端到端矩阵 | SUPER 地图变化失败；FUEL 首轮恢复异常、复测通过 |
| 区域搜索 | 几何路径、任务联动、实际搜索和递归取消 | 几何覆盖不等于传感器覆盖；部分旧入口用例受悬停失败阻塞 |
| 定位 | FAST-LIO 主链；OpenVINS、FAST-LIVO2、ORB-SLAM3 的真实传感器/估计器、真值精度和故障测试 | 不向估计器注入真值；不同模式、场景和失败轮分开记录 |
| 飞行控制与任务编排 | 起飞、航点、悬停、降落、所有权、取消/超时、接管；C++/Python SDK 和已安装模板 | 软件接口测试与实际 SITL 分开计数；失败不能被清理成功覆盖 |
| 精准降落 | 视觉对准、丢失恢复/持久丢失、取消、AUTO.LAND/RTL/POSCTL 接管及延迟/未知应答 | 预期异常通过表示保护正确；首轮 MAVROS 退出崩溃保留 |
| 电池与安全拒绝 | 三轮 SITL 任务电池遥测；低电量/无效电池/外部接管的模拟 ROS 行为 | 三轮电池任务不是实际低电量故障飞行；资源测试未放宽保护门限 |
| 桌面与平台服务 | 总览/感知/任务、参数、导入/审阅/启动/取消、回放/录制、RViz/QGC 管理；真实 ENOSPC | 17 个初始跳过/禁用均补测；在私有显示环境查看渲染截图 |
| 仿真场景与传感器 | 20 个登记/草稿资源状态、11 基础场景启动、参考任务校验；逐场景飞行进度另列 | 草稿不完整正确保留；安装 Livox CSV 依赖源路径的问题独立证实 |
| 退出、清理、资源 | 子进程身份/终态/遗留检查；私有 PID 命名空间清理脚本测试；持续运行/CPU 压力 | 一键清理有遗漏；没有长时、任意负载或机载硬件性能保证 |
| 旧独立适配与未实现包 | ego_to_mavros 13 项模拟接口补测；逐包核对实现状态 | 旧适配 4 项失败；9 个骨架标为 NOT_IMPLEMENTED，不能算功能成功 |

精确用例与证据见后续明细。各类计数存在覆盖关系，不相加作总功能数。

## 补测与覆盖边界

- 初始软件套件的 13 个跳过、4 个禁用均为桌面环境或显式开关限制；本轮已单独补测全部 17 项：12 项实际 RViz 渲染、2 项真实 RViz/QGC 外部工具、2 项桌面管理工具生命周期和录制、1 项真实 ENOSPC。
- RViz 又在私有 Xvfb 显示服务中重复 12 项，截图经人工查看。宿主显示器第一次截到其他前台窗口的图片未用于 UI 结论，记录于 `ui-evidence/invalid-host-capture.json`；最终图片见 `ui-evidence/`。实际桌面总览、感知和任务页面也已检查。
- ENOSPC 使用私有用户/挂载命名空间中的 8 MiB tmpfs，验证写满失败处理及退出，未填满宿主磁盘。证据：`enospc/result.json`、`enospc/gtest.xml`。
- YOLO CPU 与 CUDA 均实际运行并检出 5 个目标，验证 provider 日志与基准结果；`detection-provider.xml`、`detection-provider-temp/`。
- 13 组既有算法/API 独立消费、FAST-LIVO2 依赖消费、ORB-SLAM3 特征/投影消费、FUEL 实际原生规划与安装后消费均通过；消费程序使用安装目录，不使用工作区源码 include。SUPER 独立消费也已使用本轮 smoke 飞行生成的原生请求实际通过，见 `super-consumer/result.json`。
- ORB 空 Atlas 原生启停/保存通过；IMU 守卫 6/6 补测通过。空 Atlas 保存不能替代实际建图、闭环、地图重用验证，后者另列 SITL 结果。
- 11 个登记基础场景均通过实际 UI 后端启动与正常关闭，另有 22 次 C++/Python 参考任务 validate-only 通过。地面启动与离线验证不计为参考任务飞行通过。
- 设备 SDK/驱动 5 包编译成功，但遵照仿真限定不执行真实 Livox/RealSense 设备测试。串口测试使用软件模拟端点；未声称传感器、遥控器或飞控硬件通过。
- 9 个包只有骨架，状态为 **NOT_IMPLEMENTED**：`easyfly_depth_estimation`、`easyfly_gps_receiver`、`easyfly_mesh_comm`、`easyfly_swarm_planner`、`easyfly_udp_protocol`、`easyfly_vehicle_reid`、`easyfly_video_recorder`、`easyfly_video_streamer`、`easyfly_web`。这些包的构建/lint 通过不代表功能存在。桌面自有录制功能单独测试，不与空的 video_recorder 包混淆。
- 静态检查 79 个失败入口产生 3229 条 JUnit 诊断，不等于 3229 个独立运行缺陷；包含格式、版权、文档和上游遗留代码问题。cppcheck 的 28 条诊断中，26 条为解析问题，1 条为旧轨迹头文件缺少返回值（F-005），1 条为 importer 的生命周期警告。后者已核对调用/收尾路径，未发现失效后解引用的实际路径，既有导入器测试通过，尚不能认定运行缺陷；诊断与复核见 `cppcheck-diagnostics.json`、`cppcheck-triage.json`。

所有上述相对证据路径以 `artifacts/test-full-v1-20261023/` 为根。全包明细见本文末表，同时保留机器生成的 `package-coverage.md`。

## 真实感知跟随补测

`follow-full-registered-person` 在登记稀疏场景中加入本轮独立的原生静止人物模型，使用已安装人物网格/纹理、真实 Gazebo RGB/深度和原始 `/target_positions`。没有注入目标消息或修改图像。它通过了原项目的实际跟随移动、可见反馈、精确 UUID 取消、子 waypoint 取消、禁止迟到子任务、恢复悬停、正常 Land 和上锁断言。

目标由 `easyfly_pixel_to_3d` 发布，选择的三维位置约 `(5.264,0.054,0.920)` m；无人机水平移动超过 0.15 m 后取消，父/子动作均返回 CANCELED。登记入口实际包含 27 个进程，全部正常退出、无强杀和遗留；用独立角色表校正原脚本硬编码的 24 进程判断，飞行断言未改。证据：`follow-registered-summary.json`、`sitl/follow-full-registered-person/registered-evaluation.json`。

原旧入口两轮仍因 F-011 未进入跟随，保留失败。此补测证明静止真实感知目标的跟随/取消链路，不冒充对移动人物的长期追踪验收。

## 登记场景参考任务结果

11 个登记基础场景均执行了一次原项目参考任务检查：**3 通过、8 失败**。此前 11/11 地面 `base_sim` 启停和 22 次离线任务校验通过，与这里的 `navigation_dev` 飞行结果是不同检查，不能互相替代。

| 本轮结果 | 场景 | 实际到达阶段 |
| --- | --- | --- |
| 完整通过 | 办公室、基础走廊、城市街区 | 起飞、两个真实导航子任务、正常降落、连续落地后恢复、上锁、会话关闭 |
| 地面时序检查失败 F-026 | 书店、住宅、赛道 | 未发出飞行任务；前两例连续状态新鲜度不足，赛道心跳检查失败 |
| 地图启动失败 F-027 | 森林、稀疏基础场景、有屋顶仓库 | 75 秒内地图未就绪；未发出飞行任务 |
| 飞行中止 F-028 | 医院、无屋顶仓库 | 已起飞，导航输入过期或无效后任务中止，最后状态仍解锁 |

通过场景两段终点误差分别为：办公室 0.08591/0.08345 m，走廊 0.07169/0.07119 m，城市街区 0.08666/0.08676 m。这里按原项目 **UAVState/odom 导航终点**验收，不能与定位章节对原生 Gazebo 真值的误差混用；场景另有起飞前/落地后的世界坐标对齐检查。

11 轮均为 24/24 子进程正常退出、无强杀或遗留。该退出结论不替代落地；医院和无屋顶仓库明确未完成落地验收。每个场景只有本轮一轮参考任务检查，不据此估计统计成功率。证据：[场景阶段汇总](artifacts/test-full-v1-20261023/scene-stage-summary.json)，各轮详见后文执行表。

## 定位持续运行阶段记录

- OpenVINS `openvins-long-run` 已完成 600 秒仿真源时间的持续悬停、前后各两段导航及正常降落，上锁且 28/28 进程正常退出。总墙钟耗时 1056.5 秒，实际仿真/墙钟速度比约 0.685。
- 独立真值审计：10,265 个匹配、0 个未匹配；位置 RMSE 0.02576 m、最大误差 0.14643 m、最大 yaw 误差 0.00302 rad。这里只审计 yaw，不能称为完整三轴姿态误差。
- 预先冻结的原生估计器资源门限全部通过：峰值 RSS 291.68 MiB（门限 512）、后段相对初段 RSS 中位数增加 13.21 MiB（门限 64）、平均 CPU 0.110 核（门限 1）、峰值 36 线程（门限 64）。RSS 包含共享映射，不能当作整栈实际物理内存。
- 证据：`sitl/openvins-long-run/evaluation.json`、`resource-evaluation.json`、`resources.json`、`openvins-resource-limits.json`。仿真电池耗尽时间在运行前设为 1800 秒，以支持持续观察；低电量保护门限未修改。
- FAST-LIVO2 `fastlivo-long-run` 的 120 秒仿真源时间持续悬停、前后四段导航与正常降落全部通过；墙钟耗时 310.0 秒。位置 RMSE 0.00591 m、最大误差 0.02000 m、完整姿态最大误差 0.00393 rad，1962 个匹配、0 个缺失。此项使用原生仿真特征板，不能推广为任意自然环境精度；本轮没有为 FAST-LIVO2 声称已通过独立内存增长门限。
- FAST-LIVO2 的同轮资源观察：估计器峰值 RSS 1118.97 MiB，尾部相对初段中位 RSS 增加 741.91 MiB，平均 CPU 0.371 核；窗口包含初始化和建图，不能仅凭增长认定内存泄漏，也不能声称长期稳定。证据：`sitl/fastlivo-long-run/resources.json`。
- ORB-SLAM3 `orbslam3-long-run` 完成 120 秒仿真源时间悬停及前后四段导航，正常落地/退出；位置 RMSE 0.01934 m、最大误差 0.05491 m、完整姿态最大误差 0.01318 rad，5901 个匹配、0 个缺失。原生地图实际保存 4 个关键帧、1466 个地图点，本轮闭环计数为 0，不能代替闭环专项验证。估计器峰值 RSS 673.52 MiB、尾部相对初段中位增加 2.35 MiB、平均 CPU 0.678 核，仅为本机短时观察。见 `orb-long-stage-summary.json`。
- 低电量中止并只请求一次 RTL、无效电池超时和外部接管，已在任务 FSM 的模拟 ROS 接口套件中通过。`battery-three-cycles` 验证三轮实际 SITL 任务与电池遥测；它不是低电量故障飞行试验。

## ORB 地图保存与重定位专项

- `orbslam3-natural-map-seed` 使用原生照片纹理物理墙及真实 Gazebo RGB-D，实际飞行建图、往返和降落通过。保存的本轮 Atlas 为 4,198,349 字节，19 个关键帧、2035 个地图点；位置 RMSE 0.00979 m、最大误差 0.02546 m，完整姿态最大误差 0.00594 rad，1408 个匹配、0 个缺失。闭环计数为 0，此项不作为回环证据。
- 重定位首次因原生 Gazebo 查询工具退出异常受阻（F-029）。同参数冷启动补测加载上述新地图，初始旧图重定位计数为 1；通过原生遮挡模型遮住视野后，活动导航中止、指令流安静，新导航被拒绝。
- 物理移开遮挡后，重定位计数变为 2，map_id 仍为 0、19 个关键帧和 2035 个地图点保持不变。修正 epoch 从 1 到 2，公共位姿计数冻结在 557，reset 保持锁存，恢复视野后也没有自动重新准入导航。原始传感器持续输入；遮挡前、遮挡中、恢复后三张原始截图已实际查看，未编辑或反馈给估计器。
- 重定位补测原断言及独立精度审计通过：550 个匹配、0 个缺失，位置 RMSE 0.01036 m、最大误差 0.02775 m。最终正常 Land/上锁，23/23 主栈进程正常退出、无强杀/遗留；种子建图轮也为 23/23 正常退出。
- `orbslam3-live-inertial-loop` 实际完成 1 次 RGB-D/IMU 回环，保存 99 个关键帧、7272 个地图点，原始 RGB-D/IMU 与回环后输出锁存断言通过；但位置精度严重超限（F-030），**整体审计失败**。主栈、观察器、录制器正常退出，FAST-LIO 控制下正常落地。
- 范围为受控原生照片墙下的 RGB-D 地图重用、重定位隔离与独立 RGB-D/IMU 回环观察；不代表任意场景地图重用或 RGB-D/IMU 导航控制通过。

证据：`sitl/orbslam3-natural-map-seed/`、`sitl/orbslam3-map-relocalization-cold-repeat/`，各含原始 `result.json`、`evaluation.json`、Atlas 及输入指纹；视觉复核见 `orb-relocalization-visual-review.json`。

## 三维规划器 SITL 阶段记录

首轮 GCOPTER 10/10 通过；SUPER 9/10 通过、地图变化失败；FUEL 首轮 9 项通过、取消后恢复失败、不可达用例配置错误。FUEL 的取消冷启动补测与正确封闭场景补测均已通过，首次结果仍保留。上述全部规划器用例最终 armed=false、启动进程均正常退出、无强杀/遗留。

| 算法/场景 | 运行 | 独立审计 | 最小原生障碍间距 m | 最大源时间跟踪误差 m |
| --- | --- | --- | --- | --- |
| `gcopter-backend-exit` | 通过 | 通过 | — | 0.1352 |
| `gcopter-cancel` | 通过 | 通过 | — | 0.1479 |
| `gcopter-channel` | 通过 | 通过 | 0.8964 | 0.1411 |
| `gcopter-map-change` | 通过 | 通过 | 0.8892 | 0.1665 |
| `gcopter-map-loss` | 通过 | 通过 | — | 0.1368 |
| `gcopter-narrow` | 通过 | 通过 | 0.8906 | 0.1408 |
| `gcopter-no-solution` | 通过 | 通过 | — | — |
| `gcopter-over` | 通过 | 通过 | 0.9244 | 0.1368 |
| `gcopter-smoke` | 通过 | 通过 | — | 0.1393 |
| `gcopter-under` | 通过 | 通过 | 1.1002 | 0.0958 |
| `super-backend-exit` | 通过 | 通过 | — | 0.0249 |
| `super-cancel` | 通过 | 通过 | — | 0.1349 |
| `super-channel` | 通过 | 通过 | 1.0871 | 0.0667 |
| `super-map-change` | 失败 | 前阶段失败，未执行 | — | — |
| `super-map-loss` | 通过 | 通过 | — | 0.1294 |
| `super-narrow` | 通过 | 通过 | 1.0717 | 0.0490 |
| `super-no-solution` | 通过 | 通过 | — | — |
| `super-over` | 通过 | 通过 | 1.0241 | 0.0538 |
| `super-smoke` | 通过 | 通过 | — | 0.0523 |
| `super-under` | 通过 | 通过 | 1.0510 | 0.0593 |
| `fuel-bootstrap` | 通过 | 通过 | — | 0.0000 |
| `fuel-cancel` | 失败 | 前阶段失败，未执行 | — | — |
| `fuel-complete` | 通过 | 通过 | 1.0703 | 0.1338 |
| `fuel-land-takeover` | 通过 | 通过 | 2.5910 | 0.1042 |
| `fuel-map-loss` | 通过 | 通过 | 0.9251 | 0.1348 |
| `fuel-no-fly` | 通过 | 通过 | 1.1535 | 0.1528 |
| `fuel-timeout` | 通过 | 通过 | 2.1821 | 0.0933 |
| `fuel-unreachable` | 测试配置错误 | 前阶段失败，未执行 | — | — |
| `fuel-warehouse-complete` | 通过 | 通过 | 1.4342 | 0.1693 |
| `fuel-worker-exit` | 通过 | 通过 | 2.6011 | 0.1059 |
| `fuel-worker-timeout` | 通过 | 通过 | 1.4786 | 0.0925 |

证据：`artifacts/test-full-v1-20261023/sitl/<用例>/result.json` 与 `independent-evaluation.json`。无解/故障用例的通过表示按预期停止或拒绝任务，不表示导航成功。FUEL bootstrap 只验证地面启动，不计为完成探索。

<!-- SIMULATION_RESULTS_BEGIN -->

## 端到端仿真执行结果

以下按独立执行记录统计，包含首次失败和补测，不能作为互不重复的产品功能数量，也不与 CTest/JUnit 数相加。

| 功能域 | 通过 | 失败/审计失败 | 环境或前置阻塞 | 测试侧错误 | 未执行 | 运行中 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 电池/多轮任务 | 1 | 0 | 0 | 0 | 0 | 0 |
| FAST-LIVO2 | 12 | 3 | 1 | 0 | 0 | 0 |
| 真实感知跟随 | 1 | 2 | 0 | 0 | 0 | 0 |
| FUEL | 11 | 1 | 0 | 1 | 0 | 0 |
| GCOPTER | 10 | 0 | 0 | 0 | 0 | 0 |
| 导航/搜索/任务编排 | 11 | 15 | 0 | 1 | 9 | 0 |
| OpenVINS | 12 | 4 | 0 | 0 | 0 | 0 |
| ORB-SLAM3 | 13 | 1 | 1 | 0 | 0 | 0 |
| 精准降落/控制权交接 | 27 | 1 | 3 | 1 | 0 | 0 |
| 登记场景参考飞行 | 3 | 8 | 0 | 0 | 0 | 0 |
| SUPER | 9 | 1 | 0 | 0 | 0 | 0 |

### 定位误差与持续运行

误差来自本轮原生 Gazebo 真值旁路观察，固定一次初始对齐，不使用轨迹拟合、时间平移或真值反馈。OpenVINS 审计为位置与 yaw；FAST-LIVO2/ORB 审计含完整姿态，不能把 yaw 指标等同完整姿态验收。

| 用例 | 独立审计 | 位置 RMSE m | 最大位置误差 m | 最大姿态/yaw rad |
| --- | --- | ---: | ---: | ---: |
| `openvins-cpu-pressure-cold-repeat` | 失败 | 0.15835 | 1.07717 | 0.00485 |
| `openvins-map-bootstrap` | 通过 | 0.04598 | 0.13279 | 0.00386 |
| `fastlivo-dynamic` | 通过 | 0.00483 | 0.01299 | 0.00338 |
| `openvins-cpu-pressure` | 失败 | 0.34882 | 0.94643 | 0.00279 |
| `fastlivo-cancel-private-display-repeat` | 通过 | 0.00385 | 0.01057 | 0.00501 |
| `fastlivo-backend-exit-private-display-repeat` | 通过 | 0.00453 | 0.00912 | 0.00183 |
| `fastlivo-obstacle-private-display-repeat` | 失败 | 0.00998 | 0.02569 | 0.00349 |
| `openvins-mission-astar` | 通过 | 0.07778 | 0.40440 | 0.00564 |
| `openvins-mission-ego` | 通过 | 0.04451 | 0.10228 | 0.00301 |
| `openvins-mission-combined` | 通过 | 0.03612 | 0.07805 | 0.00394 |
| `openvins-cancel` | 通过 | 0.04364 | 0.12147 | 0.00456 |
| `openvins-backend-exit` | 通过 | 0.04543 | 0.08425 | 0.00407 |
| `openvins-visual-loss` | 通过 | 0.09468 | 0.51939 | 0.00321 |
| `openvins-clock-reset` | 通过 | 0.03469 | 0.09210 | 0.00390 |
| `openvins-map-loss` | 通过 | 0.03539 | 0.07547 | 0.00449 |
| `openvins-localization-loss` | 通过 | 0.04164 | 0.09145 | 0.00410 |
| `openvins-obstacle` | 通过 | 0.06323 | 0.09529 | 0.00731 |
| `openvins-long-run` | 通过 | 0.02576 | 0.14643 | 0.00302 |
| `fastlivo-mission-astar` | 通过 | 0.00490 | 0.01250 | 0.00220 |
| `fastlivo-mission-ego` | 通过 | 0.00450 | 0.01651 | 0.00455 |
| `fastlivo-mission-combined` | 通过 | 0.01031 | 0.01900 | 0.00655 |
| `fastlivo-visual-loss` | 通过 | 0.00435 | 0.01049 | 0.00153 |
| `fastlivo-clock-reset` | 通过 | 0.00524 | 0.01171 | 0.00695 |
| `fastlivo-camera-loss` | 通过 | 0.00378 | 0.00753 | 0.00325 |
| `fastlivo-imu-loss` | 通过 | 0.00567 | 0.01324 | 0.00319 |
| `fastlivo-lidar-loss` | 通过 | 0.00360 | 0.01306 | 0.00348 |
| `fastlivo-long-run` | 通过 | 0.00591 | 0.02000 | 0.00393 |
| `orbslam3-mission-astar` | 通过 | 0.01174 | 0.06650 | 0.01475 |
| `orbslam3-mission-ego` | 通过 | 0.01183 | 0.03614 | 0.01051 |
| `orbslam3-mission-combined` | 通过 | 0.01331 | 0.04075 | 0.01097 |
| `orbslam3-cancel` | 通过 | 0.01220 | 0.04601 | 0.01107 |
| `orbslam3-backend-exit` | 通过 | 0.01258 | 0.04010 | 0.00786 |
| `orbslam3-visual-loss` | 通过 | 0.01135 | 0.03024 | 0.00815 |
| `orbslam3-clock-reset` | 通过 | 0.01199 | 0.04241 | 0.01059 |
| `orbslam3-map-loss` | 通过 | 0.01226 | 0.03847 | 0.00944 |
| `orbslam3-camera-loss` | 通过 | 0.01383 | 0.03402 | 0.01015 |
| `orbslam3-depth-loss` | 通过 | 0.01271 | 0.04823 | 0.01199 |
| `orbslam3-long-run` | 通过 | 0.01934 | 0.05491 | 0.01318 |
| `orbslam3-map-relocalization-cold-repeat` | 通过 | 0.01036 | 0.02775 | 0.00544 |
| `orbslam3-natural-map-seed` | 通过 | 0.00979 | 0.02546 | 0.00594 |
| `orbslam3-live-inertial-loop` | 失败 | 16.49634 | 18.56398 | 0.06306 |

### 全部仿真执行明细

“上锁”仅取该探针实际保存的最终状态；“—”表示未提供该字段，不能推定飞行已结束。预期故障用例通过表示保护/停止/控制权断言满足，并不表示全部采用正常降落路径。每个用例的完整命令、输出、结果和审计均在证据目录，未用父进程退出码替代子进程检查。

| 用例 | 结果 | 耗时 s | 最终上锁 | 说明/证据 |
| --- | --- | ---: | --- | --- |
| `navigation-cancel-cold-repeat` | 失败 | 48.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-cancel-cold-repeat/result.json) |
| `navigation-search-cold-repeat` | 通过 | 104.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-search-cold-repeat/result.json) |
| `navigation-mission-nav-land-cold-repeat` | 失败 | 56.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-nav-land-cold-repeat/result.json) |
| `navigation-mission-search-cancel-cold-repeat` | 通过 | 91.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-search-cancel-cold-repeat/result.json) |
| `navigation-mission-search-timeout-cold-repeat` | 失败 | 57.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-search-timeout-cold-repeat/result.json) |
| `navigation-mission-search-land-cold-repeat` | 失败 | 58.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-search-land-cold-repeat/result.json) |
| `follow-full-cold-repeat` | 失败 | 45.9 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/follow-full-cold-repeat/result.json) |
| `precision-nominal-cold-repeat` | 通过 | 44.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/precision-nominal-cold-repeat/result.json) |
| `precision-external-posctl-descent-cold-repeat` | 前置条件未满足 | 42.9 | — | 原夹具缺少有效手动输入，PX4 拒绝 POSCTL；提供居中输入的独立补测另列。；[证据](artifacts/test-full-v1-20261023/sitl/precision-external-posctl-descent-cold-repeat/result.json) |
| `navigation-happy-astar` | 通过 | 65.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-happy-astar/result.json) |
| `navigation-happy-ego` | 失败 | 46.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-happy-ego/result.json) |
| `navigation-happy-combined` | 通过 | 78.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-happy-combined/result.json) |
| `navigation-happy-ego-cold-repeat` | 失败 | 46.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-happy-ego-cold-repeat/result.json) |
| `navigation-cancel` | 失败 | 47.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-cancel/result.json) |
| `navigation-map-loss` | 通过 | 61.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-map-loss/result.json) |
| `navigation-mission` | 通过 | 69.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-mission/result.json) |
| `navigation-search` | 失败 | 46.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-search/result.json) |
| `navigation-mission-nav-cancel` | 通过 | 73.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-nav-cancel/result.json) |
| `navigation-mission-nav-timeout` | 通过 | 75.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-nav-timeout/result.json) |
| `navigation-mission-nav-land` | 失败 | 56.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-nav-land/result.json) |
| `navigation-mission-search-cancel` | 失败 | 57.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-search-cancel/result.json) |
| `navigation-mission-search-timeout` | 失败 | 56.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-search-timeout/result.json) |
| `navigation-mission-search-land` | 失败 | 57.0 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-mission-search-land/result.json) |
| `follow-full` | 失败 | 45.9 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/follow-full/result.json) |
| `battery-three-cycles` | 通过 | 139.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/battery-three-cycles/result.json) |
| `precision-nominal` | 失败 | 49.4 | 是 | F-016：实际精准降落并上锁；MAVROS 退出 -11，整轮判失败。；[证据](artifacts/test-full-v1-20261023/sitl/precision-nominal/result.json) |
| `precision-mission` | 通过 | 54.8 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/precision-mission/result.json) |
| `precision-loss-recover` | 通过 | 43.8 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/precision-loss-recover/result.json) |
| `precision-loss-persistent` | 通过 | 34.3 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-loss-persistent/result.json) |
| `precision-cancel-descent` | 通过 | 33.0 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-descent/result.json) |
| `precision-cancel-handoff` | 通过 | 41.1 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-handoff/result.json) |
| `precision-external-posctl-descent` | 前置条件未满足 | 43.7 | — | 原夹具缺少有效手动输入，PX4 拒绝 POSCTL；提供居中输入的独立补测另列。；[证据](artifacts/test-full-v1-20261023/sitl/precision-external-posctl-descent/result.json) |
| `precision-external-rtl-handoff` | 通过 | 43.1 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-external-rtl-handoff/result.json) |
| `navigation-ego-legacy-observed` | 失败 | 45.9 | — | F-011：起飞后稳定悬停门限未满足，目标功能尚未执行；原始失败保留。；[证据](artifacts/test-full-v1-20261023/sitl/navigation-ego-legacy-observed/result.json) |
| `navigation-ego-registered-observed` | 通过（测试进程表适配） | 84.6 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-ego-registered-observed/result.json) |
| `follow-full-registered-person` | 通过（测试进程表适配） | 65.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/follow-full-registered-person/result.json) |
| `fuel-cancel-cold-repeat` | 通过 | 117.4 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-cancel-cold-repeat/result.json) |
| `fuel-unreachable-sealed` | 通过 | 87.3 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-unreachable-sealed/result.json) |
| `openvins-dynamic-initialization-motion` | 失败 | 161.0 | 是 | F-024：增加实际起飞/平移运动后，90 秒仍未初始化且零位姿；正常落地及 21 进程清理。；[证据](artifacts/test-full-v1-20261023/sitl/openvins-dynamic-initialization-motion/result.json) |
| `openvins-cpu-pressure-cold-repeat` | 审计失败 | 209.6 | 是 | F-025：四核+双竞争下任务与退出通过；独立最大位置误差 1.077 m 超过 1 m 门限，整体审计失败。; sitl/openvins-cpu-pressure-cold-repeat/evaluation.json: position_max；[证据](artifacts/test-full-v1-20261023/sitl/openvins-cpu-pressure-cold-repeat/result.json) |
| `openvins-dynamic` | 失败 | 92.7 | 是 | F-024：观测配置未完成初始化，短暂 input_stale 后中止；正常落地与 21 进程清理。；[证据](artifacts/test-full-v1-20261023/sitl/openvins-dynamic/result.json) |
| `openvins-map-bootstrap` | 通过 | 119.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-map-bootstrap/result.json) |
| `fastlivo-dynamic` | 通过 | 71.8 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-dynamic/result.json) |
| `openvins-cpu-pressure` | 失败 | 191.3 | 是 | F-025：四核限制下对齐不连续；准备阶段失败，两个竞争进程未启动；正常落地及清理。; sitl/openvins-cpu-pressure/evaluation.json: actual_flight_passed; sitl/openvins-cpu-pressure/evaluation…；[证据](artifacts/test-full-v1-20261023/sitl/openvins-cpu-pressure/result.json) |
| `fastlivo-cancel-private-display-repeat` | 通过 | 64.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-cancel-private-display-repeat/result.json) |
| `fastlivo-backend-exit-private-display-repeat` | 通过 | 64.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-backend-exit-private-display-repeat/result.json) |
| `fastlivo-obstacle-private-display-repeat` | 失败 | 103.0 | 是 | F-023：已真实绕行，中途 aligned_output_stale 导致中止；几何审计通过但整轮仍失败，正常落地。; sitl/fastlivo-obstacle-private-display-repeat/evaluation.json: actual_runtime_passed；[证据](artifacts/test-full-v1-20261023/sitl/fastlivo-obstacle-private-display-repeat/result.json) |
| `openvins-mission-astar` | 通过 | 110.3 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-mission-astar/result.json) |
| `openvins-mission-ego` | 通过 | 136.3 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-mission-ego/result.json) |
| `openvins-mission-combined` | 通过 | 141.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-mission-combined/result.json) |
| `openvins-cancel` | 通过 | 104.4 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-cancel/result.json) |
| `openvins-backend-exit` | 通过 | 97.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-backend-exit/result.json) |
| `openvins-visual-loss` | 通过 | 101.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-visual-loss/result.json) |
| `openvins-clock-reset` | 通过 | 111.6 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-clock-reset/result.json) |
| `openvins-map-loss` | 通过 | 97.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-map-loss/result.json) |
| `openvins-localization-loss` | 通过 | 98.3 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-localization-loss/result.json) |
| `openvins-obstacle` | 通过 | 252.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-obstacle/result.json) |
| `openvins-long-run` | 通过 | 1056.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/openvins-long-run/result.json) |
| `fastlivo-mission-astar` | 通过 | 66.8 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-mission-astar/result.json) |
| `fastlivo-mission-ego` | 通过 | 91.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-mission-ego/result.json) |
| `fastlivo-mission-combined` | 通过 | 91.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-mission-combined/result.json) |
| `fastlivo-cancel` | 失败 | 52.0 | 是 | F-022：初始化时 imu_source_gap；没有执行取消，随后正常落地上锁。；[证据](artifacts/test-full-v1-20261023/sitl/fastlivo-cancel/result.json) |
| `fastlivo-backend-exit` | 环境阻塞 | 3.3 | — | 宿主 GLX 建立失败，Gazebo 启动即退出；未进入故障注入。；[证据](artifacts/test-full-v1-20261023/sitl/fastlivo-backend-exit/result.json) |
| `fastlivo-visual-loss` | 通过 | 60.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-visual-loss/result.json) |
| `fastlivo-clock-reset` | 通过 | 71.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-clock-reset/result.json) |
| `fastlivo-camera-loss` | 通过 | 63.3 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-camera-loss/result.json) |
| `fastlivo-imu-loss` | 通过 | 64.2 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-imu-loss/result.json) |
| `fastlivo-lidar-loss` | 通过 | 64.2 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-lidar-loss/result.json) |
| `fastlivo-obstacle` | 失败 | 144.7 | 是 | F-023：alignment reference_position_source_gap；没有执行绕障，随后正常落地上锁。；[证据](artifacts/test-full-v1-20261023/sitl/fastlivo-obstacle/result.json) |
| `fastlivo-long-run` | 通过 | 310.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fastlivo-long-run/result.json) |
| `orbslam3-mission-astar` | 通过 | 78.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-mission-astar/result.json) |
| `orbslam3-mission-ego` | 通过 | 99.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-mission-ego/result.json) |
| `orbslam3-mission-combined` | 通过 | 98.4 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-mission-combined/result.json) |
| `orbslam3-cancel` | 通过 | 74.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-cancel/result.json) |
| `orbslam3-backend-exit` | 通过 | 66.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-backend-exit/result.json) |
| `orbslam3-visual-loss` | 通过 | 68.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-visual-loss/result.json) |
| `orbslam3-clock-reset` | 通过 | 78.8 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-clock-reset/result.json) |
| `orbslam3-map-loss` | 通过 | 74.6 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-map-loss/result.json) |
| `orbslam3-camera-loss` | 通过 | 65.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-camera-loss/result.json) |
| `orbslam3-depth-loss` | 通过 | 66.3 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-depth-loss/result.json) |
| `orbslam3-long-run` | 通过 | 318.8 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/orbslam3-long-run/result.json) |
| `navigation-happy-astar-registered` | 通过（测试进程表适配） | 70.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-happy-astar-registered/result.json) |
| `navigation-happy-ego-registered` | 失败 | 70.7 | — | F-015：点云失效后导航保护停止；未完成任务。; sitl/navigation-happy-ego-registered/registered-evaluation.json: original_flight_assertions; sitl/navigation-happy-ego-re…；[证据](artifacts/test-full-v1-20261023/sitl/navigation-happy-ego-registered/result.json) |
| `navigation-happy-combined-registered` | 通过（测试进程表适配） | 83.7 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/navigation-happy-combined-registered/result.json) |
| `navigation-cancel-registered` | 测试配置/脚本错误 | 65.6 | — | 测试目标超出登记推荐区域，动作在取消触发前结束；不证明取消逻辑失败。; sitl/navigation-cancel-registered/registered-evaluation.json: original_flight_assertions; sitl/navigation-cance…；[证据](artifacts/test-full-v1-20261023/sitl/navigation-cancel-registered/result.json) |
| `navigation-cancel-registered-lane` | 失败 | 76.8 | — | F-027：登记三维通道启动后地图未就绪；未进入取消阶段，无飞行动作。; sitl/navigation-cancel-registered-lane/registered-evaluation.json: original_flight_assertions; sitl/navigation…；[证据](artifacts/test-full-v1-20261023/sitl/navigation-cancel-registered-lane/result.json) |
| `navigation-map-loss-registered` | 未运行（夹具阻塞） | — | — | 登记入口夹具未建立可用地图；原项目入口的相应执行结果另列，不计本变体通过。；[矩阵](artifacts/test-full-v1-20261023/matrix-navigation-registered.json) |
| `navigation-mission-registered` | 未运行（夹具阻塞） | — | — | 登记入口夹具未建立可用地图；原项目入口的相应执行结果另列，不计本变体通过。；[矩阵](artifacts/test-full-v1-20261023/matrix-navigation-registered.json) |
| `navigation-search-registered` | 未运行（夹具阻塞） | — | — | 登记入口夹具未建立可用地图；原项目入口的相应执行结果另列，不计本变体通过。；[矩阵](artifacts/test-full-v1-20261023/matrix-navigation-registered.json) |
| `navigation-mission-nav-cancel-registered` | 未运行（夹具阻塞） | — | — | 登记入口夹具未建立可用地图；原项目入口的相应执行结果另列，不计本变体通过。；[矩阵](artifacts/test-full-v1-20261023/matrix-navigation-registered.json) |
| `navigation-mission-nav-timeout-registered` | 未运行（夹具阻塞） | — | — | 登记入口夹具未建立可用地图；原项目入口的相应执行结果另列，不计本变体通过。；[矩阵](artifacts/test-full-v1-20261023/matrix-navigation-registered.json) |
| `navigation-mission-nav-land-registered` | 未运行（夹具阻塞） | — | — | 登记入口夹具未建立可用地图；原项目入口的相应执行结果另列，不计本变体通过。；[矩阵](artifacts/test-full-v1-20261023/matrix-navigation-registered.json) |
| `navigation-mission-search-cancel-registered` | 未运行（夹具阻塞） | — | — | 登记入口夹具未建立可用地图；原项目入口的相应执行结果另列，不计本变体通过。；[矩阵](artifacts/test-full-v1-20261023/matrix-navigation-registered.json) |
| `navigation-mission-search-timeout-registered` | 未运行（夹具阻塞） | — | — | 登记入口夹具未建立可用地图；原项目入口的相应执行结果另列，不计本变体通过。；[矩阵](artifacts/test-full-v1-20261023/matrix-navigation-registered.json) |
| `navigation-mission-search-land-registered` | 未运行（夹具阻塞） | — | — | 登记入口夹具未建立可用地图；原项目入口的相应执行结果另列，不计本变体通过。；[矩阵](artifacts/test-full-v1-20261023/matrix-navigation-registered.json) |
| `orbslam3-map-relocalization-cold-repeat` | 通过 | 79.7 | 是 | 冷启动通过：同一张 19 关键帧旧地图遮挡后重定位；公共位姿计数冻结于 557，新导航持续拒绝，最终上锁。；[证据](artifacts/test-full-v1-20261023/sitl/orbslam3-map-relocalization-cold-repeat/result.json) |
| `orbslam3-natural-map-seed` | 通过 | 99.3 | 是 | 自然照片原生夹具建图及往返飞行通过；本轮保存 19 关键帧、2035 地图点的实际 Atlas，闭环计数为 0。；[证据](artifacts/test-full-v1-20261023/sitl/orbslam3-natural-map-seed/result.json) |
| `orbslam3-map-relocalization` | 前置条件未满足 | 36.2 | — | F-029：原生 gz model --pose 在输出正确模型坐标后因 Boost thread self-join 非零退出；场景夹具验证失败，尚未允许 RGB-D 输入或发送飞行任务。；[证据](artifacts/test-full-v1-20261023/sitl/orbslam3-map-relocalization/result.json) |
| `orbslam3-live-inertial-loop` | 审计失败 | 329.3 | 是 | F-030：实际完成 1 次惯性回环、输入与锁存断言通过，但位置 RMSE 16.496 m、最大 18.564 m，整体审计失败；FAST-LIO 控制下正常落地。; sitl/orbslam3-live-inertial-loop/evaluation.json: position_rms…；[证据](artifacts/test-full-v1-20261023/sitl/orbslam3-live-inertial-loop/result.json) |
| `gcopter-smoke` | 通过 | 104.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-smoke/result.json) |
| `gcopter-over` | 通过 | 107.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-over/result.json) |
| `gcopter-under` | 通过 | 117.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-under/result.json) |
| `gcopter-channel` | 通过 | 123.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-channel/result.json) |
| `gcopter-narrow` | 通过 | 106.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-narrow/result.json) |
| `gcopter-no-solution` | 通过 | 64.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-no-solution/result.json) |
| `gcopter-cancel` | 通过 | 93.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-cancel/result.json) |
| `gcopter-map-loss` | 通过 | 94.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-map-loss/result.json) |
| `gcopter-backend-exit` | 通过 | 74.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-backend-exit/result.json) |
| `gcopter-map-change` | 通过 | 117.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/gcopter-map-change/result.json) |
| `super-smoke` | 通过 | 129.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/super-smoke/result.json) |
| `super-over` | 通过 | 125.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/super-over/result.json) |
| `super-under` | 通过 | 143.2 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/super-under/result.json) |
| `super-channel` | 通过 | 142.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/super-channel/result.json) |
| `super-narrow` | 通过 | 131.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/super-narrow/result.json) |
| `super-no-solution` | 通过 | 64.7 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/super-no-solution/result.json) |
| `super-cancel` | 通过 | 102.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/super-cancel/result.json) |
| `super-map-loss` | 通过 | 103.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/super-map-loss/result.json) |
| `super-backend-exit` | 通过 | 80.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/super-backend-exit/result.json) |
| `super-map-change` | 失败 | 75.8 | 是 | F-012：更新地图后的备用轨迹不可行；原生请求独立重放重现；正常落地。；[证据](artifacts/test-full-v1-20261023/sitl/super-map-change/result.json) |
| `fuel-bootstrap` | 通过 | 38.9 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-bootstrap/result.json) |
| `fuel-complete` | 通过 | 128.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-complete/result.json) |
| `fuel-cancel` | 失败 | 70.7 | 是 | F-014：取消成功，后续新探索任务 SOURCE_MAP_INVALID；冷启动补测另列。；[证据](artifacts/test-full-v1-20261023/sitl/fuel-cancel/result.json) |
| `fuel-map-loss` | 通过 | 120.3 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-map-loss/result.json) |
| `fuel-worker-exit` | 通过 | 68.0 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-worker-exit/result.json) |
| `fuel-worker-timeout` | 通过 | 84.1 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-worker-timeout/result.json) |
| `fuel-timeout` | 通过 | 67.7 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-timeout/result.json) |
| `fuel-unreachable` | 测试配置/脚本错误 | 129.9 | 是 | 首次测试配置错误：普通房间代替封闭不可达夹具；正确场景补测另列。；[证据](artifacts/test-full-v1-20261023/sitl/fuel-unreachable/result.json) |
| `fuel-no-fly` | 通过 | 132.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-no-fly/result.json) |
| `fuel-land-takeover` | 通过 | 67.7 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-land-takeover/result.json) |
| `fuel-warehouse-complete` | 通过 | 143.5 | 是 | [证据](artifacts/test-full-v1-20261023/sitl/fuel-warehouse-complete/result.json) |
| `precision-external-takeover` | 通过 | 34.4 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-external-takeover/result.json) |
| `precision-external-posctl-handoff` | 前置条件未满足 | 43.9 | — | 原夹具缺少有效手动输入，PX4 拒绝 POSCTL；提供居中输入的独立补测另列。；[证据](artifacts/test-full-v1-20261023/sitl/precision-external-posctl-handoff/result.json) |
| `precision-external-rtl-descent` | 通过 | 35.5 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-external-rtl-descent/result.json) |
| `precision-unknown-mode-late` | 通过 | 47.7 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-unknown-mode-late/result.json) |
| `precision-unknown-mode-withheld` | 通过 | 47.5 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-unknown-mode-withheld/result.json) |
| `precision-upstream-arm-late` | 通过 | 30.4 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-upstream-arm-late/result.json) |
| `precision-upstream-arm-withheld` | 通过 | 30.4 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-upstream-arm-withheld/result.json) |
| `precision-cancel-precision-mode-late` | 通过 | 48.7 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-precision-mode-late/result.json) |
| `precision-cancel-precision-mode-withheld` | 通过 | 49.6 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-precision-mode-withheld/result.json) |
| `precision-cancel-land-mode-late` | 通过 | 40.0 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-land-mode-late/result.json) |
| `precision-cancel-land-mode-withheld` | 通过 | 39.9 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-land-mode-withheld/result.json) |
| `precision-bridge-mode-late` | 通过 | 47.7 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-bridge-mode-late/result.json) |
| `precision-bridge-mode-withheld` | 通过 | 48.9 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-bridge-mode-withheld/result.json) |
| `precision-bridge-arm-late` | 通过 | 32.6 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-bridge-arm-late/result.json) |
| `precision-bridge-arm-withheld` | 通过 | 30.4 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-bridge-arm-withheld/result.json) |
| `precision-cancel-bridge-arm-late` | 通过 | 27.3 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-bridge-arm-late/result.json) |
| `precision-cancel-bridge-arm-withheld` | 通过 | 32.7 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-bridge-arm-withheld/result.json) |
| `precision-cancel-bridge-mode-late` | 通过 | 26.5 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-bridge-mode-late/result.json) |
| `precision-cancel-bridge-mode-withheld` | 通过 | 26.7 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-cancel-bridge-mode-withheld/result.json) |
| `precision-external-posctl-descent-manual-input` | 测试配置/脚本错误 | 0.4 | — | 首次补测脚本未创建输出目录，尚未启动仿真；v2 已纠正并另列。；[证据](artifacts/test-full-v1-20261023/precision-external-posctl-descent-manual-input.execution.json) |
| `precision-external-posctl-descent-manual-input-v2` | 通过 | 34.2 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-external-posctl-descent-manual-input-v2/result.json) |
| `precision-external-posctl-handoff-manual-input` | 通过 | 41.9 | — | [证据](artifacts/test-full-v1-20261023/sitl/precision-external-posctl-handoff-manual-input/result.json) |
| `scene-bookstore_aws_v1` | 失败 | 96.0 | — | F-026：地面检查连续新鲜状态门限未满足；没有发送飞行任务，24/24 子进程正常退出。；[证据](artifacts/test-full-v1-20261023/sitl/scene-bookstore_aws_v1/result.json) |
| `scene-forest_pine_v1` | 失败 | 80.7 | — | F-027：75 秒内仅 1 条栅格、没有 nonground 点云，反复源时间 TF 等待超时；未发出飞行动作，24/24 正常退出。；[证据](artifacts/test-full-v1-20261023/sitl/scene-forest_pine_v1/result.json) |
| `scene-hospital_aws_v2` | 失败 | 98.4 | — | F-028：参考任务起飞后导航输入失效，动作 aborted；最后状态仍解锁，进程正常清理不计为落地成功。；[证据](artifacts/test-full-v1-20261023/sitl/scene-hospital_aws_v2/result.json) |
| `scene-house_aws_v1` | 失败 | 97.5 | — | F-026：60 秒连续就绪检查超时，未发送飞行任务；24/24 子进程正常退出。；[证据](artifacts/test-full-v1-20261023/sitl/scene-house_aws_v1/result.json) |
| `scene-office_willowgarage_v1` | 通过 | 117.3 | 是 | 登记往返任务及落地后恢复全部通过，两个真实导航子 UUID 完成；最终上锁，24/24 正常退出。；[证据](artifacts/test-full-v1-20261023/sitl/scene-office_willowgarage_v1/result.json) |
| `scene-primitives_corridor_seed1042_v2` | 通过 | 120.7 | 是 | 登记参考往返飞行、正常降落及落地后恢复通过；最终上锁，24/24 子进程正常退出。；[证据](artifacts/test-full-v1-20261023/sitl/scene-primitives_corridor_seed1042_v2/result.json) |
| `scene-primitives_sparse_seed42_v1` | 失败 | 80.6 | — | F-027：75 秒地面启动超时，地图未就绪，未发送飞行任务；24/24 正常退出。；[证据](artifacts/test-full-v1-20261023/sitl/scene-primitives_sparse_seed42_v1/result.json) |
| `scene-racetrack_aws_day_v2` | 失败 | 40.4 | — | F-026：地面原生心跳节奏/时间戳检查失败；未发送飞行任务，24/24 正常退出。；[证据](artifacts/test-full-v1-20261023/sitl/scene-racetrack_aws_day_v2/result.json) |
| `scene-urban_block_osrf_v1` | 通过 | 120.0 | 是 | 登记往返任务、落地后恢复均通过，导航误差约 8.7 cm；最终上锁，24/24 正常退出。；[证据](artifacts/test-full-v1-20261023/sitl/scene-urban_block_osrf_v1/result.json) |
| `scene-warehouse_aws_no_roof_v1` | 失败 | 90.6 | — | F-028：飞行中收到零时间戳地图和空点云，参考任务中止；最后仍解锁，24/24 进程正常清理。；[证据](artifacts/test-full-v1-20261023/sitl/scene-warehouse_aws_no_roof_v1/result.json) |
| `scene-warehouse_aws_v1` | 失败 | 80.9 | — | F-027：75 秒内地图未就绪，仅 1 条栅格、无 nonground 点云；未发送任务，24/24 正常退出。；[证据](artifacts/test-full-v1-20261023/sitl/scene-warehouse_aws_v1/result.json) |

<!-- SIMULATION_RESULTS_END -->

## 问题记录

### 已确认问题 F-001：GPS 转局部 ENU 的高度符号反向

- 位置：`src/easyfly_common/easyfly_utils/include/easyfly_utils/geometry.hpp`，`gps_to_local_enu()`。
- 复现：参考高度 100 m、输入高度 110 m、经纬度保持相同；ENU 的 Up 应为 +10 m，实际返回 -10 m。
- 原因：第三分量使用 `ref_alt - alt`。
- 影响范围：公共工具函数；当前自有源码中未找到运行时调用方，不能据此声称现有 SITL 高度控制必然受影响。
- 修复状态：本轮仅测试与记录，未修改产品代码。
- 复现源码：`artifacts/test-full-v1-20261023/header_contracts.cpp`。

### 已确认问题 F-002：Sophus 默认构建引用未交付目录

- 位置：`src/third_party/algorithms/localization/fast_livo2_dependencies/sophus/CMakeLists.txt:147`、`:153`。
- 本轮独立全量构建首次失败：`add_subdirectory(test)` 和 `add_subdirectory(examples)` 找不到对应目录。
- 原因：裁剪交付未包含这两个目录，但 `BUILD_SOPHUS_TESTS` 与 `BUILD_SOPHUS_EXAMPLES` 默认都是 ON。现有根构建包装和内测手册的全量构建参数未覆盖这两个选项。
- 后续处理：仅在本轮构建配置中显式关闭未交付的上游测试/示例，继续验证平台；不修改产品源码，不将首次默认构建改记为成功。
- 证据：`artifacts/test-full-v1-20261023/build-log/` 内首次 `sophus/stderr.log` 与 `build-console-resume.log`。

### 已确认问题 F-003：里程计速度转换误用了半角

- 严重程度：P1，运行时坐标契约错误。
- 位置：`src/perception/p1_preprocess/easyfly_coord_transform/src/odom_tf_publisher.cpp:64`、`:92`。
- 使用本轮编译的真实节点，向独立 DDS domain 224 发布模拟 `UAVState`，读取实际 `/odom`。
- 输入 yaw=π/2、ENU 速度 (1,0,0)；`child_frame_id=base_link` 下预期速度 (0,-1,0)，实际 (0.70710678,-0.70710678,0)。roll/pitch 90° 的独立用例也失败，零姿态对照通过。
- 原因：四元数计算所用的 `cos(angle/2)`、`sin(angle/2)` 被直接复用于完整旋转矩阵。公共头文件 `enu_to_flu()` 的同一测试正确，节点未复用该实现。
- 影响：发布给地图/规划及观察者的 `/odom` 速度方向错误；本轮尚未将某个 SITL 失败唯一归因于它。
- 原始证据：`artifacts/test-full-v1-20261023/uncovered-interfaces/result.json`。产品代码未修改。

### 已确认问题 F-004：无效状态仍发布为有效外观的里程计与 TF

- 严重程度：P1，跨模块失效语义丢失。
- 位置：`src/perception/p1_preprocess/easyfly_coord_transform/src/odom_tf_publisher.cpp:44`、`:60`。
- 实际节点分别收到 `pose_valid=false`、`attitude_valid=false`、NaN 位置、错误输入 frame 后，仍持续发布对应源时间的 `/odom` 与 `odom → base_link`。
- 各用例采样窗口收到 19～21 条里程计及 19～20 条 TF；NaN 进入实际里程计，`unexpected_map` 输入被直接改标为 `odom`。
- 原因：接收回调只缓存状态，发布只检查是否曾收到消息，不校验有效性、数值与输入坐标系。
- 原始证据同 F-003；RC 通道转发（正常、空数组、18 通道）及两个节点正常退出均通过。

### 已确认问题 F-005：仓库保留的旧轨迹头文件不能独立编译

- 严重程度：P2，遗留源码维护问题，当前活动导航链未证实受影响。
- 位置：`src/third_party/algorithms/planning/ego_planner/traj_utils/include/traj_utils/polynomial_traj.h:195`、`:237`。
- `getMeanVel()` 缺少返回语句；`getJerk()` 用 double 下标调用 Eigen 矩阵 `operator()`，在当前 Eigen 3.4 下产生编译错误。
- 本轮最小独立 include 编译失败；同一探针对实际使用的 `traj_utils_so` 版本编译成功，后者已包含返回值和 `coeffRef` 修正。
- `traj_utils` 当前 CMake 只构建消息，旧库构建被注释，因此全工作区构建未覆盖这些旧函数。没有发现 `getMeanVel()` 的实际调用方。
- 证据：`artifacts/test-full-v1-20261023/polynomial-header-compile.console.log` 与 `polynomial-active-header-compile.execution.json`。
- 因编译失败，计划中的 UBSan 运行未发生；不把测试程序未启动算作额外运行时缺陷。

### 已确认问题 F-006：EGO 退出测试与新 launch 参数不兼容

- 严重程度：P2，测试集成缺陷，导致该退出回归未执行。
- 原始 `test_ego_signal_shutdown_ros` 在 0.07 秒内失败：`NameError: name 'LaunchConfiguration' is not defined`。
- 位置：`src/third_party/algorithms/planning/ego_planner/plan_manage/test/test_ego_signal_shutdown.py:27`。测试从 launch AST 提取参数字典后，在不完整的 eval 上下文中求值；新增 `max_acceleration` 的 `LaunchConfiguration` 和 `float` 不在该上下文内。
- 本轮使用独立副本准备补测：仅通过实际 launch 默认值和 LaunchContext 补全测试侧解析，原始 12 种阶段/时钟/信号组合和退出断言保持；原始失败不覆盖，补测结果单列。
- 原始证据：`artifacts/test-full-v1-20261023/build/ego_planner/Testing/`；补测源码及差异说明：`test_ego_signal_supplement.py`、`ego-signal-supplement-source.json`（位于本轮证据目录）。
- 补测实际通过：12 个原始阶段/时钟/信号组合，8.23 秒；见 `ego-signal-supplement.execution.json`。这不改变原测试入口失败的结论。

### 运行问题 F-007：定位准入回归进程发生一次原生段错误

- 严重程度：P1 待定位；已观察到崩溃，根因未确认。
- 原始 `test_localization_selection` 收集 11 个用例，在前 4 个完成后，Python 在 `rclpy/action/client.py:603 wait_for_server` 收到 SIGSEGV（退出码 -11），未生成正常 JUnit。
- 留下该用例创建的 ROS launch 与协调器两个进程，已按域 213、PID、进程组及启动时刻核实归属，并发送 SIGINT 正常清理，无强杀、无遗留。
- GDB 完整复测 11/11 通过，逐用例 fork 隔离复测 11/11 通过，均无遗留；未再次捕获崩溃，不能宣称已定位或修复，也不能据此指认 FAST-LIVO2 算法崩溃。
- 证据：`build/easyfly_global_planner/ament_cmake_pytest/test_localization_selection.txt`、`localization-selection-diagnostic/`、`localization-selection-orphan-cleanup.json`（均在本轮证据目录）。

### 已确认问题 F-008：场景清单测试仍假设所有登记场景支持 base_sim

- 严重程度：P2，功能清单与回归测试脱节。
- `test_scene_catalog.py:24` 断言失败于 `fuel_room_v1`。当前 `fuel_room_v1`、`fuel_warehouse_v1`、`gcopter_3d_lane_v1` 明确只登记 `navigation_dev`，共 11 个场景登记 `base_sim`。
- 14 个登记场景的资源/许可/登记检查均通过；本失败不能等同于 FUEL 场景资源缺失，也不能把 3 个导航专用场景擅自改成基础场景。
- 原始证据：`build/easyfly_sim_launcher/ament_cmake_pytest/test_scene_catalog.txt` 与 `scene-catalog.json`。

### 集成问题 F-009：部分测试入口在 DDS 兼容库启用前加载了系统库

- 严重程度：P2，环境/启动顺序敏感。
- `test_scene_launch` 的两个 managed 参数用例失败于 `Unpatched Fast DDS is already loaded`；独立 `check_orbslam3_imu_guards.py` 也在任何实际用例执行前触发同一错误。
- 测试进程先导入 rclpy/相关库，再调用 `activate()`；原普通构建环境没有提前将项目私有 Fast DDS 路径置于动态库搜索路径前部。
- 本轮另建 `simulation-environment.bash`，只为显式仿真进程从启动时加载本轮构建的私有兼容库；不替换系统库、不降低守卫检查。原失败保留，新环境补测单列。
- 原始证据：`build/easyfly_sim_launcher/ament_cmake_pytest/test_scene_launch.txt`、`orbslam3-imu-guards.console.log`。
- 通过显式私有 DDS 环境补测：场景启动 62/62 通过，ORB IMU 守卫 6/6 通过；原入口的环境敏感性仍需处理。

### 已确认问题 F-010：仓库的 MID360 等价性测试未随原生 IMU 接入更新

- 严重程度：P2，回归契约过期。
- `test_warehouse_scene.py:40` 仍要求裁剪模型与旧模型除外观外完全相同；当前模型已更换项目 IMU 插件、使用 `livox_imu_frame`、采用逐轴噪声结构，并移除无效旧参数，因此断言失败。
- 这些变化及先后哈希已记录于 `src/simulation/easyfly_gazebo_models/licenses/runtime/ORIGINS.json` 的 `mid360_updates`，并非本轮测试改写模型。
- 需要将测试更新为核验已声明的传感器语义；本轮不通过回退 IMU 模型或删除断言来消除失败。
- 证据：`build/easyfly_sim_launcher/ament_cmake_pytest/test_warehouse_scene.txt`。

### 运行问题 F-011：旧 perception_demo 入口间歇性未达到起飞后悬停门限

- 严重程度：P1 待定位；实际仿真失败，不能归因于某一个导航策略。
- `navigation-happy-ego`、EGO 独立冷启动复现、combined 的 `navigation-cancel`、带只读观察者的 EGO 均在 `/takeoff` 成功后、发起导航前失败。原门限是目标高度 1.5 m±0.1 m、速度≤0.3 m/s、连续 1 秒稳定、15 秒墙钟等待，未放宽。
- 失败轮在解锁前就报告约 1.28～1.33 m 的本地高度与 0.61～0.72 m/s 的向上速度；成功 A*、combined 对照轮接近零高度和零速度。只读原生 Gazebo 真值与飞控高度观测出现明显分歧；MAVROS 悬停目标仍为 1.5 m。精确根因尚未确定。
- 原生 ULog 对照进一步确认：失败轮解锁前 PX4 `vehicle_local_position` 的 NED z≈-1.247 m、vz≈-0.585 m/s，而 A* 成功轮 z≈0.055 m、vz≈-0.0012 m/s。异常在 PX4 原生估计中已存在，不能仅归因于 ROS 转换；具体传感器/估计器根因仍未确定。见 `hover-px4-prearm-comparison.json`。
- [原生高度/速度对照图](artifacts/test-full-v1-20261023/hover-px4-prearm-comparison.png) 仅按首次解锁时刻对齐时间，不修改定位数据；图源为本轮 ULog。
- `navigation-search` 也在发起搜索前复现同样问题（解锁前高度 1.322 m），仅 `/takeoff` 完成，搜索功能在该轮受阻。
- `navigation-mission-nav-land`、搜索取消/超时的准备任务完成后，也未在 15 秒内满足 1.5 m±0.1 m 的稳定门限（截止高度约 1.37～1.39 m）；尚未进入各自故障触发阶段。导航任务取消和超时用例实际通过。
- 这些失败轮未完成 `/navigate`/`search_area` 与正常 `/land`，最后状态仍 armed=true；测试拥有的仿真栈随后正常退出，24 进程结束、无强杀、无遗留。进程退出不等于安全降落通过。
- 使用正式登记 `navigation_dev / primitives_sparse_seed42_v1` 的 EGO 对照轮完成起飞、导航、降落并最终 disarmed，终点误差约 0.0635 m。
- 登记入口额外启动 parameter_manager、perception_services、mission_services，共 27 进程；旧导航探针按其原入口硬编码 24 进程，因此原 verdict=false 被原样保存。补测使用明确的 27 角色多重集与全部正常退出检查，所有飞行门限不变，独立评价通过。这是本轮测试入口适配，不将它误报为产品缺陷或旧入口已修复。
- 证据：`sitl/navigation-happy-ego/`、`sitl/navigation-happy-ego-cold-repeat/`、`sitl/navigation-cancel/`、`sitl/navigation-ego-legacy-observed/`、`hover-native-witness.json`、`sitl/navigation-ego-registered-observed/registered-evaluation.json`。

### 运行问题 F-012：SUPER 地图变化后的备用轨迹生成失败

- 严重程度：P2，动态重规划可用性失败；拒绝输出不满足约束的轨迹本身符合保护策略。
- 实际 `super-map-change` 在飞行至 x≈0.253 m 时插入原生障碍；先检测到 `current_map_collision_or_unknown` 并触发重规划，随后 `worker_rejected_or_exited`，导航 status=6、`planner_emergency_stop`，没有完成预期往返。
- 原生 job-2 日志显示 CIRI 多面体不可行，报告最近距离约 0.1611 m；随后 `generateBackupTrajectory` 和 `PlanFromRest` 返回 FAILED。用完全相同请求、本轮安装 worker、新目录独立重放，0.615 秒内再次返回 1，同样原因；不是本轮外层超时。
- 本轮场景失败，但正常降落、最终 armed=false、26/26 进程正常退出、worker 已回收、无强杀与遗留。未发现该轮继续输出撞障轨迹的证据；尚未确定改善地图采样、原生种子/走廊生成或其他策略的具体修复。
- 同算法另外 9 项通过，GCOPTER 同类地图变化场景通过；不能用这些通过覆盖此项失败。
- 证据：`sitl/super-map-change/result.json`、`sitl/super-map-change/super-UmMLCU/job-2.log`、`super-map-change-replay/result.json`。

### 已确认问题 F-013：Livox 仿真插件安装后仍依赖构建时源码路径

- 严重程度：P1，独立部署缺少点云；当前源码仍在的工作区运行不受这个条件触发。
- 位置：`src/simulation/easyfly_gazebo_plugins/livox_laser_plugin/CMakeLists.txt:22` 将 `SCAN_MODE_DIR` 编译为源码绝对路径；`src/livox_points_plugin.cpp:60` 用该路径解析相对 CSV 文件名。CSV 虽安装到包的 share 目录，运行时没有从该位置查找。
- 对同一安装模型和同一 18 秒纯 Gazebo 传感器用例，在私有用户/挂载命名空间进行对照：源码可见时收到 148 条点云（每条 20000 点）；仅遮蔽源码 scan_mode 目录后收到 0 条，并出现 `cannot get csv file!`。两轮安装 CSV 均存在、哈希相同，世界 SDF 哈希相同，服务正常退出。
- 此补测未启动 PX4、不发飞行指令；源码遮蔽仅对子进程可见，宿主源码仍存在且与安装 CSV 哈希一致。测试工具退出 0 表示完成了观察，隐藏源码轮的功能判定明确为 `passed=false`，不混算为通过。
- 修复方向：运行时按已安装包资源位置解析扫描文件，并保留显式绝对路径支持；本轮未修改产品代码。
- 证据：`livox-install/summary.json`、`livox-install/visible/server.log`、`livox-install/hidden/server.log`、`livox_install_probe.py`。

### 运行问题 F-014：FUEL 取消后的新探索任务因源地图失效中止

- 严重程度：P2 待定位，任务恢复可用性失败；保护性停止不等于恢复通过。
- `fuel-cancel` 的首个探索任务正常响应取消，返回 status=5 / `CANCELED`，原生 worker 正常回收。用例随后创建的新任务被接受、产生真实原生规划并开始移动，但在首个视点完成前返回 status=6 / `SOURCE_MAP_INVALID`。
- 终止时覆盖率约 55.13%、完成视点 0；`node.py:92` 在活动任务收到 valid=false 的源地图时锁存该原因。现有日志尚不足以唯一解释源地图失效原因，不能把它归因为取消请求未响应。
- 最终正常降落、armed=false、全部启动进程正常退出、原生 worker 回收、无强杀或遗留。正常房间完整探索已通过；后续异常/恢复用例另列，不覆盖本次失败。
- 证据：`sitl/fuel-cancel/result.json`、`fuel-cancel.console.log`、`sitl/fuel-cancel/fuel/*/events.jsonl` 和 `source-maps.jsonl.gz`。
- 独立冷启动复测 `fuel-cancel-cold-repeat` 完整通过（含取消后的新任务恢复及独立审计），117.44 秒；首次故障仍保留，当前为间歇性问题、根因未确认。

### 运行问题 F-015：登记场景导航中非地面点云失效，EGO 保护停止

- 严重程度：P1 待定位，正常导航链间歇性中断；保护停止本身按失效契约执行。
- `navigation-happy-ego-registered` 已正常通过起飞及稳定悬停，但在 x≈1.770 m（目标约 2 m）时 `/navigate` 返回 status=6 / `planner_emergency_stop`。
- 同时观测到 semantic_map 的“点云对应源时间的 TF 等待超时”，随后 EGO 报告“可信非地面点云已失效”。只读输入记录显示末段 `/pointcloud_nonground` 的接收间隔达到 1.009 秒，下一条为 width=0 的失效标记；该窗口 `/odom` 最大接收间隔约 0.186 秒。
- 此前 `navigation-ego-registered-observed` 同场景完整通过；本轮未唯一确定 TF/点云停顿原因，也不能把它等同于旧入口 F-011 的起飞前高度异常。
- 失败轮没有执行正常 Land，最后仍 armed=true；随后 27/27 仿真进程正常退出、无强杀和遗留。不能将“进程正常退出”记录为“安全降落通过”。
- 证据：`sitl/navigation-happy-ego-registered/result.json`、`console.log`、`input-gap-summary.json`、`registered-evaluation.json`。

### 运行问题 F-016：精准降落结束后的 MAVROS 退出段错误

- 严重程度：P2 待定位，退出可靠性问题；本轮未观察到飞行过程中崩溃。
- `precision-nominal` 实际完成起飞、视觉对准、下降、模式交接与降落，`final_disarmed=true`，水平落点误差 0.02194 m。
- 正常退出阶段收到 SIGINT 后，系统 MAVROS 节点 `/opt/ros/humble/lib/mavros/mavros_node` 返回 -11；launch 包装进程仍返回 0，但探针检查到子进程异常并正确将整项判为失败。不能只凭动作成功或父进程退出码记为通过。
- 观察者正常退出、无遗留。已从本次 PID 404432 的 Apport core 取得原生栈：崩溃线程位于 `libmavconn.so` 的 `mavlink::mavlink_get_msg_entry`，调用链包含 `Router::route_message`、`Endpoint::recv_message`、`parse_buffer`；另一线程已进入 `_dl_fini` / `__cxa_finalize`。这支持退出清理与消息处理发生重叠的判断，但尚未确定具体对象寿命错误或修复，不据此指认精准降落算法崩溃。
- 证据：`sitl/precision-nominal/result.json`、`sitl/precision-nominal/bridge.log:245`、`mavros-shutdown-core-gdb.console.log`、`mavros-shutdown-crash/provenance.json`。

### 已确认问题 F-017：一键清理脚本遗漏新算法节点并误报零遗留

- 严重程度：P2，仿真清理工具与新增模块未同步。
- `easyfly_clean.sh` 的进程匹配表未包含本轮已安装的 `fastlivo_mapping`、`run_subscribe_msckf`、`openvins_guard`、`openvins_alignment`、`orbslam3_rgbd`、`super_worker`、`super_navigation`。
- 在私有 user/PID/mount 命名空间及私有 `/proc` 中，仅创建相同命令行的无业务行为进程，执行原脚本：18 个旧名称/ROS launch 匹配对象均终止，2 个无关对照保留，上述 7 个新名称全部存活。脚本却输出“残留进程: 0”并另起一行“0”；计数查询也遗漏这些名称，`pgrep -c` 零匹配时又触发 `echo 0`。
- 本轮没有在宿主执行全局 `pkill`；全部模拟进程随后回收，运行中的真实测试仿真未受影响。该证据验证脚本匹配行为，不声称启动并杀过这 7 个真实算法进程。
- 第一版模拟进程继承了忽略 SIGTERM 的状态，导致测试侧收尾等待超时；命名空间结束后无遗留，纠正模拟进程信号默认行为后重测。原产品脚本没有修改。
- 证据：`cleanup-helper-v2/result.json`、`stdout.log`、`cleanup-helper-v1-harness-error.json`、`cleanup_helper_probe.py`。

### 已确认问题 F-018：旧 ego_to_mavros 默认 launch 参数类型错误

- 优先级：P2；范围是旧独立适配节点，非当前主执行链。
- 已安装 `bridge.launch.py` 默认 `drone_id=0` 被 launch 推断成 integer，节点声明 string，抛出 `InvalidParameterTypeException` 并以 -6 退出；父 launch 返回 0，不能据此判断节点成功。
- 建议显式使用字符串类型的 ParameterValue。证据：`legacy-bridge/default-launch.log`。

### 已确认问题 F-019：旧适配节点在首条轨迹到达前进入悬停会崩溃

- 优先级：P1（启用该旧节点时）；当前主执行链未使用它。
- 模拟有效里程计及 connected/armed/OFFBOARD，尚未发送 PositionCommand 时，定时器抛出 `can't subtract times with different time sources [1 != 2]`，进程 -6、零 setpoint。
- `bridge.hpp` 中默认构造的 `last_cmd_time_` 与 `this->now()` 时钟类型不同；应统一时钟，并在首次有效指令前避免计算指令超时。
- 证据：`legacy-bridge/first_hold.log`、`legacy-bridge/result.json`。提前发送有效轨迹的独立对照流程能运行，未修改产品代码。

### 已确认问题 F-020：旧适配节点断流回退使用过期的初始位置

- 优先级：P1（启用该旧节点时）。
- OFFBOARD 初始位置为 `(1,2,3)`，后续模拟里程计移动至 `(7,8,3)`；超过 0.5 秒指令超时后发布的悬停位置仍为 `(1,2,3)`，可能产生非预期返航式位移。
- `last_hold_pose_` 仅在首次进入 HOLD 时记录，断流切换 FALLBACK_HOLD 时未更新。速度/加速度屏蔽位正常。
- 证据：`legacy-bridge/result.json`、`legacy-bridge/seeded_yaw_false.log`。此项为模拟接口行为，不是实际飞行轨迹。

### 已确认问题 F-021：旧适配节点直接转发非有限位置指令

- 优先级：P1（启用该旧节点时）。
- READY PositionCommand 的 x=NaN 被转发为 11 条非有限 setpoint，节点缺少有限数值校验。
- 同轮正常 ENU 位置/速度/加速度转发、yaw 配置、非 READY 忽略、外部模式接管停止输出均通过；13 项接口检查共 9 通过、4 失败。
- 证据：`legacy-bridge/result.json`；仅启动旧适配节点及本机模拟 ROS 发布/订阅端，无 MAVROS、PX4 或硬件。

### 运行问题 F-022：FAST-LIVO2 无故障注入时出现 IMU 源时间间隔失效

- 优先级：P1，根因待定；首轮 `fastlivo-cancel` 未实际进入取消流程。
- 已完成初始化起飞前后，guard 报 `imu_source_gap`，alignment/navigation 随后锁定失效。源码阈值为连续 IMU 源时间间隔大于 20 ms；本轮没有修改该阈值或伪造补齐样本。
- 探针执行 Land，最终上锁，24 个子进程正常退出、无强杀或遗留。保护动作生效；自然数据流不连续的生产端/传输端原因尚未证明。
- 证据：`sitl/fastlivo-cancel/result.json`、`sitl/fastlivo-cancel/console.log`（`FAST-LIVO2 state=4: imu_source_gap`）。三种规划策略的此前独立任务均通过；后续冷启动补测另列，不覆盖本次失败。

FAST-LIVO2 取消补测 `fastlivo-cancel-private-display-repeat` 已通过，实际触发并完成取消；首次 IMU 间隔失效仍保留，尚未证明其根因消失。

### 运行问题 F-023：FAST-LIVO2 避障流程的对齐失效与短时过期

- 优先级：P1，集成根因待定。`fastlivo-obstacle` 尚未发出导航或执行绕障，不能记为避障算法失败或通过。
- `gcopter_3d_lane_v1` 场景中原生 FAST-LIVO2 持续 tracking，但共享 alignment 从 ready 转为 `reference_position_source_gap`，navigation 继而 `navigation_session_changed`；仅收到 8 条有效 aligned_pose。
- 探针在等待定位对齐 90 秒后失败，仍完成 Land 和上锁，24/24 子进程正常退出、无遗留。未放宽时间戳门限；安排相同参数的独立冷启动补测。
- 证据：`sitl/fastlivo-obstacle/vio.jsonl`、`console.log`、`result.json`；原始状态明确区分估计器仍运行与控制侧对齐失效。
- 同参数冷启动复测 `fastlivo-obstacle-private-display-repeat` 已进入真实绕行，但在约 `(3.486,-2.253,1.452)` m 时 navigation_input_stale / aligned_output_stale 导致导航中止，随后状态恢复 ready。复测最终也正常落地上锁，24/24 进程正常退出。
- 独立诊断审计保留整体失败：本轮位置 RMSE 0.00998 m、最大误差 0.02569 m，绕障轨迹几何检查通过，但任务没有完成；不能用局部几何通过覆盖运行失败。故障附近 aligned_pose 最大到达间隔约 1.05 秒，对应源戳相差 0.333 秒；全程最大 1.76 秒发生在起飞前，不能将其当成本次飞行失效的直接证据。具体延迟环节尚未确定，见 `fault-stale-timing-correlation.json`。
- 两轮失败原因不同；首轮参考位置源时间断档没有在复测中再次出现，不把它与临时新鲜度失效直接认作同一根因。补测证据：`sitl/fastlivo-obstacle-private-display-repeat/state-transitions.json`、`evaluation.json`。


### 运行问题 F-024：OpenVINS 观测模式的默认运动用例未完成初始化

- 优先级：P2，初始化可用性与输入时序问题，根因待定；主导航模式此前 11 项均通过。
- `openvins-dynamic` 使用观测模式的 `estimator_config.yaml` 与默认 0.3 m/s 起飞，估计器持续等待初始化，没有输出可用位姿；原生日志多次报告 static init 未检测到所需加速度变化。
- 源时间约 46.76 秒时 guard 短暂变为 `input_stale`，探针按初始化健康检查中止；随后 Land 成功、上锁，21/21 子进程正常退出，没有强杀/遗留。
- 该用例名称 dynamic 表示运动中的观察测试，不表示启用了上游动态初始化器；当前配置的 `init_dyn_use` 为 false。保留默认动作失败，另请求 1.0 m/s 起飞并执行 0.4 m 实际平移补测初始化运动条件，不改变配置、输入时间戳或精度门限。
- 证据：`sitl/openvins-dynamic/state-transitions.json`、`console.log`、`result.json`；补测矩阵为 `matrix-localization-extended-recovery.json`。
- `openvins-dynamic-initialization-motion` 补测也失败：起飞和平移命令都成功，随后等待 90 秒仍为 waiting_for_estimator_output / waiting_for_localization_initialization，实际输出 pose 数为 0；正常 Land，上锁，21/21 子进程正常退出。保留不同运动条件的两轮失败，不再以重复试跑替代问题记录。


### 运行问题 F-025：OpenVINS 四核限制下的对齐失效与压力精度超限

- 优先级：P1，限制资源下的集成可用性问题，根因待定。
- `openvins-cpu-pressure` 整个仿真子树限制在 CPU 0–3。初始化起飞、落地建图后，在第二次起飞准备期 alignment 锁定 `aligned_reference_discontinuity`，navigation 随后 session_changed；60 秒内未恢复可用地图/定位，因此未发送导航。
- 失败轮真值诊断也未过精度门限：191 个匹配、0 个缺失，位置 RMSE 0.34882 m（门限 0.30 m）、最大误差 0.94643 m；该段为失效前仍发布的位姿，未以丢弃失败数据换取通过。
- 两个 75% 占空竞争进程尚未启动，`cpu_pressure=null`；不能把本轮记录写为“完成四核加双竞争压力测试”。实际发生的是四核限制下的准备阶段失败。
- 最终 Land 成功、上锁、28/28 进程正常退出，未强杀、无遗留；保持全部门限，另行同参数冷启动复测。
- 证据：`sitl/openvins-cpu-pressure/result.json`、`vio.jsonl`、`evaluation.json`（失败轮只读诊断审计，不改变运行失败判定）。
- 冷启动复测 `openvins-cpu-pressure-cold-repeat` 实际启动两个 75% 占空竞争进程，覆盖完整往返导航，所有动作与清理通过；但独立真值审计最大位置误差 1.07717 m 超过 1 m 门限，整体仍失败。RMSE 0.15835 m，820 个匹配、0 个缺失；最终降落附近还出现 aligned_reference_discontinuity。不能以动作成功替代定位精度验收。
- 复测 28/28 子进程与两个竞争进程均正常结束、无遗留，最终上锁。复测和首轮表现不同，共同说明本轮未通过该受限资源场景的完整验收；不据两轮结果推断所有四核硬件都不适用。


### 运行问题 F-026：部分场景在本轮软件渲染下未通过起飞前时序检查

- 优先级：P2，仿真运行/测试环境耦合问题，尚未证明硬件或某个算法故障。首例为 `scene-bookstore_aws_v1`。
- 书店场景资源、组件、相机/深度/Livox 数据及地图已实际加载，但在地面静止检查的 60 秒期限内没有获得连续 3 秒就绪状态，未发送任何飞行任务。
- 该轮仿真源时间/墙钟比约 0.261；`/uav_state` 到达间隔中位数 0.400 秒、最大 1.459 秒。原门限要求到达年龄≤0.3 秒；即使忽略其他健康条件，记录推导出的最长连续新鲜状态也只有 1.104 秒，不能满足 3 秒要求。不能把超时误记为书店导航碰撞或任务飞行失败。
- 本轮使用私有 Xvfb/Mesa；不降低原保护门限，不据此声称所有登记场景均不可用。24/24 子进程正常退出、没有强杀或遗留，最终记录仍未解锁。
- 住宅 `scene-house_aws_v1` 同样在地面连续就绪检查超时；状态到达中位间隔 0.503 秒、最大 2.247 秒，连续新鲜状态上界仅 0.599 秒，仿真/墙钟速度比约 0.201。未发送任务，24/24 正常退出。见该目录 `stage-summary.json`。
- 赛道 `scene-racetrack_aws_day_v2` 在地面原生心跳检查中失败，未发送任务，24/24 正常退出。原脚本将“到达间隔超过 2 秒”或“源戳不递增”都报为 `native heartbeat cadence too slow`；此轮没有保存失败分支的详细心跳样本，不能断言具体是哪一个条件。该现象归入地面时序问题，根因不强行与前两例合并。
- 证据：`sitl/scene-bookstore_aws_v1/result.json`、`states.jsonl`、`preflight-timing-analysis.json`。

### 运行问题 F-027：部分登记入口启动后地图无法就绪

- 优先级：P1，集成原因待定。`scene-forest_pine_v1` 在 75 秒启动期限内只有 1 条占据栅格消息、没有 `/pointcloud_nonground`；相机、深度、Livox 雷达和 IMU 已有实际数据，状态流到达间隔正常，日志反复报告“点云对应源时间的 TF 等待超时”。
- 此轮未通过启动检查，未进入参考任务或发出飞行动作，24/24 进程正常退出、无遗留。不能把地面模型加载通过等同于导航链可用。
- `scene-primitives_sparse_seed42_v1` 也在 75 秒启动检查中超时，只有 1 条占据栅格消息、没有 nonground 点云，重复记录相同 TF 等待超时；未发送飞行任务且 24/24 正常退出。其他轮次在此场景有成功记录，不能认定该场景必然失败。
- 有屋顶仓库 `scene-warehouse_aws_v1` 同样在 75 秒启动期限内仅 1 条栅格、无 nonground 点云，未发出飞行任务；24/24 进程正常退出。
- 先前登记通道的取消补测也在启动阶段出现相同的地图不可用表象；不据此认定两轮底层原因完全相同。完整源时序/TF 根因仍需后续修复调查。
- 证据：`sitl/scene-forest_pine_v1/result.json`、`console.log`、`inputs.jsonl`；登记通道补测见对应执行明细。

### 运行问题 F-028：登记参考任务飞行中因导航输入无效或过期中止

- 优先级：P1，场景与时序集成原因待定。`scene-hospital_aws_v2` 已通过检查和起飞，执行登记往返任务时返回 `step_failed:navigation input invalid or stale`，父动作 ABORTED。
- 中止位置约 `(0.305,0.002,1.425)` m；终态处最后地图/点云已约 1.078 秒未到达，里程计仍有消息。原保护实际中止任务，没有完成两段参考导航及降落。
- 任务失败策略为 abort，探针随后关闭仿真；最后记录 `armed=true`。24/24 进程正常退出只能证明资源清理，不能记为成功落地、上锁或任务完成。
- 该表象与 F-015 相似，尚未证明相同根因；保留原场景、原任务及全部时间门限。证据：`sitl/scene-hospital_aws_v2/result.json`、`stage-summary.json`、`inputs.jsonl`。
- 无屋顶仓库 `scene-warehouse_aws_no_roof_v1` 也在飞行中返回同一失败消息，中止位置约 `(0.868,-0.001,1.426)` m。此轮终态前收到的是源时间戳 0 的地图及宽度 0 的点云，不能描述成与医院完全相同的无消息过期；最后仍 `armed=true`，24/24 进程正常退出。见对应 `stage-summary.json`。

### 运行问题 F-029：原生 Gazebo 模型查询在退出时发生线程自连接异常

- 优先级：P2，原生辅助进程运行问题，根因待定。`orbslam3-map-relocalization` 构建原生照片墙时，`gz model --pose` 已输出正确坐标 `4 0 2 0 -0 0`，随后报 `boost::thread_resource_error` / `trying joining itself: Resource deadlock avoided`，查询非零退出导致原探针拒绝夹具。
- 模型 info 中实际有正确名称与 collision，但不能忽略子进程异常而强行当作校验通过。没有修改原断言，没有重用这轮部分夹具；按同参数另开冷启动补测。
- 本轮 Atlas 原生加载已读入本次新建的 19 个关键帧、2035 个地图点，但 RGB-D 准入尚未打开、没有发送飞行任务，不能据此声称完成重定位。23/23 主栈进程正常退出、无强杀或遗留；模型查询异常与主仿真正常退出分别记录。
- 证据：`sitl/orbslam3-map-relocalization/orb_loop_wall_0.native-pose.txt`、`orb_loop_wall_0.native-info.txt`、`result.json`。该轮分类为重定位前置条件未满足，保留原失败。
- 同参数冷启动补测 `orbslam3-map-relocalization-cold-repeat` 已完成真实重定位及全部原断言；未再出现查询退出异常。首次异常仍保留，不声明已修复。

### 运行问题 F-030：ORB RGB-D/IMU 观察模式出现未伴随 epoch 变化的公共位姿大跳变

- 优先级：P1，定位集成问题，具体上游原因待定。`orbslam3-live-inertial-loop` 实际完成 1 次回环，惯性初始化及真实运动完成，但独立真值精度审计失败：3111 个匹配、0 个缺失，位置 RMSE **16.49634 m**（门限 0.3）、最大误差 **18.56398 m**（门限 1.0），完整姿态最大误差 0.06306 rad。
- 只读诊断按原审计的首次位姿 yaw/平移固定对齐复算，结果与原审计完全一致，没有尺度拟合、重对齐或丢弃失败点。仿真源时间 94.201→94.234 秒，公共位姿在 0.033 秒内跳变 **10.11077 m**，原生真值同期仅移动 **0.00931 m**；附近状态仍为 inertial_ready=1、map_id=0、correction_epoch=50。不能仅以固定初始坐标原点不同解释该跳变。
- 真实回环在后续 epoch 51/52 触发并完成，公共位姿计数冻结于 3111，后续输出隔离断言通过。它不能掩盖此前已发布的错误位姿；`correction-audit.json` 因依赖整体精度验收失败而仍为失败，不把它写成整项通过。
- 原始 RGB-D 四个话题各 5187 条，同源戳配对、标定/编码/坐标架校验通过；原始 IMU 34577 条、约 199.992 Hz、最大源时间间隔 8 ms，无无效输入。输入审计通过不能证明所有估计器或配置问题都已排除。
- 此轮 ORB 为旁路观察，**飞行控制使用 FAST-LIO，没有启用 ORB 对齐/控制桥**；60 个动作完成，正常 Land/上锁，主栈 24/24 子进程及观察器、录制器均正常退出，无强杀或遗留。不能将误差解释为无人机实际飞离十余米，也不能据此认定 RGB-D 导航模式同样失败。
- 证据：`orb-live-loop-summary.json`、`sitl/orbslam3-live-inertial-loop/evaluation.json`、`pose-jump-diagnosis.json`、`pose-error-trace.json`、完整原生输入 rosbag。误差诊断图已实际查看，见下图；图中全部匹配样本保持原固定对齐。

![ORB 惯性回环公共位姿与原生真值误差](artifacts/test-full-v1-20261023/sitl/orbslam3-live-inertial-loop/pose-error-diagnostic.png)

## 测试配置与未满足的前置条件（不计为运行产品缺陷）

- `precision-external-posctl-descent` 的 POSCTL 请求实际未生效：MAVROS 记录 command 176 / result 1，状态流始终没有进入 POSCTL，降落继续并返回成功。原用例要求外部接管后中止，因此判失败，但不能据此声称平台抢回控制或忽略了已确认接管；此项为接管注入前提未满足，已保存 `injection-summary.json`，原夹具冷启动复测仍未进入 POSCTL，带有效手动输入的补测另列。
- 夹具审查：本机 PX4 `ModeUtil/mode_requirements.cpp:76` 和 `checks/modeCheck.cpp:153` 要求 POSCTL 有有效手动输入。原探针没有提供该输入；已完成独立的居中摇杆输入补测，仍只连接测试拥有的仿真 ROS 栈，原模式确认、停止指令流和清理断言保持不变。消息编码按所安装版本的 [MAVROS 2.14.0 源码](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/manual_control.cpp#L82) 核对，发送端使用原始数值 z=500。

- `fuel-unreachable` 首次误用了 `fuel_room_v1` 普通房间，而 `run_fuel_matrix.py:23` 规定不可达用例使用 `fuel_sealed_v1` 封闭夹具及显式非受管调试入口。普通房间实际覆盖约 99.97% 并返回完成，不能证明不可达区域误报；该首次记录保留；`fuel-unreachable-sealed` 使用正确世界已通过实际运行与独立审计，未误报覆盖完成。
- `navigation-cancel-registered` 使用 4 m 目标，超出稀疏场景登记推荐 x≤1.8 m 的区域；动作在运动/取消触发前已结束，原探针未捕获结果载荷。此轮不证明取消功能失败。随后使用推荐范围 x∈[-0.5,6]、y∈[-3,3] 的 `gcopter_3d_lane_v1` 登记场景补测，目标、速度、动作和断言不变；但该组合 75 秒内没有建立可用 occupancy map，仅收到 1 条栅格消息、持续 TF 等待超时，未发出任何飞行动作。因补测夹具受阻，剩余登记入口变体不重复运行，功能场景继续由原项目 `matrix-core.json` 入口执行。旧记录保留，测试侧版本及哈希见 `harness-history/`。
- 登记导航入口的 27 进程适配、桌面管理工具首次错误 domain 等测试侧问题分别保留原始记录和纠正后的证据，不混入产品问题数量。

FUEL 补测证据：`sitl/fuel-cancel-cold-repeat/`、`sitl/fuel-unreachable-sealed/`；两项均通过。

两项 POSCTL 补测均通过：`precision-external-posctl-descent-manual-input-v2`、`precision-external-posctl-handoff-manual-input` 实际确认 POSCTL，动作按预期中止，平台及下游迟到输出均为 0，无子进程异常或遗留。见 `precision-posctl-supplement-summary.json`。原始用例缺少手动输入，结论为前置条件未满足；补测脚本第一次未创建输出目录，尚未启动仿真即失败，已作为本轮辅助脚本错误单列保存，不计为产品问题。

区域搜索冷启动复测已通过：`navigation-search-cold-repeat` 完成几何覆盖和正常降落，24/24 进程正常退出。该动作明确仅认证几何路径覆盖，不认证实际传感器覆盖。首轮悬停失败仍保留。

搜索取消冷启动复测已通过：`navigation-mission-search-cancel-cold-repeat` 实际取消搜索、递归停止子动作、保留已完成 1.65 m² 几何条带，随后新任务恢复并正常降落。搜索超时复测仍在准备悬停阶段受 F-011 阻塞，不能据此认定超时处理逻辑失败。

精准降落正常场景冷启动复测 `precision-nominal-cold-repeat` 通过，飞行和所有子进程退出均正常；这说明 F-016 本轮未在复测中再次出现，不代表崩溃已修复。

## 测试口径、环境与复现

- 基线为当前工作区（含测试开始前的未提交改动），HEAD 为 `4646fb4d7a1af523ffd434fbcf2344e78ece3158`；具体文件哈希和差异见 `baseline/`。本报告不把历史报告或旧运行结果计入通过数。
- Ubuntu 22.04.5、ROS 2 Humble、Gazebo Classic 11.10.2、PX4 1.16.2、GCC 11.4、Python 3.10；GPU 为 RTX 5060 Ti。详细版本见 `baseline/runtime-environment.json`。
- 111 个工作区包使用本轮独立的 `build/`、`install/` 重建并复制安装，开启 `BUILD_TESTING`、媒体和嵌入 RViz。PX4 使用既有固定哈希的兼容输入，不声称本轮重新编译了 PX4；Ceres、Pangolin、Fast DDS 源压缩包等已有依赖同样明确登记。
- 2026-10-24 继续执行时，宿主 `DISPLAY=:1` 的 GLX 建立失败，导致一项 Gazebo 启动受阻。后续使用测试拥有的私有 Xvfb 和 Mesa 软件渲染，启动/回收记录保存在 `display-runs/`，不改变相机数据、算法输入或精度阈值。
- 所有运行均为软件模拟、模拟接口或 PX4/Gazebo SITL。ROS 仅本机通信，仿真串行执行；硬件驱动仅编译。纯传感器补测没有飞控动作。
- 原始软件 CTest 入口数、JUnit 子项数、补充用例数、SITL 轮数分别统计，不能相加作“总测试数”。通过数不是代码行覆盖率，也不意味着任意参数组合均已验证。
- 运行成功需要满足原探针断言及退出检查；有独立审计的算法用例还需要审计通过。故障用例通过表示按预期拒绝、取消、锁定或停止，不等于完成导航或自动降落。
- 定位与原生规划审计使用只读 Gazebo 真值和源时间；定位只允许预先规定的初始姿态/平移对齐，不以真值替代算法输入或反馈控制。精度门限在运行前固定，不因失败放宽。
- 首轮失败、测试配置错误、补测和复测均分别保留。只修正本轮测试辅助入口或夹具；未修改产品实现来取得通过。实际未进入目标功能阶段的用例明确标注受阻。
- `navigation_dev`、旧 `perception_demo`、精准降落专用世界和登记参考任务分开列出；一个入口成功不证明另一个入口无故障。

证据根目录为 `artifacts/test-full-v1-20261023/`。每项外层命令、起止时间、返回码和超时保存在 `<用例>.execution.json`，标准输出保存在 `<用例>.console.log`；SITL 结果在 `sitl/<用例>/result.json`，独立审计使用 `evaluation.json` 或 `independent-evaluation.json`。测试配置清单为 `matrix-*.json`，汇总工具为 `collect_results.py` 和 `collect_simulations.py`。

复测需使用新用例 ID，保留本轮证据。例如只复测 SUPER 地图变化：

```bash
cd /home/jia/easyfly_ws
source artifacts/test-full-v1-20261023/simulation-environment.bash
python3 artifacts/test-full-v1-20261023/private_display.py \
  --tag super-map-change-recheck -- \
  python3 artifacts/test-full-v1-20261023/run-case.py \
  --id super-map-change-recheck --timeout 1200 -- \
  python3 artifacts/test-full-v1-20261023/run_sitl.py super-map-change-recheck \
  --family super --scenario map-change --managed
```

这是可执行的仿真复现命令，会向其新建的本机仿真发送飞行指令。全量构建、补建与软件套件的准确命令分别保存在 `build.sh`、`recover-build.sh`、`test-software.sh`；这些脚本使用本轮固定目录，重新完整运行时应先改用新的证据目录。

## 收尾核验与后续处理重点

本轮基线的 7539 个源码/脚本文件 SHA-256 全部保持不变，缺失 0；已暂存和未暂存差异与测试前一致，已有工作区改动保留。测试自有进程无遗留，所有私有显示服务已退出。证据：[最终工作区与进程审计](artifacts/test-full-v1-20261023/final-workspace-audit.json)。新增内容为本报告与独立证据目录中的测试辅助文件。

当前主链应优先处理坐标/无效状态接口（F-003/F-004）、安装后的资源路径依赖（F-013）以及地图、定位新鲜度和受限资源场景（F-015、F-023、F-025～F-028）；ORB RGB-D/IMU 观察模式的大跳变（F-030）应在考虑接入该模式控制前处理。原生退出崩溃和间歇性启动问题保留原失败及冷启动对照，不能用一次复测成功宣布修复。旧 `ego_to_mavros` 的 F-018～F-021 限定为启用该旧适配时的风险，不冒充当前主执行链缺陷。

覆盖边界仍包括：被地面准备失败挡住的具体飞行分支、9 个未运行登记入口变体、长期移动人物追踪、超出本轮持续时间的稳定性及任意参数组合。定位观测模式、导航控制模式与 RGB-D/IMU 回环观察的结论分别列出。后续修复应以新用例 ID 复测对应失败链，保留本轮原始证据。

## 全部 111 包的软件测试明细

下表为首轮全量软件套件口径；补测、冷启动复测和 SITL 单独列出，不覆盖原始失败/跳过。功能子项列不是 CTest 入口数。桌面 17 个跳过/禁用项已另行实际补测通过。

| 包 | 构建 | 功能入口通过/失败 | 功能子项通过/失败/错误/跳过或禁用 | 静态入口通过/失败 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `bspline_opt` | 通过 | 1/0 | 5/0/0/0 | 0/0 |  |
| `easyfly_apriltag_detection` | 通过 | 3/0 | 16/0/0/0 | 8/0 |  |
| `easyfly_astar` | 通过 | 1/0 | 15/0/0/0 | 0/0 |  |
| `easyfly_camera_calib` | 通过 | 2/0 | 7/0/0/0 | 8/0 |  |
| `easyfly_color_line` | 通过 | 1/0 | 5/0/0/0 | 6/0 |  |
| `easyfly_color_line_detection` | 通过 | 1/0 | 4/0/0/0 | 8/0 |  |
| `easyfly_connection_manager` | 通过 | 4/0 | 49/0/0/0 | 8/0 |  |
| `easyfly_coord_transform` | 通过 | 0/0 | 0/0/0/0 | 4/4 | 实际 ROS 补测：发现 F-003/F-004；见 uncovered-interfaces/ |
| `easyfly_core` | 通过 | 1/0 | 10/0/0/0 | 3/3 |  |
| `easyfly_coverage_planning` | 通过 | 1/0 | 2/0/0/0 | 0/0 |  |
| `easyfly_csrt` | 通过 | 2/0 | 3/0/0/0 | 0/0 |  |
| `easyfly_demos` | 通过 | 1/0 | 3/0/0/0 | 4/4 |  |
| `easyfly_depth_estimation` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 未实现骨架，不能算功能通过 |
| `easyfly_desktop` | 通过 | 12/0 | 156/0/0/15 | 5/1 |  |
| `easyfly_desktop_tests` | 通过 | 3/0 | 26/0/0/2 | 5/1 |  |
| `easyfly_detection_api` | 通过 | 1/0 | 2/0/0/0 | 0/0 |  |
| `easyfly_ellipse` | 通过 | 1/0 | 5/0/0/0 | 6/0 |  |
| `easyfly_ellipse_detection` | 通过 | 1/0 | 3/0/0/0 | 8/0 |  |
| `easyfly_fast_lio_integration` | 通过 | 2/0 | 3/0/0/0 | 0/0 |  |
| `easyfly_fast_livo2_integration` | 通过 | 2/0 | 19/0/0/0 | 0/0 |  |
| `easyfly_fastdds_compat` | 通过 | 12/0 | 5/0/0/0 | 0/0 |  |
| `easyfly_flight_control` | 通过 | 11/0 | 73/0/0/0 | 3/0 |  |
| `easyfly_fuel_core` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 原生增量规划、LKH、安装消费与 FUEL SITL 覆盖 |
| `easyfly_fuel_exploration` | 通过 | 2/0 | 9/0/0/0 | 0/0 |  |
| `easyfly_fuel_native` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 原生 worker、安装消费与 FUEL SITL 覆盖 |
| `easyfly_gazebo_models` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 20 场景资源检查、11 基础场景启动；参考任务另列 |
| `easyfly_gazebo_plugins` | 通过 | 22/0 | 50/0/0/0 | 0/0 | 功能套件通过；独立安装传感器补测发现 F-013 |
| `easyfly_gazebo_worlds` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 20 场景资源检查、11 基础场景启动；参考任务另列 |
| `easyfly_gcopter_core` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 安装后实际优化及 10 项 GCOPTER SITL 通过 |
| `easyfly_gimbal_control` | 通过 | 2/0 | 21/0/0/0 | 5/0 |  |
| `easyfly_global_planner` | 通过 | 13/1 | 58/0/1/0 | 2/6 |  |
| `easyfly_gps_receiver` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 未实现骨架，不能算功能通过 |
| `easyfly_grid_planner_api` | 通过 | 1/0 | 8/0/0/0 | 0/0 |  |
| `easyfly_ground_segmentation` | 通过 | 5/0 | 36/0/0/0 | 0/0 |  |
| `easyfly_image_detection_common` | 通过 | 0/0 | 0/0/0/0 | 6/0 | 由颜色线/椭圆/AprilTag 检测 ROS 集成间接覆盖 |
| `easyfly_landing_detection` | 通过 | 4/0 | 7/0/0/0 | 8/0 |  |
| `easyfly_lidar_camera_fusion` | 通过 | 4/0 | 9/0/0/0 | 4/1 |  |
| `easyfly_local_planner` | 通过 | 0/0 | 0/0/0/0 | 2/3 | EGO/combined SITL、启动配置及 12 种退出组合覆盖 |
| `easyfly_mavlink_bridge` | 通过 | 4/0 | 19/0/0/0 | 3/0 |  |
| `easyfly_mesh_comm` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 未实现骨架，不能算功能通过 |
| `easyfly_mission_format` | 通过 | 3/0 | 10/0/0/0 | 6/0 |  |
| `easyfly_mission_fsm` | 通过 | 4/0 | 26/0/0/0 | 0/0 |  |
| `easyfly_mission_scripts` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 11 登记参考任务 C++/Python validate-only 共 22 次通过；飞行另列 |
| `easyfly_mission_srvs` | 通过 | 7/0 | 36/0/0/0 | 8/0 |  |
| `easyfly_motion_prediction` | 通过 | 1/0 | 9/0/0/0 | 0/0 |  |
| `easyfly_msgs` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 生成消息/服务/动作接口由 ROS 集成及 SITL 使用 |
| `easyfly_multi_object_tracking` | 通过 | 2/0 | 6/0/0/0 | 8/0 |  |
| `easyfly_object_detection` | 通过 | 2/0 | 2/0/0/0 | 4/4 |  |
| `easyfly_obstacle_association` | 通过 | 1/0 | 4/0/0/0 | 0/0 |  |
| `easyfly_obstacle_avoidance` | 通过 | 1/0 | 2/0/0/0 | 4/4 |  |
| `easyfly_obstacle_geometry` | 通过 | 2/0 | 39/0/0/0 | 0/0 |  |
| `easyfly_occupancy_mapping` | 通过 | 3/0 | 17/0/0/0 | 0/0 |  |
| `easyfly_openvins_integration` | 通过 | 10/0 | 74/0/0/0 | 0/0 |  |
| `easyfly_orbslam3_core` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 安装后原生特征/投影、空 Atlas 启停保存；完整地图场景另列 |
| `easyfly_orbslam3_integration` | 通过 | 1/0 | 16/0/0/0 | 0/0 |  |
| `easyfly_params` | 通过 | 2/0 | 36/0/0/0 | 4/1 |  |
| `easyfly_perception_srvs` | 通过 | 2/0 | 5/0/0/0 | 5/0 |  |
| `easyfly_pixel_to_3d` | 通过 | 1/0 | 2/0/0/0 | 4/4 |  |
| `easyfly_pointcloud_filter` | 通过 | 1/0 | 1/0/0/0 | 4/4 |  |
| `easyfly_pointcloud_filters` | 通过 | 1/0 | 7/0/0/0 | 0/0 |  |
| `easyfly_precision_landing` | 通过 | 27/0 | 510/0/0/0 | 8/0 |  |
| `easyfly_projection` | 通过 | 1/0 | 5/0/0/0 | 0/0 |  |
| `easyfly_projection_py` | 通过 | 1/0 | 4/0/0/0 | 0/0 |  |
| `easyfly_px4_sitl` | 通过 | 11/0 | 56/0/0/0 | 0/0 |  |
| `easyfly_pybind` | 通过 | 1/0 | 0/0/0/0 | 4/4 |  |
| `easyfly_rc_receiver` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 实际 ROS 正常、空、18 通道转发及退出均通过 |
| `easyfly_semantic_map` | 通过 | 10/0 | 47/0/0/0 | 3/5 |  |
| `easyfly_serial` | 通过 | 1/0 | 1/0/0/0 | 0/0 |  |
| `easyfly_sim_launcher` | 通过 | 8/3 | 338/4/0/0 | 0/0 |  |
| `easyfly_single_object_tracking` | 通过 | 2/0 | 2/0/0/0 | 8/0 |  |
| `easyfly_single_tracking_api` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 单目标/CSRT 接口及安装消费覆盖 |
| `easyfly_sort` | 通过 | 1/0 | 6/0/0/0 | 0/0 |  |
| `easyfly_super_planner` | 通过 | 2/0 | 16/0/0/0 | 0/0 |  |
| `easyfly_swarm_planner` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 未实现骨架，不能算功能通过 |
| `easyfly_tag_pose` | 通过 | 1/0 | 5/0/0/0 | 0/0 |  |
| `easyfly_target_prediction` | 通过 | 1/0 | 5/0/0/0 | 8/0 |  |
| `easyfly_target_tracking` | 通过 | 2/0 | 11/0/0/0 | 5/3 |  |
| `easyfly_time_sync` | 通过 | 2/0 | 8/0/0/0 | 8/0 |  |
| `easyfly_track_id_fusion` | 通过 | 1/0 | 15/0/0/0 | 0/0 |  |
| `easyfly_tracking_api` | 通过 | 1/0 | 9/0/0/0 | 0/0 |  |
| `easyfly_trajectory_optimizer` | 通过 | 3/0 | 31/0/0/0 | 0/0 |  |
| `easyfly_udp_protocol` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 未实现骨架，不能算功能通过 |
| `easyfly_ui_bridge` | 通过 | 6/0 | 41/0/0/0 | 6/0 |  |
| `easyfly_ui_control` | 通过 | 4/0 | 49/0/0/0 | 5/1 |  |
| `easyfly_utils` | 通过 | 0/0 | 0/0/0/0 | 3/3 | 补充头文件 15 项：14 通过、GPS 高度失败 F-001 |
| `easyfly_vehicle_reid` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 未实现骨架，不能算功能通过 |
| `easyfly_video_recorder` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 未实现骨架，不能算功能通过 |
| `easyfly_video_streamer` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 未实现骨架，不能算功能通过 |
| `easyfly_web` | 通过 | 0/0 | 0/0/0/0 | 3/0 | 未实现骨架，不能算功能通过 |
| `easyfly_yolo_onnx` | 通过 | 1/0 | 2/0/0/0 | 0/0 |  |
| `ego_planner` | 通过 | 10/1 | 55/0/0/0 | 2/6 |  |
| `ego_to_mavros` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 已编译；隔离 ROS 接口补测 13 项：9 通过、4 失败（F-018～021）；非当前主执行链 |
| `fast_lio` | 通过 | 1/0 | 3/0/0/0 | 0/0 |  |
| `fast_livo` | 通过 | 1/0 | 2/0/0/0 | 0/0 |  |
| `librealsense2` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 设备测试排除，仅编译 |
| `livox_ros_driver2` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 设备测试排除，仅编译 |
| `livox_sdk2` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 设备测试排除，仅编译 |
| `ov_core` | 通过 | 0/0 | 0/0/0/0 | 0/0 | OpenVINS 原生定位 SITL 间接覆盖 |
| `ov_init` | 通过 | 0/0 | 0/0/0/0 | 0/0 | OpenVINS 原生初始化 SITL 间接覆盖；首构环境缺 Ceres |
| `ov_msckf` | 通过 | 0/0 | 0/0/0/0 | 0/0 | OpenVINS 实际估计器 SITL，精度另有真值审计 |
| `path_searching` | 通过 | 0/0 | 0/0/0/0 | 1/5 | EGO/API 安装消费与规划集成覆盖 |
| `plan_env` | 通过 | 6/0 | 32/0/0/0 | 0/0 |  |
| `quadrotor_msgs` | 通过 | 0/0 | 0/0/0/0 | 1/7 | 生成消息由 EGO 与飞控集成使用 |
| `realsense2_camera` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 设备测试排除，仅编译 |
| `realsense2_camera_msgs` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 设备测试排除，仅编译 |
| `rog_map` | 通过 | 0/0 | 0/0/0/0 | 0/0 | SUPER 安装消费与原生 SITL 覆盖 |
| `sophus` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 安装依赖消费覆盖；默认构建缺目录 F-002 |
| `super_planner` | 通过 | 0/0 | 0/0/0/0 | 0/0 | SUPER 安装消费、原生 SITL 覆盖；地图变化失败 F-012 |
| `traj_utils` | 通过 | 0/0 | 0/0/0/0 | 1/5 | 无独立功能套件；另看集成/消费覆盖 |
| `traj_utils_so` | 通过 | 0/0 | 0/0/0/0 | 0/0 | 活动轨迹头文件编译、EGO 安装消费与 SITL 覆盖 |
| `vikit_common` | 通过 | 0/0 | 0/0/0/0 | 0/0 | FAST-LIVO2 安装依赖消费覆盖 |


## 2026-10-26 软件／仿真补验结论

本轮 V 编号以 [未完成.md](未完成.md) 为准。V-15 十项范围、V-16 当前输入组合／完整栈与补充飞行、V-14 虚拟双机／两版 QGC／PTY 验收已按限定软件范围签收。当前源时间修复、全部原始失败、实际运行配置及仍未覆盖的硬件／纯 CPU 完整栈限制见[最终记录](artifacts/v01-v16-20261025/final-result.md)。此增量记录不把本文旧测试统计改写成当前全仓重跑，不关闭车尾漏检、许可或证据不足的历史异常。
