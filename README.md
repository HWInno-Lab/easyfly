# EasyFly

基于 **ROS 2 Humble、PX4 和 Gazebo Classic** 的自主无人机开发平台，将传感器、感知定位、地图、规划控制、任务编排和桌面操作组织在同一工作区中。

EasyFly 面向自主飞行算法开发、仿真联调和任务验证，提供 C++ / Python SDK 与 Qt 桌面应用。当前处于**仿真内测阶段**：核心任务链已有 PX4/Gazebo 验证，已有约定范围的软件、小时级运行和仿真验收；真实设备接入、实机验收和两项历史退出异常仍未全部完成。

**首次部署与内测请先阅读 [内测手册](内测手册.md)**，其中包含完整安装准备、构建步骤、功能用例、仿真飞行入口、通过标准和问题记录模板。

状态核对：**2026-10-29**。本文描述当前工作区快照；部分实现及本地验收证据尚未进入 Git。部署时须绑定完整源码、安装层和依赖清单，不能仅凭本次文档提交复现全部能力。

RealSense 驱动包含固定的本地构建适配。新克隆后先运行
`python3 tools/source/restore_realsense.py`；它使用随源码交付的 Git bundle 离线恢复
准确 gitlink，并拒绝覆盖已有改动。获取与升级边界见[驱动恢复说明](src/third_party/drivers/README.md)。

## 当前能力

| 方向 | 已有实现 | 支持范围 |
| --- | --- | --- |
| 连接与仿真 | 本机仿真启动、已有平台 DDS 连接、MAVROS 通信配置、QGC 转发 | 桌面统一管理；已验隔离虚拟双机 DDS、QGC 4.4.3／5.0.8 与 PTY 断链恢复，实体链路待硬件 |
| 感知与跟踪 | YOLO ONNX、AprilTag、椭圆/彩线检测、CSRT、SORT、ID 融合和目标预测 | 部分组件使用独立启动入口；检测输出不直接授予飞行权限 |
| 定位与建图 | FAST-LIO、地面分割、障碍几何与关联、占据地图、雷视投影 | FAST-LIO 为既有完整链的默认定位后端 |
| 其他定位后端 | OpenVINS、FAST-LIVO2、ORB-SLAM3 的守卫、机体对齐、状态及显式 Gazebo 导航 | 三后端已有受限场景与 Qt 受管导航验收；显式 ORB 惯性 Atlas、LiDAR 地图重用和双定位外部视觉切换另有原生仿真验收，分别遵守包内运行条件 |
| 规划与控制 | A*、EGO、combined、GCOPTER、SUPER 导航，FUEL 有界探索、目标跟随、起降与轨迹执行 | 指定场景和参考任务已有验证；包含取消、失效及外部接管处理 |
| 任务与 SDK | 顺序任务、区域搜索、任务服务、C++ SDK 和 Python 绑定 | Python 绑定复用 C++ SDK；动作结果与控制权释放分别跟踪 |
| 精准降落与云台 | 静止板精降、实测单轴动态外参、低速移动板原生仿真 | 独立 D5 配置；移动板的速度、面积和 Land 时限须显式声明，真实硬件尚未验收 |
| 桌面工具 | 任务编辑、同帧感知显示、离线地理底图、完整机体显示、录像与隔离回放 | 默认只读；任务执行、导入和元数据写入需要显式启用 |
| 通信与媒体 | 独立 NMEA GPS、签名 UDP／Mesh、录像、HTTPS 视频和 Web 工作台 | 已验软件／协议边界；实体串口与射频另验，Web 默认只读 |
| 多机与独立感知 | 受管多机仿真、协同规划、车辆重识别、双目米制深度、在线相机标定 | 原生飞行范围为双机；独立输入、标定与显式配置不可省略 |

GPS 接收、Mesh 通信、独立 UDP 协议、独立录像、视频流、Web、集群规划、车辆重识别、独立深度估计均已实现并完成各自声明的软件／协议或仿真验收。双机集群通过原生任务与失效检查；三／四机性能、实体串口、射频网络和实景识别精度仍需独立验证。阶段 1–3 共 24 项中已有 22 项签收，V-05／V-07 历史退出异常继续保留；逐项范围与证据见[未完成清单](未完成.md)。

## 架构与目录

```mermaid
flowchart TD
    UI[Qt 桌面 / C++ 与 Python 应用] --> API[SDK / 平台服务 / 任务编排]
    API --> EXEC[导航 / 跟随 / 飞行动作]
    SENSOR[相机 / 雷达 / IMU] --> PERCEPTION[感知 / 定位 / 地图]
    PERCEPTION --> EXEC
    EXEC --> CONTROL[统一飞控]
    CONTROL --> BRIDGE[EasyFly MAVLink Bridge]
    BRIDGE --> MAVROS[MAVROS]
    MAVROS <--> PX4[PX4 SITL / 飞控]
```

飞行控制统一通过 `easyfly_flight_control → easyfly_mavlink_bridge → MAVROS` 下发；任务和算法通过约定的消息、Action、服务及会话协议协作。源时间、坐标系、输入有效性、动作所有权和取消排空由各层明确处理。

```text
easyfly_ws/
├── src/
│   ├── communication/       # 驱动、飞控协议、媒体与网络模块
│   ├── perception/          # 预处理、检测、跟踪、定位与融合节点
│   ├── execution/           # 任务状态机、规划与控制
│   ├── application/         # SDK、服务、任务与仿真入口
│   ├── algorithms/          # 可复用计算库与能力接口
│   ├── easyfly_common/      # 公共消息、工具与任务格式
│   ├── easyfly_ui/          # 桌面、连接管理、观察与控制适配
│   ├── simulation/          # 场景、模型、插件及仿真兼容层
│   └── third_party/         # 外部算法、运行库、设备 SDK 与工具资料
├── models/                  # 推理模型资产
├── tools/                   # 算法检查、场景准备与诊断工具
├── scripts/                 # 开发与诊断辅助脚本
├── build_all.bash           # 全量构建包装
├── build_layer.bash         # 按层及依赖构建
└── 内测手册.md              # 安装、操作与测试指南
```

项目维护的算法部分目前包含 **4 个能力 API 和 17 个计算组件**。普通算法库通过安装后的 CMake target 或 Python module 消费；FAST-LIO、EGO、OpenVINS 等完整 ROS 工程保留其结构，通过平台适配接入。新增算法参见 [算法索引](src/algorithms/README.md) 和 [接入指南](src/algorithms/integration-guide.md)。

## 环境与安装准备

当前主要开发环境为 **Ubuntu 22.04 x86_64**。完整平台依赖以下已验证版本及兼容输入：

| 组件 | 当前基线与要求 |
| --- | --- |
| ROS 2 | Humble，使用系统兼容的 Python 3.10 / GCC 11 环境 |
| PX4 | 1.16.2，以及项目审核过的仿真 HRT 兼容构建；部分场景另需磁力计采样间隔修复 |
| Gazebo Classic | 11.10.2，私有兼容层同时校验相关系统头文件和库的指纹 |
| Fast DDS | 2.6.11，受管场景使用项目私有兼容库 |
| Qt / OpenCV | Qt5；本机基线为 Qt 5.15.3、OpenCV 4.5.4 |
| 内嵌 RViz | 启用时要求 common/rendering/default_plugins 11.2.26 |
| 检测运行库 | ONNX Runtime C/C++ SDK 1.23.2 与项目模型；车辆重识别另需同版 Python 运行库与 OSNet 参考库，CUDA 还需匹配宿主机依赖 |
| 设备与定位依赖 | Livox SDK、librealsense、GeographicLib 数据；OpenVINS 需要 Ceres，ORB-SLAM3 需要独立 Pangolin 前缀 |

完整安装需要准备系统依赖、PX4 源码与原始构建、兼容输入、ORT SDK、模型和场景资源。**仅克隆 Git 仓库不能得到完整可运行环境**：ORT 载荷、构建目录和本地缓存不入库，RealSense 嵌套仓库需按随附 bundle 恢复并核对固定提交。

隔离离线版本目录的安装、升级、卸载和配置迁移工具见 [交付说明](tools/delivery/README.md)。其实际验收状态以交付摘要为准。

既有 `offline-r3` 归档冻结于此前交付验收，不包含本轮阶段 1–3 的全部新增功能。
本轮运行证据来自 `artifacts/stages1-3-20261028` 的复制／符号链接安装及其明确的 underlay；
不能把旧归档的验收结论当作最新源码的全新离线安装验收。

安装步骤和检查清单见 [内测手册](内测手册.md)。部署时保留精确版本与文件指纹；遇到不匹配应使用已审核输入或重新适配，不通过删除版本/SHA 校验启动。当前尚无经过完整验收的自包含 deb/AppImage 安装包。

## 快速开始

### 使用已部署的完整平台

先按[内测手册第 4 节](内测手册.md#4-路线一在当前已部署内测机使用)核对本轮安装入口；应用菜单可能仍指向旧版本。打开已核对的 **EasyFly**，进入“连接与仿真”：

1. 新建连接，选择“启动本机仿真”。
2. 首次检查使用 `base_sim` 和 `primitives_sparse_seed42_v1`；需要感知、建图与导航时选择 `navigation_dev`。
3. 核对场景、PX4 兼容输入、DDS domain 和模拟时间，点击“启动仿真并连接 DDS”。
4. 在总览确认真实连接和新鲜状态，再按内测手册开展观察或任务测试。

安装环境已加载时，也可启动统一入口：

```bash
ros2 run easyfly_connection_manager easyfly_connections
```

桌面包含七项导航：**连接与仿真、运行总览、任务中心、相机与感知、空间视图、记录与回放、诊断与设置**。启动后台不会自动解锁或提交飞行任务。QGC 联网维护具有双向控制能力，与 EasyFly 的任务执行、导入及参数修改互斥。

“结束观测”或关闭工作窗口会保留后台。停止后台需通过管理入口，并满足新鲜、未解锁、在地面和无未决任务等条件。详细操作见 [连接管理说明](src/easyfly_ui/easyfly_connection_manager/README.md) 与 [桌面操作说明](src/easyfly_ui/easyfly_desktop/OPERATIONS.md)。

### 通过终端选择场景

使用本次完整安装及其登记的 underlay。新构建使用安装的 `setup.bash`；本机多层验收环境按手册加载配套 `environment-copy.bash` 或 `environment-symlink.bash`。将下面两个占位路径替换为实际部署目录；兼容目录应包含 `compatibility.json` 和 `px4-input/`：

```bash
source /opt/ros/humble/setup.bash
export EASYFLY_INSTALL=/absolute/path/to/verified/install
source "$EASYFLY_INSTALL/setup.bash"
export EASYFLY_PX4_HRT_COMPAT_DIR=/absolute/path/to/reviewed-px4-compat
export ROS_DOMAIN_ID=84
export ROS_LOCALHOST_ONLY=1
export PYTHONNOUSERSITE=1

ros2 run easyfly_sim_launcher scene_catalog list --json
ros2 run easyfly_sim_launcher scene_catalog validate primitives_sparse_seed42_v1 --json
ros2 launch easyfly_sim_launcher profile.launch.py \
  profile:=navigation_dev scene:=primitives_sparse_seed42_v1 headless:=true rviz:=false
```

本例只启动平台，不发送飞行 Goal；不要与 UI 已启动的后台并行运行。场景离线校验通过只表示资源满足检查，不代表当前飞机、地图或任务已就绪。`headless:=true` 关闭 Gazebo 客户端，相机渲染仍需要可用的图形环境。

| Profile | 用途 |
| --- | --- |
| `base_sim` | 基础飞控状态，不启动 SLAM 或导航链 |
| `full_perception` | 完整感知及相关平台能力 |
| `navigation_dev` | 导航开发，默认 combined 策略 |
| `landing_dev` | 感知/TF 准备；精准降落使用独立配置和入口 |
| `hardware_unverified` | 未验证硬件占位，禁止作为可运行方案启动 |

当前场景目录共 20 项，其中 14 项已登记、6 项为草稿；11 个登记场景支持 `base_sim`，其余 3 个仅支持 `navigation_dev`，覆盖基础几何、仓库、住宅、书店、办公室、医院、森林、赛道和街区。以安装目录的 `supported_profiles` 与资源检查为准；`empty` 等草稿不能通过受管选择启动。场景和参考任务说明见 [仿真启动器](src/application/a3_mission/easyfly_sim_launcher/README.md)。

### 仅构建独立只读桌面

已有兼容的系统 ROS、Qt 和开发依赖时，可以先构建桌面的六包依赖闭包。它可以观察已有平台、编辑本地草稿；完整仿真和连接管理需另外部署。

在工作区根目录的新终端执行，只加载系统 ROS。以下使用独立目录，关闭录像和内嵌 RViz：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log-readonly build --base-paths src \
  --build-base build-readonly --install-base install-readonly \
  --packages-up-to easyfly_desktop --executor sequential \
  --cmake-args -DBUILD_TESTING=ON \
  -DEASYFLY_ENABLE_MEDIA=OFF -DEASYFLY_ENABLE_RVIZ_EMBED=OFF
source install-readonly/setup.bash
ros2 run easyfly_desktop easyfly_desktop
```

默认使用模拟时间，没有 `/clock` 时实时数据不可用。连接使用系统时间的真实数据源时，按 [桌面说明](src/easyfly_ui/easyfly_desktop/README.md) 选择 `--system-time`，并匹配实际 domain 和 topic。

完整平台的 SDK 准备、编译参数与可选能力安装见内测手册。`build_all.bash` 和 `build_layer.bash` 是构建包装，不负责自动安装这些依赖或准备 PX4 兼容输入；按层选择可先用 `./build_layer.bash p --dry-run` 查看。

## 测试与内测

测试按“安装检查 → 软件组件 → 只读观察 → 指定仿真任务 → 异常与压力专项”推进。每轮保留源码、依赖、配置和原始结果；行为检查、规范检查、跳过项及硬件未测项分别报告。

上面的独立桌面构建可执行软件检查：

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=226
export QT_QPA_PLATFORM=offscreen
colcon --log-base log-readonly-test test --base-paths src \
  --build-base build-readonly --install-base install-readonly \
  --packages-select easyfly_msgs easyfly_core easyfly_mission_format \
  easyfly_ui_control easyfly_ui_bridge easyfly_desktop \
  --executor sequential --return-code-on-test-failure --ctest-args -j1 --output-on-failure
colcon test-result --test-result-base build-readonly --verbose
```

这是软件/夹具检查，不证明真实图形渲染或飞行通过；GUI 人工检查应使用正常图形环境。完整平台还需包含 `easyfly_desktop_tests` 等联合用例。集成测试含固定 domain，按手册串行运行。

导航、跟随、电池、场景参考任务和精准降落有**显式 SITL 验收入口**，会发送仿真飞行动作，不由普通 `colcon test` 自动执行。参数、前提、取消与收尾标准统一见 [内测手册](内测手册.md)。

需要区分：**请求接受、动作终态、控制权释放、实际落地、进程正常退出**。取消 ACK 或窗口关闭不能替代执行结束的证据。

## 当前验证边界与剩余工作

当前源码发现 **118 个包**，两种安装均核对到 115 个 ament 包与 3 个原生 CMake 库。阶段 1–3 的 24 项中 **22 项已按声明的软件／协议或原生仿真范围签收**，包括原九个骨架模块及 E-01～E-10。包数、测试条数和完成事项数分别统计。

已有验收包括双机协同、惯性 Atlas／激光地图重用、双定位外部视觉融合、动态相机与移动板精降、独立媒体与网络服务。每个模块仍有明确的输入、场景、速度和生命周期边界；专项入口与复验条件见[内测手册第 13 节](内测手册.md#13-定位与新增能力专项)。

| 未完成内容 | 当前处理 |
| --- | --- |
| V-05／V-07：Gazebo／独立观察器历史退出异常 | 当前原场景及退出复测通过，缺少原异常阻塞堆栈，根因仍未关闭 |
| B-01：FUEL 取消后重新探索偶发地图失效 | 按用户决定暂缓；正常复跑不替代失败／修复对照 |
| V-09：近距离卡车车尾漏检 | 跟随控制已修，识别质量问题按用户决定保留 |
| D-04：来源、许可与组合分发材料缺口 | 按用户决定暂缓；规范检查通过不等于许可齐备 |
| 真实云台、D435i 双目惯性产品接入、目标机／传感器／飞控／RC、HIL／实飞 | 含待实现接口与待设备验收项；仿真不能代替实体链路 |
| 最新版本自包含离线交付 | 已验的旧 `offline-r3` 保持冻结，最新 118 包工作区尚未重新制作并验收完整离线归档 |

14 个登记场景的既有任务矩阵、小时级运行、虚拟双机 DDS／QGC／PTY、公开数据定位基线均保留各自的验收范围。纯 CPU 四核双负载的锁步验证约为 **0.16 倍实时速度**，不代表实时吞吐；GPU 与纯软件渲染也分别验收。三／四机原生飞行性能、更广实景识别和任意高速导航属于尚未覆盖的扩展。

本机完整证据见[阶段汇总](artifacts/stages1-3-20261028/summary.json)、[当前退出审计](artifacts/stages1-3-20261028/shutdown-final-review.json)和[未完成清单](未完成.md)。这些本地记录按仓库策略单独交付，源码克隆可能不含它们；本节和随包说明保留可独立阅读的结论。

## 开发与文档

| 主题 | 入口 |
| --- | --- |
| 完整安装、内测用例、故障排查和报告 | [内测手册](内测手册.md) |
| 桌面与连接操作 | [连接管理](src/easyfly_ui/easyfly_connection_manager/README.md) · [桌面操作](src/easyfly_ui/easyfly_desktop/OPERATIONS.md) |
| 仿真与场景 | [仿真启动器](src/application/a3_mission/easyfly_sim_launcher/README.md) · [PX4 兼容输入](src/simulation/easyfly_px4_sitl/README.md) · [Fast DDS 兼容层](src/simulation/easyfly_fastdds_compat/README.md) |
| SDK | [C++ SDK](src/application/a1_sdk/easyfly_core/README.md) · [Python 绑定](src/application/a1_sdk/easyfly_pybind/README.md) |
| 算法开发 | [算法索引](src/algorithms/README.md) · [接口契约](src/algorithms/interface-contracts.md) · [构建约定](src/algorithms/build-conventions.md) · [新增算法接入](src/algorithms/integration-guide.md) |
| 新增模块与专项复验 | [内测手册第 13 节](内测手册.md#13-定位与新增能力专项) · [多机仿真](src/simulation/easyfly_fleet_sim/README.md) · [集群规划](src/execution/e2_planning/easyfly_swarm_planner/README.md) |
| 地图与精降 | [LiDAR Atlas](src/perception/p3_tracking/easyfly_lidar_atlas/README.md) · [双定位切换](src/perception/p3_tracking/easyfly_localization_failover/README.md) · [动态相机／移动板](src/execution/e3_control/easyfly_precision_landing/DYNAMIC.md) |
| 网络与原始记录 | [Web](src/easyfly_ui/easyfly_web/README.md) · [远端管理](src/easyfly_ui/easyfly_connection_manager/REMOTE.md) · [独立录像／历史回放](src/communication/c3_media/easyfly_video_recorder/README.md) |
| 定位与检测 | [OpenVINS](src/perception/p3_tracking/easyfly_openvins_integration/README.md) · [椭圆/彩线](src/algorithms/classical-detection.md) · [D435i 采集](src/application/a1_sdk/easyfly_params/config/sensors/README.md) |
| 外部依赖与来源 | [第三方清单](src/third_party/catalog.json) · [来源与补丁](src/third_party/sourcing-and-patches.md) · [推理运行库](src/third_party/runtimes/README.md) |

新增后端应同时提交实现、平台适配、配置、来源/许可证与对应检查。能力声明只填写实际支持项；变更公共消息或 C++ ABI 时同步重编译消费者。

`docs/`、开发计划、阶段验收报告和 `artifacts/` 中的本地证据按当前策略不入 Git。源码仓库中的使用说明与本轮交付包共同构成部署依据；需要复现历史验收时，应另外取得相应原始证据。

## 许可证

EasyFly 自有代码采用 [Apache License 2.0](LICENSE)，署名为 `EasyFly Project`。

第三方源码、设备 SDK、模型、场景资源及本地移植保留各自许可证和版权通知，根许可证不替代这些声明。部分历史资产的来源/许可仍有待补全项，具体见 [第三方来源与补丁](src/third_party/sourcing-and-patches.md) 和 [模型资产登记](src/third_party/model-assets.provenance.json)。
