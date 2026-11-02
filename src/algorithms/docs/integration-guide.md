# 新增算法接入指南

新增算法应先确定能力契约，再完成实现、登记、配置和实际验证。公共登记见 [easyfly_backend_registry](../../easyfly_common/easyfly_backend_registry/README.md)，计算/数据约束见[接口契约](interface-contracts.md)与[感知版本契约](extension-contracts.md)。

## 1. 职责与目录

| 内容 | 位置与职责 |
| --- | --- |
| 项目维护的实现或普通包装 | algorithms/对应功能/easyfly_实现名；负责计算、资源限制与算法状态 |
| 原样导入的外部源码 | third_party/algorithms/主要功能/上游项目；保留上游布局、许可与补丁记录 |
| 公共能力接口 | 对应 API 包；不依赖具体算法、ROS 或飞控对象 |
| 节点、适配器与工厂 | 对应功能包；负责消息转换、输入接纳、配置、worker 和发布 |
| 飞行执行与会话 | 任务/执行层；负责准入、所有权、取消排空、接管与停止 |

按实际实现建立包，不新增空目录占位。消费者链接已安装 target/module，不读取相邻包源码。完整 ROS 引擎可以保持自身工程结构，通过平台协议接入。来源、版本、模型、许可及补丁按[第三方规范](../../third_party/sourcing-and-patches.md)登记。

## 2. 选择接口并实现适配

| 能力 | 接口 | 当前平台创建入口 |
| --- | --- | --- |
| 多目标跟踪 | `easyfly::tracking::Tracker` | `easyfly_multi_object_tracking::make_tracker` |
| 二维栅格规划 | `easyfly::grid_planning::GridPlanner` | `easyfly_global_planner::make_planner` |
| 检测 | `easyfly::detection::DetectorBase` | `easyfly_object_detection::createBackend` |
| 单目标跟踪 | `easyfly::single_tracking::Tracker` | `easyfly_single_object_tracking::make_tracker` |

定位、三维轨迹规划及探索继续使用专用适配和既有会话协议。二维 XY 路径不能冒充三维可执行轨迹；新增能力需要先设计相应输入、输出和执行边界。

多目标跟踪实现 `update(Frame)`、`reset(epoch)` 和 `capabilities()`，明确框、身份、类别、确认/丢失状态、置信度和速度单位。实例串行拥有；借用图像不能在调用返回后保留。缺少外观特征、原图或低置信度候选时，应拒绝不满足需求的输入。

| 新输入需求 | 必须补足的链路 |
| --- | --- |
| 原图 | 按源时间、相机、epoch 匹配；明确编码、stride、只读所有权、同步窗口与队列上限 |
| 外观特征 | 明确模型、维数/归一化、检测对应关系、推理依赖及预算 |
| 低置信度检测 | 确认检测上游未提前过滤候选，并提供明确的输入完整性声明 |
| px/s 速度 | 消费者识别单位，不能沿用 px/frame 或米制解释 |

当前多目标节点不能仅靠修改 Requirements 就获得上述新输入。调试图像订阅也不等于已按检测源时间同步的原图。

二维规划实现 `plan(Request, SearchControl)` 并回传地图身份；先验证布局、数值、边界与资源预算。明确未知格、占据阈值、穿角、端点及同格语义；失败不带可执行部分路径。需要取消、deadline 和展开预算时，在验证、搜索、回溯与剪枝中设置有界检查点，只声明实际支持的能力。

单目标跟踪使用独立的 `initialize(image, ROI)`、`update`、`reset`、`initialized` 契约。ROI 属于所显示源图像；跟踪丢失与初始化失败不能发布旧框作为有效结果。参考 [KCF 包](../tracking/easyfly_kcf/README.md)和 [CSRT 包](../tracking/easyfly_csrt/README.md)。

## 3. 登记、工厂与参数

1. 在 `easyfly_backend_registry/data/backends.json` 增加真实后端，并添加独立参数 schema；不得把候选算法参数伪装成 SORT/A* 的参数。
2. 在对应功能包的创建表注册 creator/normalizer，使用 `BackendConfig` 与参数映射，转换成该实现的强类型参数；创建后核对实际实例身份。
3. 声明 CMake/package.xml 依赖，私有链接实现，公开 target 只传播公共契约与配置依赖，详见[构建约定](build-conventions.md)。
4. 保留既有默认选择：SORT、A*、ONNX、CSRT。未知后端、字段、版本、非有限/越界参数及重复 JSON 字段明确失败，不静默切换算法。
5. 实例在启动或显式重新初始化时创建；没有目录扫描、动态插件或运行中热切换。生成的 C++ 登记按内容快照隔离符号，不能手工修改生成头。

连接管理页在编译时嵌入登记生成的 UI 选项。新增后端后，除实现和节点外，还须以新登记安装为依赖重建 `easyfly_connection_manager`；仅更新 Python 登记或启动环境不会改变旧界面的下拉框。验收应实际选择、保存并重新打开该选项。启动时使用新安装提供的 `easyfly_connections` 入口；旧管理后台若仍驻留，先确认无运行会话并正常退出，再从新安装启动，避免它重新打开旧窗口。

角色分别为 detection、multi_tracking、single_tracking、global_grid、combined_grid。全局和 combined 分别使用 `planner_backend/planner_parameters`、`combined_planner_backend/combined_planner_parameters`，不要把两者的选择与预算混为一项。

## 4. 配置贯通与状态观测

终端与桌面共用 `easyfly_params.backends.resolve_selection`。配置顺序为角色/schema 默认 → profile/安装 YAML → 配置文件 → 显式算法选项 → 结构化最终覆盖。新增登记应核对 UI 选项、保存/重载、profile、叶 launch、节点和实际创建参数；选择成功不代表实例或输入就绪。

只读检查，不启动节点：

```bash
ros2 run easyfly_params backend_selection --profile full_perception \
  --algorithms-json '{"single_tracking_backend":"kcf"}'
```

`--file` 和 `--backends-json` 可验证结构化选择；`--check-dependencies` 只检查安装资产存在，不赋予运行或飞行权限。ROS 参数映射必须作为字符串传递，例如：

```bash
ros2 run easyfly_single_object_tracking single_object_tracking_node --ros-args \
  -p tracker_backend:=kcf -p 'backend_parameters:="{detect_thresh: 0.5}"'
```

V1/V2 采用明确的 `perception_version` 配置，默认仍为 1；V2 应在整个数据链一致启用。话题、身份和兼容规则见[感知契约](extension-contracts.md)。

运行报告位于 `/platform/backends/<role>`，使用 DiagnosticArray：一个 DiagnosticStatus，name 为角色、hardware_id 为报告会话，header 为 ROS 时间。values 包含 `schema_version`、`requested_backend`、`active_backend`、`created`、`phase`、`provider`、`effective_parameters`、`sequence`。schema_version 为 1，序号在会话内递增，phase 为 creating/waiting_input/ready/failed；非检测角色 provider 为 not_applicable。

观察端核对唯一 DDS 发布源、会话/序号、ROS 与 steady 年龄、实际参数和预期选择。重放、过期、来源含糊和配置不一致不能维持就绪。`created` 只表示创建完成，`ready` 只表示对应近期输入条件满足，不参与解锁、任务准入、飞行所有权或停止授权；单目标 ready 也不等于已经成功跟踪某个目标。

## 5. 交付与验收

- 提交真实算法实现及数值对照，覆盖空/非法输入、资源限制、重置、身份和需求不满足。
- 验证真实节点的消息、坐标、时间、单位、worker、过期及故障行为；涉及任务时补齐取消、接管、停止和所有权释放。
- 验证配置到实例和实际参数的贯通；测试替身只证明协议，不能作为新增真实实现。
- 验证最小依赖、开发软链接、复制搬迁及安装后独立消费，检查真实编译头和动态加载路径。
- 更新包 README、来源/许可和使用方法；机器结果、源/模型/SDK 指纹及原始失败保存到 artifacts，不在本目录追加分批验收 Markdown。

公共分层入口：

| 入口 | 用途 |
| --- | --- |
| `tools/algorithms/verify_extension_acceptance.py` | 公共构建、回归、基线比较与报告；支持 build/test/report 分层和显式清单 |
| `tools/algorithms/verify_extension_migration.py` | 显式安装/underlay 下的受影响 CTest 与原生夹具 |
| `tools/algorithms/verify_extension_dependencies.py` | 最小依赖和两种安装消费矩阵 |
| `tools/algorithms/verify_single_tracking_install.py` | CSRT/KCF 安装消费及固定输入 CPU 对照 |
| `tools/algorithms/verify_extension_ui_features.py` | 显式媒体、原生 RViz 和真实离线工具检查 |

各工具参数以 `--help` 为准，输出使用新目录。源码、依赖或构建输入改变后重新生成相关证据，并更新清单中的证据前缀；历史 SHA 不变的报告不自动证明当前输入已通过。分别报告真实算法、协议替身、不适用和未覆盖项，跳过不计为通过。完整飞行、硬件和真实数据集精度具有各自验收范围。
