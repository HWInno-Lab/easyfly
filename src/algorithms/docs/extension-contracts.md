# 感知版本与后端配置契约

本文约定 V1/V2 感知的身份、时间、观测来源、兼容迁移和公共后端配置。对应消息位于 `easyfly_msgs`，公共接纳实现见 [easyfly_perception_contracts](../../easyfly_common/easyfly_perception_contracts/README.md)，公共登记见 [easyfly_backend_registry](../../easyfly_common/easyfly_backend_registry/README.md)。启用和配置方式见[接入指南](integration-guide.md)。

## 1. 已决定的边界

1. 像素运动、世界运动使用独立字段/类型。旧 `TrackInfo.velocity_*` 不原地改单位，不由帧率推算 m/s。
2. 新增独立 V2 感知消息与版本化话题；保留 V1 定义、类型名和旧话题。新旧消息不混发到同一话题。
3. 跟踪身份使用 UUID session、epoch、int64 raw ID 组合；稳定 ID 独立标记，不与 raw ID 混用。
4. 输入观测、滤波输出和纯预测明确区分；无观测的预测框不进入深度融合/外观识别。
5. 公共后端登记归`src/easyfly_common/easyfly_backend_registry`，不放在 UI、SDK 或某个具体算法包。
6. 首期静态登记、强类型 creator、启动时选择；不做动态加载、运行中切换或静默算法回退。
7. 导航策略与计算后端分开。combined 的栅格后端有明确选择，不继承独立节点的隐含状态。
8. 定位、地图、轨迹、飞行 Action 的现有会话协议继续保持；公共定位和三维规划抽取由后续按需批次实施。

## 2. 数据契约

### 2.1 通用观测上下文

V2 检测、跟踪、三维目标数组均包含下列上下文；具体 ROS 类型名冻结为 `PerceptionContextV2`：

| 字段 | 类型/语义 | 生成与接纳规则 |
| --- | --- | --- |
| `header` | 源观测时间与该数组几何所在坐标系 | 图像结果用光学系；三维结果用明确世界系；不得以发布时刻替换源观测时间 |
| `camera_id` | 长度 1..32 的配置内稳定相机标识 | 匹配已校验的图像/内参/深度绑定；不是任意 frame_id 的别名 |
| `image_width/image_height` | uint32，原图尺寸 | 来自实际输入帧，均大于 0；禁止用固定 640×480 覆盖真实尺寸 |
| `producer_session_id` | `unique_identifier_msgs/UUID`，非零 | 每个节点实例/输出流启动生成；不借用可重复的进程号或只用时间戳 |
| `epoch` | uint64，实例内重置代次 | 从 0 开始，reset、时钟回退、输入写者/上游 session 更换时递增；溢出即停止并要求新实例 |
| `sequence` | uint64，实例/epoch 内结果序号 | 严格递增；重复不更新接收时效；计数溢出按 epoch 变化处理 |
| `status` | 有效、未就绪、输入无效、输入不支持、资源耗尽、后端失败、过期 | `valid + empty` 表示有效空观测；错误不能编码为成功空数组 |
| `reason` | 有界 UTF-8 字符串，最长 256 字节 | 诊断用途，不作为消费者状态机唯一依据 |

上下文标识的是**该结果流的生产者**，不宣称是相机硬件会话。相机写者身份、时钟域与绑定由接纳层监测；发生变化必须清空缓冲并改变 epoch。仅靠 ROS Header 无法证明跨发布者数据同源。

跨节点结果必须保留明确的上游引用 `ObservationRefV2`：上游 producer session、epoch、sequence 及原始观测 header。三维结果另记录实际使用的深度时间与 TF 时间/查询方式。V2 不读取最近一帧调试图冒充算法原图。

源时间和 steady 接收时间只在各自时钟域计算年龄。steady 时间不作为可跨机器比较的 ROS 字段发送。上下文 session/epoch 与源时间共同检查，不能仅凭较新的时间戳接纳旧会话。

### 2.2 跟踪身份与数组

`TrackKeyV2`：

- `tracker_session_id`：跟踪节点实例 UUID。
- `tracker_epoch`：跟踪历史代次。
- `raw_id`：int64，非负，算法实例内身份；作为深度定位与世界坐标回流的关联键。
- `stable_id_valid` 与 `stable_id`：稳定 ID 及其有效性；独立于 raw ID，其作用域仍由 tracker session/epoch 限定。

对外身份是完整组合，不允许跨相机、session 或 epoch 仅比较 raw/stable 整数。稳定关联不能改写 raw ID；可缺失稳定 ID，不能把“尚未匹配”伪造成有效 ID 0。SORT 现有原始数值与进程内防复用逻辑在 V1 路径保持。

V2 单个跟踪结果保留以下字段：

数值类型决定：像素框坐标、图像速度、世界位置/深度、预测位置/速度采用 float64；class_id 为 int32，置信度为 float32 并有独立有效性。V1 兼容输出显式执行原精度转换，新增存储精度不代表物理测量精度提升。

| 内容 | 决定 |
| --- | --- |
| bbox | `filtered_box` 与 `observed_box`，均为原图像素 xyxy；不沿用归一化字段名；有效性分别声明 |
| 观测状态 | `observed_this_frame`、`confirmed`、hits、age、missed_updates 明确传递 |
| 速度 | `image_velocity_x/y`、`image_velocity_valid`、`image_velocity_unit`；单位枚举 UNKNOWN、PIXELS_PER_UPDATE、PIXELS_PER_SECOND |
| SORT 速度 | 原 `pixels_per_frame` 精确映射到 PIXELS_PER_UPDATE，保持每次 update 的原数值，不按相机标称频率换算 |
| 类别/置信度 | class_id、class_name；置信度有值/未知独立标记，有效值为有限数且在 [0,1]；不伪造置信度 1 |
| 世界位置/速度 | 不放入像素跟踪字段；由独立世界目标和预测结果表达 |

V2 使用 `DetectionV2/DetectionArrayV2`、`ImageTrackV2/ImageTrackArrayV2`、`WorldTargetV2/WorldTargetArrayV2`。多目标与单目标保留各自计算接口，单目标 ROS 输出可以引用同一像素结果类型，但不冒充多目标 raw ID 空间。

数组上限由后端/平台预算共同限制；初始平台/UI 消费上限保持 100，API 现有 1024 输入保护不自动扩大平台输出上限。未来提高上限需单独验收消费者与资源预算。

### 2.3 世界目标、回流与消费者

`WorldTargetV2` 使用米制 position、显式 `position_valid`，完整保留 TrackKey 和原观测引用；世界系明确为平台接纳的 `odom`，其他 frame 必须经过显式变换后才能用于现有飞行消费。相机距离字段明确为光学 Z 深度 `optical_depth_m`，不把现有 Z 深度值标为欧氏距离。

V2 深度定位仅消费 `observed_this_frame=true` 的**原观测框**，输出 raw key。稳定 ID 如尚未确定保持无效；世界回流由跟踪节点按完整 raw key 接纳，之后输出的稳定映射不改变已发布 raw key。预测、跟随、搜索和分类不得将 `/tracks` 的稳定整数与旧 `/target_positions` 的原始整数直接等同。

V2 融合的深度配对须匹配相机绑定、原图尺寸与源时间窗口，拒绝错误布局/旧会话。窗口初值保持现有 0.2 秒，作为显式配置记录，不能因时间接近就推定几何已对齐。动态 TF 必须按观测时间查询，失败使结果无效；现有 V1 的 latest TF 回退保留为历史行为。有效性变化应作为独立行为修正验证，不归入纯字段迁移。

世界速度由独立预测器从同一完整 TrackKey 下的有序世界观测估计，单位 m/s；预测结果保留模型参考时间、真实观测时间、预测时域、有效性和非校准方差信息。V2 预测使用完整 key，不丢失成 int32。

消费端首次绑定合法生产者 session/epoch，发现其变化即清空相关历史、排空相关在途计算；旧写者迟到数据不能重新激活已退休 session。仅有 UUID 不能自动授权新来源：同时验证唯一发布者/固定端点、预期相机、上下游引用与时效。执行中的目标任务在来源变更时终止，不把旧整数自动绑定给新目标。

## 3. V1 → V2 兼容迁移

### 3.1 话题与部署决定

| 现有类型/话题 | V2 计划入口 | 兼容决定 |
| --- | --- | --- |
| DetectionInfoArray `/detections` | DetectionArrayV2 `/perception/v2/detections` | V1 字段与归一化语义保留；V2 从真实原图取得尺寸与上下文 |
| TrackInfoArray `/tracks_raw` | ImageTrackArrayV2 `/perception/v2/tracks_raw` | raw key；仅真实观测进入融合，预测可在显示通道表达 |
| TrackInfoArray `/tracks` | ImageTrackArrayV2 `/perception/v2/tracks` | raw/stable 身份同时显式保留；不依赖话题名猜 ID 类型 |
| Target3DArray `/target_positions` | WorldTargetArrayV2 `/perception/v2/world_targets` | V1 raw int32 不变；V2 携带完整 key、原观测和投影证据 |
| TargetPredictionArray `/target_predictions` | TargetPredictionV2/TargetPredictionArrayV2 `/perception/v2/target_predictions` | 保留原预测模型与单位，拒绝跨代次历史 |
| SingleTrackState 等单目标输出 | SingleTrackStateV2 与 ImageTrackV2 像素结果 | 保留单目标管理服务；该服务返回的旧整数不能代表跨 session 身份 |
| ManageSingleTrack.srv（响应内嵌 SingleTrackState） | ManageSingleTrackV2 独立服务入口 | 旧服务与响应不变；ui_control 的新服务客户端返回完整状态/key，不能在旧响应中塞入 V2 |
| VehicleIdentity/VehicleIdentityArray | VehicleIdentityV2/VehicleIdentityArrayV2 | 保留外观身份语义，同时保留完整来源 TrackKey，禁止只回传 int32 |
| GetPerception 服务 | 新 `GetPerceptionV2` 及 `/platform/perception/v2/query` | 旧服务响应类型不变；新服务返回带完整上下文的世界目标 |
| TrackTarget 等现有飞行 Action | 暂保留类型与入口 | 适配边界把旧 int32 解析到当前唯一 key；接受后固定绑定，来源变化终止；禁止猜测 raw/stable 对应 |

字段级映射：V1 Header 保留源观测时间；V1 归一化 bbox 通过真实尺寸转换；V1 `velocity_*` 保留原像素/更新语义且不得写入米制速度；V1 trajectory_* 不当作算法观测；V1 Target3D.distance 在迁移时确认为 optical Z；V1 `position_valid` 映射独立有效性；V1 raw/stable ID 在桥接时显式指定通道语义。

兼容发布由**一个指定来源**拥有，不允许旧节点和桥同时发布同一旧话题。V2→V1 如需 int32，使用有界的进程内别名表，不截断 int64。别名只用于兼容，不反写原始 key；兼容桥重启或来源变化必须令旧消费缓存与执行任务失效，无法证明失效时拒绝恢复控制用途。

不默认开启 V1→V2 在线桥用于控制：V1 缺少会话、尺寸、观测状态等证据，不能合成后宣称其等价于原生 V2。明确来源与固定配置的 V1 导入只用于隔离回放/诊断，标注 legacy evidence，不授予导航或跟随就绪。

### 3.2 消费者迁移范围与顺序

| 顺序 | 包/边界 | 必须完成的迁移 |
| --- | --- | --- |
| 1 | easyfly_msgs、tracking API/适配、object_detection、multi_object_tracking | V2 数据、上下文与 raw/stable 映射，保留 V1 接口 |
| 2 | pixel_to_3d、track_id_fusion、target_prediction | 观测框、完整 key 回流、深度/TF 证据与跨代次历史隔离 |
| 3 | target_tracking、mission_fsm/search_area、semantic_map/obstacle_classification、gimbal_control | 完整 key 接纳、任务绑定与来源失效；geometry 与分类有效性分开 |
| 4 | single_object_tracking、vehicle_reid | 单目标状态、真实观测框、同帧图像配对与 VehicleIdentity 来源 key；VehicleIdentity 亦需 V2 版本或等效不丢 key 的版本化输出 |
| 5 | perception_srvs、core/pybind、ui_control、ui_bridge、desktop | V2 查询、单目标管理响应、类型发现、缓存/显示与身份解析；SDK 的 Action 类型保持，但任务绑定语义更新 |
| 6 | connection_manager 的 onboard/remote 配置与启动模板 | 新话题/类型与配置版本；确保所有原生消费者使用同一代链路 |
| 7 | 相关测试、工具、录包导入说明 | 新旧链路分别验证，兼容输出明确标记 |

`easyfly_core` 当前没有直接解析 TrackInfo/Target3D 感知数组；不要因此增加无需求的 SDK 感知 API。SDK 间接受 GetPerception/目标 Action 等消费者迁移影响的部分按真实使用调整。

实现可以分批构建与测试，但不能让未隔离的新旧身份链一起参与控制。完成原生消费者迁移与真实回归后，受管方案才能整体切换；V1 发布仅用于显式兼容配置。V1 定义至少保留到下一个接口主版本，不按日期静默删除；删除需全部登记消费者和交付目标完成迁移并提供独立变更记录。

### 3.3 录包、回放与 ABI

- 旧 CDR 使用原 V1 类型解释，不用 V2 定义反序列化旧数据；不追写旧包。旧消息类型不修改，避免类型同名但二进制不兼容。
- 旧感知包可在隔离域用 V1 安装或明确诊断适配读取；缺会话/尺寸/观测证据的结果不可恢复为飞行输入。
- 当前独立录像/历史播放器只录放 Image 和 Odometry，暂不直接消费这批感知消息，不增加感知回放白名单。桌面画面录像与感知 CDR 是不同边界。
- 引入 V2 及改变 ui_bridge 公开快照后，重编译实际消费者；公开 C++ 布局改变时更新其 SOVERSION，禁止混载旧库。EGO 的 PositionCommand 本批不改。

## 4. 公共登记与配置 schema

运行规范文件：[登记 schema](../../easyfly_common/easyfly_backend_registry/easyfly_backend_registry/data/schemas/backend-registry-v1.schema.json)、[选择配置 schema](../../easyfly_common/easyfly_backend_registry/easyfly_backend_registry/data/schemas/backend-selection-v1.schema.json)。公共包安装这些 schema 并供真实入口使用；算法目录下的 schemas/examples 保留离线设计示例，不能作为完整运行登记。

### 4.1 包归属与依赖方向

`easyfly_backend_registry`：包含安装的 JSON 描述、schema、无 ROS 的读取/结构校验工具及 C++ 需要的静态元数据。Python 使用标准 JSON/必要 schema 校验；C++ 不嵌入 Python。若编译期生成 C++ 元数据，由构建步骤从同一份描述生成并验证，不维护第二份手写名单。

依赖方向：

```text
UI / easyfly_params / launch / 平台适配器 → backend_registry
平台适配器 → 能力 API、所选实现包
计算实现 → 能力 API、实际计算库
backend_registry → 描述与校验依赖（不依赖具体实现、SDK、ROS节点或UI）
```

登记里的 package/creator/launch 仅为可验证引用，公共包不链接或加载实现。具体 creator 位于对应功能适配包，以 capability+backend ID 查找；schema 中的 creator 字符串不能任意指定系统可执行路径。

### 4.2 配置结构与验证责任

选择配置分为 `backends`、`navigation`、`platform` 三部分：

- `backends.<role>`：调用角色对应的后端 ID 和专用强类型参数，参数由登记引用的专用 schema 严格校验，禁止未知字段。角色包括 detection、multi_tracking、single_tracking、global_grid、combined_grid、localization、trajectory_planning；global_grid/combined_grid 都消费 grid_planning 能力。
- `navigation.strategy`：astar、ego、combined、gcopter、super 的编排策略。`grid_role` 固定引用 global_grid，`combined_grid_role` 固定引用 combined_grid；初始两角色都显式选择 astar，参数独立，互不隐式继承。
- `platform`：节点输入/输出绑定、QoS、队列、时效、时钟；按能力的节点 schema 严格校验。飞行约束继续由原平台配置与执行准入拥有，后端参数不得覆盖。

读取时执行结构校验 → 专用参数校验 → 选择与登记引用校验 → 能力匹配 → 运行范围/组合规则校验。JSON schema 不负责确认文件存在、库可加载、输入新鲜或线程可取消；节点继续执行这些检查。

运行后端、参数及平台能力以公共包的登记、schema 和实际节点路由为准。尚未贯通的 platform 覆盖明确拒绝；新增节点、模型/SDK 资产与组合时同步登记并验证路由，不能因外层允许 object 就接纳任意参数。

有效配置优先级由低到高：后端 schema 默认值（须与实现默认值核对）→ profile 缺省值 → 显式配置文件 → 明确支持的 launch/CLI 覆盖。profile 仅补未设置值。旧 YAML 经一次性 legacy importer 映射到新结构；同一层的新字段与旧字段同时指定且冲突时拒绝，禁止靠遍历顺序决定。UI 提交的是同一结构，不额外拥有高于 CLI 的隐含优先级。

合并后只向节点传入一份不可变生效配置，记录每项覆盖来源与实际后端/provider；后端、模型、输入绑定等变更要求重启对应受管会话。未来运行中调参需单独定义适用字段与状态转换。

### 4.3 现有参数的明确映射

| 现有入口 | 新结构归属 | 默认值与决定 |
| --- | --- | --- |
| tracking.yaml 的 tracker_backend | backends.multi_tracking.backend | sort |
| iou_threshold/max_lost_frames/min_hits/max_tracks/dist_gate_scale | backends.multi_tracking.parameters | 0.15/45/3/100/1.5，保持正式 YAML；不覆盖为 C++ 裸默认 |
| tracking 的话题、尺寸和 consolidation 配置 | platform 的输入绑定/ID关联策略 | 尺寸由真实图像验证；ID关联不变成 SORT 专用参数 |
| global_planner 的 planner_backend | backends.global_grid.backend/parameters | astar；max_cells 与 unstick_cells 属 A* 配置，公共请求预算由节点拥有 |
| combined 内固定 make_planner | backends.combined_grid.backend/parameters | astar；对应 max_cells=1000000、unstick_cells=0 独立保留 |
| detection.backend/model/imgsz/阈值/provider请求 | backends.detection | onnx；640/0.35/0.45；原模型与类别顺序不变 |
| 单目标工厂创建 | backends.single_tracking.backend | 默认 csrt，可显式选择 kcf；参数按各自 schema 校验 |
| algorithm_ui 的 slam/provider/strategy | 定位后端选择、检测设备请求、navigation.strategy | profile/auto 哨兵在边界解析为具体值，不登记为真实后端 |

同一个算法在全局节点和 combined 中可以有不同参数；登记描述实现能力，选择配置保存各调用角色参数。各角色应生成独立实例，不把不同参数合并成全进程共享对象。

CUDA→CPU 是现有 ONNX provider 内部行为，保留并报告真实 provider；不得据此引入 SORT→其他跟踪器或任意规划器的静默回退。当前 TensorRT/OpenVINO 未实现，登记时不能声明可用。

## 5. 变更与兼容约束

- 变更消息或契约时逐项核对生产发布者、订阅者、回流、控制绑定及兼容输出，不只检查编译。
- schema 使用 JSON Schema Draft 2020-12；跨记录唯一性和引用存在性另做语义验证。未知字段、版本、非法类型及不支持的组合明确失败。
- API/ABI 改变时同步迁移并重建对应消费者，不混用旧头与新库。静态登记查询按内容快照隔离符号，避免不同安装版本在同一进程中相互覆盖。
- 消息可编译、登记存在或依赖文件齐全不等于算法运行、来源切换、任务停止或飞行验收通过。实现与实际结果按[接入指南](integration-guide.md)分别验证，未覆盖项保留明确范围。
