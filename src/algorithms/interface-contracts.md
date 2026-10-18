# 算法接口与行为冻结（ALG-0 / ALG-1）

基线：`bdbc34c`。以下现有算法行为在 ALG-0 冻结，ALG-1 已建立跟踪与二维规划 API，
ALG-2 已接入真实 SORT 与 ID 整合；二维规划适配仍待 ALG-3。
本文件记录不变的计算语义和各批次显式新增的接口检查。
实际头文件与细节见各 API 包 README；以下单位和职责边界继续作为迁移约束。

## 1. 通用约定

- 长度 m、角度 rad；图像 bbox 为原图像素 xyxy，消息端归一化由适配器负责。
- 时间戳用显式整数纳秒，ROS/source 与 steady/receipt 是不同时间域，不交叉相减。
- 输入包括有效布局、有限数值、维度和资源预算；算法仍检查自身必需的数值/状态条件。
- 输出区分有效、无结果、输入不合法、超时、取消、运行失败，不将失败转换为成功空列表。
- 输入内存只读，视图声明 stride 和所有权。worker 持有消息/拷贝直到算法调用完成。
- 实例单线程拥有。reset/重建必须使历史与输入 epoch 一起失效；线程安全不作默认承诺。
- 配置在启动时选择具体后端。未知后端或依赖缺失明确失败；不新增自动后端切换。
- 保留现有算法内部明确提供的 provider 行为：ONNX CUDA 初始化失败会回退 CPU，
  当前真实 provider 必须上报；这与静默切换 SORT/其他跟踪器是不同机制。
- 公共数据类型不含 ROS 消息，API 不依赖任何具体实现，不创建跨所有算法的总基类。

## 2. 多目标跟踪（ALG-1 / ALG-2）

ALG-1 接口：`Tracker::update(const Frame&) -> UpdateResult`、`reset(epoch)`、后端能力描述。
Frame 携带检测框、类别、置信度、图像尺寸、源时间/epoch 和可选图像/特征；普通类型表达。
UpdateResult 保留滤波框、最近观测框、原始 ID、命中数、年龄、丢失帧数、确认状态和速度单位。
对外稳定 ID 由独立 ID 整合器处理，不能覆盖 SORT 原始 ID。

当前真实行为：

- `Sorter::update()` 每次调用预测一个帧步长，8 维状态 `[cx,cy,w,h,vcx,vcy,vw,vh]`。
  当前速度是 px/frame，迁移时不改为 px/s。
- 候选满足 IoU 阈值或中心距离门限之一才可关联，代价仍为 `1-IoU`；已知不同类别拒绝。
- 新 ID 从 0 单调递增；命中累计达到 min_hits 确认；丢失帧数严格大于 max_lost_frames 删除。
- 所有存活轨迹均从核心输出，包含未确认和纯预测状态；节点再决定消息表示。
- C++ 默认 IoU=.3、max_lost=30，但正式 YAML 为 .15、45；min_hits=3、max_tracks=100、
  dist_gate_scale=1.5。接口默认值不能覆盖现有 YAML。
- ID 整合以原始 SORT ID 查世界观测；`/tracks_raw → /target_positions` 回流供下一帧整合，
  对外 `/tracks` 使用稳定 ID。半径 3m、TTL 10s，墓园上限 128，reset 清空整合历史。
- 原生 Sorter 没有独立 reset 方法。ALG-2 适配器重建实例并保留进程内 raw ID 偏移，
  同时清除整合器和缓存，不使旧世界观测绑定重置后的新轨迹。

ALG-2 已实现 reset/epoch、SORT 工厂和错误结果，并实际验证
检测→跟踪→三维定位→ID 回流。首次延迟世界观测的继承是独立记录的原有问题修复，
不能把这部分新旧差异归为纯目录移动；详见 ALG-2 验收。

## 3. 二维栅格路径规划（ALG-1 / ALG-3）

ALG-1 接口：`GridPlanner::plan(const Request&, const SearchControl&) -> PathResult`。
Request 包含网格、起终点和地图 epoch/revision/源时间，结果回传地图身份。
GridView 明确 width/height、row-major 数据、分辨率、原点、占据阈值和未知格策略。
SearchControl 含计算预算和轻量取消检查；结果返回世界 XY 航点及明确失败状态。
高度由平台接纳并在执行时使用，不把二维路径伪装成可执行三维轨迹。

当前真实行为：

- 8 邻域，直线代价 1、对角约 √2，欧氏启发；占据值 >=50 阻塞，越界格阻塞。
- 核心对 `-1` 未知格不判占据；节点另有地图有效性接纳。不得借接口重构改写此行为。
- 对角移动只有在两个相邻直向格都占据时才禁止，不是“任一侧占据即禁止”。
- 默认起点脱困半径 5 格；目标在障碍内失败。边界和输入异常以当前回归及源码为依据。
- 输出保留真实起点和精确终点，同格请求也有两端点；LOS 剪枝在最终端点加入后执行。
- 当前核心只有同步 `planPath()`，无预算/取消参数。节点有任务 deadline/取消管理，
  不等于 A* 搜索循环已支持中断。
- 正式配置 grid_stale_timeout=5s、segment_acceptance=.1m、z=.5..2.5m；协调器有独立
  input_freshness 和取消超时。地图接纳、真实端点与子任务排空保留在平台。

待实现/覆盖：预算/取消应在 ALG-3 独立小提交中补成有界检查点，并验证超时和停止语义；
先做等价迁移，再启用该增强，不将未来能力记成当前行为。

## 4. 单目标、检测和标志位姿（ALG-5）

单目标接口与多目标接口分开：图像和显式 ROI 初始化、update、停止/reset；保留 CSRT
初始化/重定位成功后才报告 tracking，未知置信度不伪造为 1。`single_node.cpp` 当前直接
拥有 `cv::TrackerCSRT`，仅移动 `track_core.hpp` 的 ROI 函数并不能完成算法抽离。

检测接口接收只读图像视图，返回原图像素框及类别、置信度和实际 provider。
现有 DetectorParams 混有话题、QoS 和节点性能字段，需要拆分算法配置和节点配置。
`DetectionTiming` 算法只填写预处理/推理/后处理部分，不清空调用者填写的队列和转换耗时。
默认模型 yolo11n.onnx、输入 640、conf=.35、NMS=.45；TensorRT/OpenVINO 仍是预留。

标志位姿包装保留 OpenCV 36h11、3 像素 SUBPIX、IPPE-square 候选、正深度、重投影和
歧义拒绝。算法返回相机光学系位姿及原因，CameraInfo 接纳、源时间和结果过期由节点管理。
标签尺寸默认 0 表示未配置，不能用接口默认值悄悄变为固定尺寸。

现有检测 contract 测试主要检查源码结构/观测字段，不代表本轮重新验证了实际模型精度。
仓库 m2_frame.bin/m2_baseline.json 可作后续实推理对照输入，其历史 CUDA 耗时不是本轮结果。

## 5. 预测、预处理和融合建图（ALG-4 / ALG-6 / ALG-7）

- 预测保留两个严格有序真实观测、三维范数限速、跳变/类别变化重置、空帧清历史、
  source/receipt 双时钟与暂停/回退失效。位置按 `observation + velocity*(age+horizon)`；
  方差是简化增长模型，不能宣称校准协方差。算法使用显式 ros_now/steady_now，不读时钟。
- 点云过滤保留球域范围、每体素首点及 intensity；unordered_map 的输出迭代顺序不作为
  稳定保证，等价比较按体素/点内容进行，不凭“first-wins”推断全局排序确定。
- 像素定位取框内有效中位深度、针孔反投影，深度单位 m；正式 YAML max_depth=19.5m，
  节点结构默认值不同，迁移按实际配置加载链保持。
- 雷达相机投影保留原 XYZ/header、最近像素 Z-buffer，稀疏光学 Z 为 32FC1 m，未支持处 NaN。
  两源时间 TF 经固定 odom，不回退 latest。ROS 字节布局解析留在适配层。
- 类别关联纯核只返回双向唯一匹配及歧义；节点复制原几何、ID、速度和 header，独立
  写类别/TTL。几何无效与关联无效分别保留，不依据类别删除障碍。
- 地面可信状态需要显式未解锁、连接、位姿、时效证据；飞行中冻结，空中重启不能初始化。
  现有空间支撑策略、map/odom 平面变换、跳变和低障碍规则整体保留。
- 障碍几何保留当前帧聚类、独立动态关联和完整预测足迹；不恢复历史点云混合。
- 占据图保留 cluster_boxes/observed_cells、膨胀半径至少 .7m、20 帧持久化、未知格重置。
  `map_validity.hpp` 依赖 ROS 消息并定义平台失效协议，留在功能包，不能整头迁入纯库。

## 6. 接口外的平台职责

保留的公开能力包括现有话题/QoS、服务/Action 名称、CameraInfo 模型、坐标系约定、
源时间/接收时间和 TTL、飞行所有权、取消排空、接管与故障锁。以下文件/模块明确留在平台：

- calibration/camera_contract、time_sync/pairing、coord_utils、各感知包 safety.hpp。
- global_planner 的 map_validity、goal_validation、segment_acceptance、waypoint_lifecycle。
- semantic_map 的 ROS 失效消息、TF 和状态接纳、服务与能力报告。
- mission_fsm 的任务调度/格式；flight_control 和 precision_landing 的执行策略与所有权。
- target_tracking 的目标选择与航点生命周期；gimbal 的设备/仿真状态和任务管理。

搜索路线的几何范围验证仍可在算法中保留；最终飞行边界接纳由调用方再次检查。
接口边界按职责而非“文件是否为纯 C++”判断。具体 QoS、配置和消息定义以冻结指纹对应
的源码为准，后续变更须更新基线并说明原因。
