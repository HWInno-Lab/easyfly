# 新增算法接入指南

当前结构和接口已落地；本指南中的 `candidate_mot`、`candidate_grid` 是接入示例名称，尚未实现或注册。
现有生产工厂只接受 `sort`、`astar`，未知名称会失败。仓库没有自动扫描third_party并加载后端的插件系统。
新增同类算法通常集中修改实现包装、适配器、工厂、依赖和配置；新增数据需求仍须修改相关数据链。

## 1. 先确定代码与职责的位置

| 内容 | 放置位置 | 职责 |
| --- | --- | --- |
| 项目维护的计算实现/普通包装 | algorithms/对应功能/easyfly_实现名 | 计算、资源限制和算法状态 |
| 原样导入的外部源码 | third_party/algorithms/主要功能/上游项目 | 保留上游布局、版权、依赖与本地补丁记录 |
| 能力类型和抽象接口 | 已有对应API包，确需新增能力时另行设计 | 不依赖具体实现或平台消息 |
| 节点、适配器与工厂 | 使用该能力的EasyFly功能包 | 消息转换、输入接纳、配置、实例创建、worker与发布 |
| 飞行执行与会话编排 | 既有执行/任务层 | Action所有权、取消排空、接管、失效及停流 |

自有实现指项目维护代码，不表示SORT/A*等理论为原创；导入代码不因目录位置而改用项目许可。
SDK、模型、外部工具分别归类，模型文件不会成为算法实现。分类目录按实际代码创建，不建空包占位。
FAST-LIO/EGO类完整ROS工程保留自己的节点、内部模块与消息，通过现有平台接线集成。
新库使用安装target/module，不读取相邻包源码目录。构建细节见[构建规范](build-conventions.md)。

## 2. 示例：新增多目标跟踪器 candidate_mot

参照现有[跟踪接口](tracking/easyfly_tracking_api/README.md)、
[SORT适配器](../perception/p3_tracking/easyfly_multi_object_tracking/src/adapters/sort_adapter.cpp)和
[工厂声明](../perception/p3_tracking/easyfly_multi_object_tracking/include/easyfly_multi_object_tracking/adapters/tracker_factory.hpp)。

1. 登记上游仓库、精确版本/未知项、输入归档或文件SHA、许可证和本地patch。
   外部原始源码放第三方目录；项目维护的普通封装可建立easyfly_candidate_mot包。
2. 明确输入、状态和输出，选择实现 `easyfly::tracking::Tracker`；
   保留 `update(const Frame&)`、`reset(epoch)`、`capabilities()`，不把ROS对象传入计算库。
3. 包装层做普通类型到vendor类型的转换。算法实例串行拥有，不能在返回后保留借用ImageView。
   先检查布局/需求/资源预算，再检查同epoch的时间顺序；失败不能留下部分轨迹或错误提交时间。
4. 在跟踪功能包增加适配/创建逻辑，声明实际算法包依赖并链接安装target。
   当前 `make_tracker(backend, SortParams)` 仍绑定SORT参数；新算法应有自己的参数结构和显式配置映射。
   需要时扩展工厂参数及调用处，不把候选算法参数伪装成SortParams。
5. 为新的名字增加明确分支，继续拒绝未知后端；保留默认sort。
   启动配置目前从tracking.yaml的 `tracker_backend` 选择，没有热切换或自动回退。
6. 实现并验证raw ID、原观测框/滤波框、确认/丢失状态、类别、置信度和速度单位映射。
   世界坐标稳定ID整合仍由独立模块处理；reset同时清理节点世界观测和整合历史。

假设候选跟踪器需要128维预计算特征和低置信度检测，可以这样声明需求：

```cpp
easyfly::tracking::Capabilities capabilities() const override
{
  return {"candidate_mot", {false, 128, true},
    easyfly::tracking::VelocityUnit::pixels_per_second};
}
```

这只是类型用法示例。必须在update中用实际Requirements调用validate_frame，缺特征/低置信度输入应返回
unsupported_input。声明不等于节点已经提供这些数据：现有跟踪节点只构造检测Frame，未填image、appearance
或低置信度输入声明。debug图像订阅不是已经同步的算法图像输入，不能直接复用最近一帧。

| 额外需求 | 必须补足的接入工作 |
| --- | --- |
| 原图 | 按源时间/相机/epoch匹配检测与图像，管理编码、stride、只读所有权、同步窗口和队列上限 |
| 外观特征 | 明确特征模型、维数/归一化和检测一一对应关系；声明推理依赖、预算及无特征失败行为 |
| 低置信度候选 | 检查检测器/适配过滤是否已丢弃候选；新增明确输出契约，不能仅修改Requirements布尔值 |
| px/s速度 | 消费者显式识别VelocityUnit，不能按原SORT的px/frame解释 |

ROS TrackInfo目前没有epoch且ID是int32。现有SortAdapter用进程内不重复raw ID保护reset后身份，
新适配器也要处理可表示范围和旧消息；不能仅依靠API中的epoch就安全复用已发布ID。
单目标ROI跟踪应使用easyfly_single_tracking_api，其initialize/update/reset契约与多目标接口不同。

## 3. 示例：新增二维栅格规划器 candidate_grid

参照[栅格接口](planning/easyfly_grid_planner_api/README.md)、
[A*适配器](../execution/e2_planning/easyfly_global_planner/src/adapters/astar_adapter.cpp)和
[工厂声明](../execution/e2_planning/easyfly_global_planner/include/easyfly_global_planner/adapters/planner_factory.hpp)。

1. 判断候选算法是否真的消费轴对齐二维占据栅格并返回XY路径；符合时实现 `GridPlanner`。
   项目实现/包装和vendor源码分别归类，参数、图结构及资源限制保留在实现包。
2. 实现 `plan(const Request&, const SearchControl&)` 并原样回传MapIdentity。
   访问网格前校验布局/溢出、存储长度和边界；明确未知格、阈值、端点和同格请求语义。
   GridView只借用到plan返回，不能保存原指针；缓存地图须自有存储并计入资源预算。
   增量规划状态须按epoch/revision检查和重建，当前GridPlanner没有通用reset方法。
3. capabilities只声明实际支持的控制。请求不支持的取消/deadline/展开预算时返回unsupported_request。
   在输入扫描、搜索和剪枝的有界检查点执行控制；API辅助函数不会自动中断vendor搜索线程。
4. 结果区分无路、输入错误、取消、超时和资源耗尽，失败不带可执行部分路径。
   成功保留精确起终点。平台继续检查地图新鲜度、参考系、端点准入及飞行边界。
5. 在global_planner增加适配器和工厂分支、CMake/package.xml依赖及候选算法专用参数。
   当前 `make_planner(backend, AStarParams)` 绑定A*参数，同样需要显式扩展配置映射。
6. 全局节点的启动参数名是 `planner_backend`；默认astar。
   combined协调器目前单独调用 `make_planner("astar", {1000000, 0})`，不会自动读取该参数。
   若要替换combined的路径规划，须单独修改其选择/参数，并复验该模式。

高度、waypoint切换、Action会话、取消排空和交接不进入GridPlanner。
如果候选算法需要三维体素、ESDF、动力学状态或输出带时间的轨迹，应先设计对应能力/请求类型或ROS接入协议。
现有GridView不能表达这些数据，不能把3D信息截成2D就宣称可替换EGO；本仓库尚无通用三维轨迹API。
EGO的现有完整ROS工程继续通过easyfly_local_planner及导航会话协议接入。

## 4. 构建、配置与验收交付

原有能力API可直接通过安装target消费，例如：

```cmake
find_package(easyfly_tracking_api REQUIRED)
target_link_libraries(my_tracker PRIVATE easyfly_tracking_api::easyfly_tracking_api)
# 二维规划同理：easyfly_grid_planner_api::easyfly_grid_planner_api
```

实现自己的公开头暴露API或数值依赖时用PUBLIC并导出依赖；私有SDK留在实现中。
Python普通算法导出已安装模块。接口版本尚不承诺稳定ABI，改变接口须同步消费者。
新增算法应提供以下实际证据，不能只提交接口声明或示例替身：

- 原始算法数值/关联或搜索测试，非法/空输入、资源限制、reset/身份与能力不满足检查。
- 真实适配器与节点回归：坐标、时间、速度单位、消息精度、回流/worker及故障语义。
- 接入新依赖时，两种安装及普通复制安装搬迁后的真实消费者；核对编译头和实际加载库。
- 新输入/模型/SDK的获取与版本证据、许可证、默认配置和明确的能力就绪条件。
- 行为改变单独登记/提交，原算法差分和迁移不与新优化混为一项通过结果。

可复用现有tools/algorithms验证脚手架并按新实现补测试；ALG-1中的替身不代表生产后端。
完整ROS引擎保留ROS消费方式，不要求伪装成无ROS核心。SDK目录存在或provider文件齐全也不等于算法就绪。
源、依赖、配置或二进制不可用时明确失败，保留既有ONNX实际provider上报/CPU回退语义。
来源和patch登记方式见[第三方来源规范](../third_party/sourcing-and-patches.md)，
兼容入口保留原因见[兼容层审计](compatibility-audit.md)。
