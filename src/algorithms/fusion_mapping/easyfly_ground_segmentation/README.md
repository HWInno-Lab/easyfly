# easyfly_ground_segmentation

ALG-7.1 的自有 C++17 计算包，target `easyfly_ground_segmentation::easyfly_ground_segmentation`。
依赖系统 Eigen3/PCL；实际 RANSAC 由 PCL 实现，本包维护原拟合策略、空间证据与可信模型。
不依赖 ROS 消息、TF 查询、设备 SDK、地图发布或飞行执行；公开几何类型不含平台对象。

`fitGroundCandidate(cloud,vehicle,params)` 同步只读借用完整地面参考系 PCL XYZI 点云与显式车辆位姿。
调用者保持输入存活且不并发修改；函数只复制候选拟合所需的子集，不修改/裁剪原点云。
不足100点保持原无候选行为。保留 below-vehicle 筛选、原完整帧置信度分母、0.12m/200次 RANSAC，
以及显式空间策略下 0.02m/300次独立拟合、0.025m残差和环带网格证据。
FitParams 的默认空间支撑为 false，节点继续显式传入正式 YAML 的 true；所有原阈值不变。
FitResult.has_candidate 仅表示有可供 tracker 评估的候选，不表示地面可信或地图就绪。
inlier_count/coefficient_count 保留原诊断意义，不能用候选内点直接删除障碍。

`GroundPlaneTracker` 串行拥有模型历史，update 显式接收候选、位置与 connected/pose_valid/armed/state_age_s 证据。
它只消费数值证据，不采样时间或查询车辆状态。保留连续初始化、净空/倾角/创新门限、飞行中冻结、
空中重启拒绝和 reset 语义。完整帧低比例候选只有显式空间策略且五帧连续支持才能初始化。
空间支撑保留1–3m环带、0.2m独立格、全部八方向、至少200点/120格/90%/每方向5格，以及低返回格否决。

GroundPlaneModel、groundSpatialSupport 与等式变换保持原计算；平面长度单位m，角度rad。
groundPlaneInWorld 接收明确 world_to_ground 刚体变换，只运输同一物理平面。
groundBasisContinuous/groundBasisStampFresh 接收显式变换/时间证据；不把不同时间域相减。
模型只在调用方确认参考系/epoch有效时使用；状态/TF失效、地图失效与输出生命周期由功能包管理。

节点继续拥有ROS状态和steady接收年龄、TF/时效、参考系跳变后的reset、当前帧管理、低障碍过滤、占据图和服务。
旧四个头只转发并保留命名空间别名，支持重新编译的旧包含路径，不承诺旧二进制ABI。
原点云与原阈值语义的前提沿用，不在本批改写任意非法参数、极端坐标或退化输入支持。
