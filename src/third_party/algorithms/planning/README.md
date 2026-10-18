# 外部规划算法

`ego_planner/` 保留完整8包 ROS2 工程：ego_planner（plan_manage）、plan_env、path_searching、
bspline_opt、traj_utils、traj_utils_so、quadrotor_msgs、ego_to_mavros。
EasyFly 通过 easyfly_local_planner 和现有导航会话协议接入；不将内部模块复制为自有算法。

来源、逐文件字节身份及未知项见 [ego_planner.provenance.json](ego_planner.provenance.json)。
本地导入点为7455e06，原上游仓库/fork/branch/commit未确认，未联网重取或升级。
多数package.xml仍为TODO许可，ego_to_mavros为MIT、quadrotor_msgs为BSD元数据；
未发现统一根LICENSE，现有文件通知原样保留，不据部分元数据推断整个工程许可。

CMake通过包导出依赖消费plan_env/path_searching/traj_utils_so，不再猜工作区根、
拼src/planner或读取相邻包源码include。traj_utils_so补导出namespaced target，
bspline_opt通过导出target引用依赖，以支持安装根搬迁。
安装消费者需要ROS/PCL/Eigen/OpenCV；EGO是完整ROS工程，不是无ROS纯库。

本地managed会话/固定终点、严格速度限制、碰撞和失效/停流、退出补丁均保留。
现有测试通过安装前缀选择节点；信号测试由安装share读取正式适配参数。
未为了迁移放宽时间/速度/安全边界，不以目录移动认定真实飞行已通过。

本机可能存在被Git忽略的历史planners/ego_planner软链接及父目录COLCON_IGNORE，
用于已经运行的旧安装/配置链接。它不提交、不参与新包发现，也不是第二份源码。
新检出应从分类目录构建；旧主安装/构建缓存未被隔离验证覆盖，旧会话结束并切换后才清理桥接。
复验与范围见 [ALG-8.2](../../../algorithms/alg8-2-validation.md)。
