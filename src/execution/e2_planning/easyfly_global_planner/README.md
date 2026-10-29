# 全局规划接入（ALG-3）

A*、脱困与视线剪枝位于 `easyfly_astar`；本包通过 `GridPlanner` 适配器/工厂调用。
节点继续负责 ROS 地图与状态接纳、目标高度、航点 Action、取消排空和导航执行。
导航协调器与局部规划接入保持原职责。旧 astar.hpp 仅包含新头与兼容别名。

只读启动 ROS 参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| planner_backend | astar | 当前唯一实现；未知值启动失败 |
| max_grid_cells | 4000000 | 输入地图格数上限 |
| planning_max_expansions | 4000000 | 每次 A* 展开上限，0 禁止展开 |
| planning_timeout | 5.0 | 搜索/输入转换的 steady-clock 秒预算，范围 (0,300] |

适配器校验布局和有限坐标，显式转换未知格/占据阈值策略，回传地图身份和展开数。
默认阈值 50、未知可通行、5 格脱困及原路径输出保持。API 上边界采用半开区间，
非法字节 fail-closed；失败没有可执行部分路径。取消、超时和展开耗尽分别返回。
公开适配器 target 为 `easyfly_global_planner::easyfly_global_planner_backends`，
可由无 ROS overlay 的普通 CMake 工程消费；节点运行仍需要 ROS。

无效地图消息会中断正在进行的搜索；地图 reset/失效与时效超限会在段发送、
等待结果前重新接纳，现有子航点必须清理到终态才能释放 parent。
一次失效事件在当前导航内保持有效，不因之后收到有效地图而恢复旧路线。
持续收到有效地图不自动触发重规划；这仍是原有静态规划模式。

回归使用合成地图/遥测与显式 waypoint mock；既有导航链还使用真实协调器、EGO、
轨迹服务器和飞控软件节点。测试不启动 PX4/Gazebo，不代表实际飞行认证。


GCOPTER 的显式滚动导航通过 `navigate.launch.py strategy:=gcopter
 gcopter_dynamic_navigation:=true` 开启，需配套开启 GCOPTER 地图与所有者的动态模式。
父目标保持到全局到达；每段实际终态与精确 STOP 确认后才接续，实际无进展、观测不足、
300 s 总墙钟／8 次重规划／20 次会话预算耗尽均失败退出。默认固定地图行为不变。
参数、观测清障和地图坐标约束见相邻 `easyfly_trajectory_optimizer/README.md`。
