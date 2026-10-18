# 可信地面与室内空间支撑

地面候选在未解锁、连接正常、位姿有效且状态新鲜时初始化。已解锁时保持已有模型，
空中重启不能初始化。分类始终针对完整当前帧和可信模型，不直接删除 RANSAC 候选内点。

原有全帧规则保持：至少 50 个内点，内点占完整帧至少 15%，倾角、净空、状态新鲜度和
连续帧门槛均保留。默认 C++ 策略仍仅使用这条规则。

## 显式空间支撑策略

运行配置 `ground_spatial_support_enabled: true` 增加一条独立质量判据，供仓库等
室内场景使用。它改变地面候选的准入方式；不是将全帧比例调小。项目 YAML 显式启用，
删除此项或设置 false 恢复原规则。场景清单不会覆写它或其他安全参数。

全帧比例不足时，以 0.02 m 残差重新拟合水平候选，再从完整点云检查：

- 机体周围水平半径 1–3 m 的环带，按 0.2 m 独立 XY 网格计数。
- 至少 200 个支持点、120 个支持网格；八个方向各至少 5 个网格。
- 地面支持残差不超过 0.025 m；同一网格内任意低处异常返回均使该网格不合格。
- 用候选平面坐标判断机体以下的证据；低于平面的异常点计入分母。
  至少 90% 的相关独立网格合格。高处墙面和屋顶不会增加地面证据。
- 候选相对机体净空为 0.03–0.25 m，倾角最多 0.10 rad，至少五帧连续一致。
  原有状态新鲜度、连接、位姿和未解锁限制继续适用。

上方返回仍留在原障碍点云中，障碍膨胀、地图有效性、超时、低障碍分类阈值均未降低。
一侧地面、小桌面、重复点、地面台阶、稀疏支持、机体同高平面和非有限值不能通过
这条策略。它沿用“未解锁允许地面初始化”的系统假设，不能替代着地/接触传感器；
大面积平台仍可能成为局部地面。它也不是跨运行世界坐标标定。

`ground_initialized frame=... method=spatial_support` 日志记录实际采用的方法、全帧内点数、
独立网格数、支持点数与净空。`test_ground_spatial_support` 覆盖对应拒绝路径，
实际仓库运行证据见工作区 `schedule-v5-scene.md`。

## 地面模型参考系

空的 `ground_model_frame` 沿用 world_frame，原默认入口继续在 odom 保存地面。
仓库场景在 `navigation.ground_model_frame: map` 中显式声明 SLAM 参考系；
受管入口从清单传递，禁止再用同名 raw 参数覆写。输出点云与规划坐标仍是 odom。

可信平面在 map 中保存，按新鲜 map←odom TF 变换平面方程后分类当前 odom 点云。
飞行中不拟合或融合新地面；这只是同一物理平面的坐标变换，避免 odom 估计器变化
将地面变成障碍。缺失/过期的 TF 使地图无效；相邻参考变换平移超过 0.25m 或转角
超过 0.10rad 时重置地面模型并清图，已解锁时不能重新初始化。不会用过期变换替代。
`test_ground_plane_transform` 验证变换后的地面、低障碍、已解锁冻结及拒绝路径。

## Explicit occupancy footprints for indoor environments

`grid_footprint_mode` defaults to `cluster_boxes`, preserving existing maps and
compact-obstacle behavior. A managed scene may explicitly select `observed_cells`.
That mode projects every trusted nonground voxel, including samples outside usable
semantic clusters, with the existing inflation radius plus half the voxel width.
A connected U-shaped wall therefore blocks its observed walls without filling its
room interior. Semantic obstacle messages and markers keep their cluster labels.
Confirmed dynamic boxes and complete predicted footprints remain filled envelopes.
The 20-frame occupancy persistence and all ground/TF/freshness invalidation gates
are shared with the default mode; this does not turn unobserved space into a
certified free-space map. The projection is bounded to 200000 points and 1000000
grid cells and uses O(points + cells) work. Invalid mode selections fail startup.
Scene selection rejects CLI overrides and records the actual ROS parameter and
loaded semantic library before a mission.

An opt-in `grid_height_limit_m` (default `0.0`, unlimited) restricts only the
`observed_cells` 2D projection to heights above the current trusted ground plane.
Voxels intersecting the limit remain occupied, including half-width and ground
slope margins. Low obstacles, horizontal inflation, persistence and full dynamic
envelopes remain unchanged. Full nonground clouds, semantic messages and the
local 3D planner retain overhead surfaces. Invalid ground/current-cloud evidence
or vehicle height above `limit - 0.4m` invalidates the grid. Managed scenes must
declare a measured recommended region with `max.z + 0.6m <= limit`; arbitrary
mission heights outside that region require independent qualification.

## ALG-7.1 算法归属

候选筛选/PCL RANSAC、空间支撑、可信平面状态及坐标变换数学迁入
`easyfly_ground_segmentation`，原四个头仅转发别名；完整当前帧和车辆/TF证据由本包显式传入。
节点继续接纳车辆状态、steady接收年龄、TF时效/参考系跳变，管理地图失效、低障碍分类、栅格和服务。
候选可用不代表地面可信，分类仍使用可信平面；原阈值和正式 YAML 的空间支撑选择保持。
