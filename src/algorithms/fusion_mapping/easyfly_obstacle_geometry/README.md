# easyfly_obstacle_geometry

EasyFly 自有 C++17 算法，导出 `easyfly_obstacle_geometry::easyfly_obstacle_geometry`。
依赖 STL / 系统 PCL / Eigen，不依赖 ROS，不放入 third_party。

- `clustering.hpp`：当前帧欧式聚类、动态观测 AABB、输出 AABB 及原尺寸/碎片/float32 门限。
- `dynamic_obstacle_tracker.hpp`：Hungarian 几何关联、歧义/尺寸/方向门限、EMA 速度和动态确认、有限休眠 ID、带不确定度增长的九点截面前向扫掠、完整截面预算分配。

输入点云为同一世界参考系中的完整当前非地面帧，单位 m；调用同步只读借用，不保存点云。
clusterCloud 要求非空指针、有限点和合法 PCL 聚类参数；索引必须属于输入点云。
extractCurrentDynamicObservations 额外接受空指针/不足最小点数并返回空观测。
两条 AABB 路径保留历史数值差异：动态观测先扩展到 double，输出路径先作 float 运算，
输出初值为 ±1e9；不要混用精度或合并两轮聚类。max_obstacle_size 和 .05m 碎片门限未变。
输出状态区分跳过过大/碎片簇与整帧非法几何/float 字段，最终失效策略由节点执行。
fields 仅 kAccepted 时可消费，bounds 顺序 xmin/ymin/zmin/xmax/ymax/zmax。

tracker 输入时间为显式源帧时间 s；调用者负责 ROS 时钟、输入年龄、TF 和车辆状态接纳。
update/predict 不读取时钟，不处理消息或启动线程。实例有状态，调用者负责串行化和 reset。
空帧不输出历史框/预测并清除速度确认，允许有界短期休眠 ID；时间回退 reset。
预测仅基于当前确认动态目标，按距离分配完整九点截面；无历史点云拼接。
节点继续负责全局点数预算、float 点云编码、消息/服务/地图版本和输入失效 reset。

旧 `easyfly_semantic_map/dynamic_obstacle_tracker.hpp` 提供源码兼容别名；不承诺二进制 ABI。
