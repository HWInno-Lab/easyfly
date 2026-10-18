# easyfly_grid_planner_api

ROS 运行时无关的 C++17 二维栅格路径规划接口。ALG-1 提供普通类型、O(1) 布局校验、
控制能力检查和确定性停止原因辅助；尚未迁移 A*、修改搜索循环或接入 EGO。

```cmake
find_package(easyfly_grid_planner_api REQUIRED)
target_link_libraries(my_planner PRIVATE easyfly_grid_planner_api::easyfly_grid_planner_api)
```

公共头：`easyfly_grid_planner_api/planner.hpp`，命名空间 `easyfly::grid_planning`。
安装 target 仅传递 C++17 与安装头路径，无 ROS 消息、节点或第三方数值库依赖。

- GridView 是同步借用的轴对齐 row-major int8 网格，长度为 width*height；分辨率/原点/路径
  均使用 m。调用方持有存储，plan 返回后不得保留其指针；并发修改地图须由调用方隔离。
- -1 为未知，0..100 为占据值，默认阈值 50、未知按可通行解释，与当前 A* 核心一致。
  必须显式保留策略；平台地图有效性检查仍先于算法准入。非法格值在 occupied_value 中按占据。
- validate_layout 只检查布局和有限几何，不扫描或认证整幅地图，也不作 freshness 或端点
  接纳。max_cells 显式由调用方提供。后端访问前检查 bounds，并在计算时保留预算检查点。
- Request/PathResult 带 map 的 epoch、revision 和源时间；结果必须原样回传身份，供上层
  拒绝过期地图结果。API 不查询时钟或 TF，也不定义飞行坐标系。
- SearchControl 包含借用的 atomic 取消标志、steady deadline、展开次数限制。nullopt 表示
  不请求该限制，engaged 的 0 表示不允许展开；不支持所请求控制须返回 unsupported_request。
- check_control 使用调用方显式传入的 steady now，按取消→deadline→展开预算顺序判定。
  后端必须在输入扫描、搜索与剪枝的有界检查点调用；辅助函数不提供线程强停或硬实时保证。
- 失败不携带可执行的部分路径；成功路径保留真实起点和精确目标，同格请求也保留两端点。
  路径是 XY 几何，不是三维轨迹或执行许可。高度、子航点、Action 所有权仍由平台管理。

ALG-3 将独立验证 A* 对该接口的适配和预算/取消增强，当前 API 测试通过不能代替那项验收。
0.1 接口尚无稳定二进制 ABI 承诺，消费包需随接口变更重新编译。
