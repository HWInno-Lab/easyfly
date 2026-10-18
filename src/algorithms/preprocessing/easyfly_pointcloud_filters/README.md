# easyfly_pointcloud_filters

项目维护的 C++17 距离过滤与体素 first-wins 降采样，迁自 easyfly_pointcloud_filter。
仅依赖标准库，导出 `easyfly_pointcloud_filters::easyfly_pointcloud_filters`。
公开头 `easyfly_pointcloud_filters/filter_core.hpp`，命名空间 `easyfly::pointcloud_filters`。

保留 Pt{x,y,z,intensity}、rangeFilter、voxelDownsample、filterCloud 的原签名与计算：
球域距离边界包含端点，先距离过滤再降采样；同一体素保留输入中的首点及其全部属性。
voxel_size <= 0 原样返回。原网格键打包方式与 unordered_map 保留；输出迭代顺序不承诺
跨编译器/标准库稳定，比较应按完整点内容而非全局排序。

本批仅等价迁移，不扩大原数值输入域：直接降采样需有限 XYZ、有限体素大小，
启用体素时其倒数和 floor(坐标/体素) 必须可表示，整型索引/原打包键的限制沿用旧实现。
未新增 NaN/极端配置的清洗或键算法修复，调用方负责适用的输入与配置。

ROS 节点继续负责 Livox 解析、源时间 TF、最多十帧等待队列、双时钟过期与输出封装。
过滤先在源坐标系执行，再按测量时刻 TF 变换；不会在核心查询 TF 或改用 latest。
旧 easyfly_pointcloud_filter/filter_core.hpp 仅含新头与 using 别名。
