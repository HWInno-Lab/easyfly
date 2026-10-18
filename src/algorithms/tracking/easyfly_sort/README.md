# easyfly_sort

从 EasyFly 跟踪功能包迁出的自有 SORT 实现，保留原卡尔曼、IoU/中心距离门限、
匈牙利代价和生命周期计算。无 ROS/OpenCV/PCL/推理运行库依赖。

头文件 `easyfly_sort/sort.hpp`，命名空间 `easyfly::sort`；通过
`easyfly_sort::easyfly_sort` CMake INTERFACE target 消费。
默认参数与原 C++ 一致，正式节点仍加载原 YAML 中的 .15/45 等配置。
原算法的 6 条回归用例随库迁移。

SORT 理论来自 Bewley 等人的工作，本目录是项目维护实现，不宣称理论原创。
原始每次 update 仍是一个帧步长，速度 px/frame。原始 Sorter 不负责 ROS 时间或 epoch；
平台 SortAdapter 实现统一 Tracker、输入/顺序检查和不复用 ID 的 reset。
旧功能包保留同名 include 的 using 别名，不保留第二份实现。
