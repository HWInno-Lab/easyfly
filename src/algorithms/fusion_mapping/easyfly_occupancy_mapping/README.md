# easyfly_occupancy_mapping

EasyFly 自有 C++17 header-only 算法库，导出 INTERFACE target
`easyfly_occupancy_mapping::easyfly_occupancy_mapping`，公开实现仅依赖 STL。

- grid_inflation.hpp：原0.70m默认/最小膨胀与配置归一化。
- observed_grid.hpp：所有当前非地面点的方形足迹差分投影、边界裁剪、可信地面以上高度/体素交集筛选。
- occupancy_grid.hpp：普通 GridGeometry、cluster-boxes膨胀投影、完整预测截面转包围盒、占据持久化。

所有几何单位为m，GridGeometry分辨率为m/cell，二维数据行优先、0空闲/100占据。
调用同步只读借用点/框，写入调用方普通vector，不持有点云、不启动线程或读取时钟。
GridPersistence有状态；调用方串行化apply/reset并决定何时失效。原规则保留：
出现占据立即置100；连续漏观测不超过persist_frames仍为100，超过才释放，尺寸变化/reset清空。
不改变输入未知值的历史处理；平台发布失效未知图之前必须reset历史。

observed投影保持原输入上限和数值拒绝，失败时调用方不能发布该结果。
有界层接收结构化地面对象指针，要求normal[3]、valid()及zAt(x,y)，直接复用原地面类型；
无需链接该类型的实现包。无地面/显式nullptr只支持无界层；有界层返回false。
NoGroundPlane为无地面重载的内部占位，不是地面估计器。
原真实GroundPlaneModel回归仅测试依赖easyfly_ground_segmentation；导出的target没有该依赖或PCL。

GridGeometry构造和cluster-boxes路径保留原数值前提：尺寸/分辨率正、有限且可表示，
格数与索引可表示，框有限、坐标到int的转换有界，输出长度为width*height。
预测尾段点数不超过容器长度，必须是非零footprint大小的整倍数；节点已按完整截面接纳。
预测转框保留每段首点Z，不改变高度规则；点容器只需size()和operator[]。
本批保持原算法契约，未为这些调用增加新配置守卫或阈值。

ROS帧接纳、地面/TF/车辆时效、高度能力接纳、地图有效性/版本、失效未知消息、
QoS/服务和发布都留在semantic_map；旧grid_inflation/observed_grid头只转发/using。
旧observed_grid头保留原GroundPlaneModel传递包含，提供源码兼容，不承诺旧二进制ABI。
