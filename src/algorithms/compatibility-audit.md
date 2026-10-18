# 算法迁移兼容层审计

ALG-8.5只核对并登记现状，没有删除公开头、修改接口或运行中的本地路径桥接。
计算实现已迁入独立算法包，旧功能包的公开include仍服务原消费者和已安装头路径。
逐文件身份见[compatibility-audit.json](compatibility-audit.json)；本批所有这些头均与迁移前提交逐字节相同。

| 原消费者/入口 | 实际计算归属 | 保留内容 |
| --- | --- | --- |
| multi_object_tracking的sort/kalman/hungarian/id_consolidator头 | easyfly_sort、easyfly_track_id_fusion | 类型/函数/常量别名，无第二份SORT或关联实现 |
| global_planner/astar.hpp | easyfly_astar | 类型、函数和detail命名空间别名 |
| mission_fsm/search_geometry.hpp | easyfly_coverage_planning | 旧命名空间入口 |
| target_prediction/predictor.hpp | easyfly_motion_prediction | 类型/函数别名 |
| pointcloud_filter/filter_core.hpp | easyfly_pointcloud_filters | 转发include及别名 |
| object_detection、apriltag_detection、single_object_tracking旧计算头 | easyfly_yolo_onnx、easyfly_tag_pose、easyfly_csrt | 计算包装转发；原节点继续处理worker、图像与消息 |
| pixel_to_3d节点直接include新核心 | easyfly_projection | 没有旧投影计算转发头；frame_contract.hpp是保留的平台接纳 |
| semantic_map地面相关头 | easyfly_ground_segmentation | PCL/模型接口转发，平台状态/TF/车辆证据仍在节点 |
| semantic_map/dynamic_obstacle_tracker.hpp | easyfly_obstacle_geometry | 原公开类型/函数别名 |
| semantic_map/grid_inflation.hpp、observed_grid.hpp | easyfly_occupancy_mapping | 膨胀别名及具体地面类型的薄包装，保留原函数签名 |

observed_grid的地面类型绑定包装不是第二份占据算法；平台的map_validity、来源/时间/TF接纳、
消息失效、全局预算和飞行所有权也不是应搬走的重复计算。
Python投影/类别关联已调用普通模块，未另复制一份旧Python算法。
FAST-LIO内嵌ikd-Tree/IKFoM、EGO内部地图/搜索/优化随原工程保留，不按功能分类拆散上游模块。

ALG-2..7的既有安装消费者已验证这些旧入口，详细证据仍在各批次验收文件。
本批不重复宣称执行那些构建/测试；只核对现有转发内容和SHA，保留原公开路径。
未来删除前应完成仓库反向引用、外部安装消费者、公开兼容承诺及替代入口核对，再明确安排破坏性版本变更。
只发现内部调用改用新头，不足以证明旧公开入口可删。

本机旧 `third_party/slam/fast_lio` 和 `third_party/planners/ego_planner` 是忽略的软链接，
父目录有本地COLCON_IGNORE。它们不提交、不参与包发现，服务已启动旧二进制和原安装资源链接。
主安装重新构建并确认旧消费者结束后再移除；本批不执行仿真启停，存活状态以当次进程核对为准。
旧SDK build缓存中的历史绝对路径和ALG-0盘点/历史Git命令保留为历史记录，不能作为新消费者的源码include路径。
