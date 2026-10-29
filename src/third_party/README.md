# 第三方分类索引

ALG-8.1..8.5已完成当前依赖的分类与接入记录；ALG-9整体构建/受管场景回归仍待执行。
[分类总表](catalog.json)记录主要位置、功能标签、接入方式、版本证据和未知项；
[来源与补丁规范](sourcing-and-patches.md)说明原许可、已知本地历史与新增项目登记方式。
历史ALG-0清单保持不变，当前布局以本索引及[迁移进度](../algorithms/migration-progress.json)为准。

## 当前项目

| 主要位置 | 类型与次级用途 | 接入/来源记录 |
| --- | --- | --- |
| algorithms/localization/fast_lio | 完整ROS定位/惯性建图；内嵌ikd-Tree/IKFoM | [说明](algorithms/localization/README.md) · [清单](algorithms/localization/fast_lio.provenance.json) |
| algorithms/planning/ego_planner | 完整8包ROS规划、搜索、建图和轨迹优化 | [说明](algorithms/planning/README.md) · [清单](algorithms/planning/ego_planner.provenance.json) |
| algorithms/planning/super | 固定官方 SUPER 与 ROG-Map；原生 A*/CIRI/MINCO-S4 | [说明](algorithms/planning/README.md) · [清单](algorithms/planning/super.provenance.json) |
| runtimes/onnxruntime | 本地可选推理运行库，载荷忽略；CPU/CUDA依赖 | [说明](runtimes/README.md) · [清单](runtimes/onnxruntime.provenance.json) |
| sensor_sdks/livox_sdk2 | 原生雷达SDK | [说明](sensor_sdks/README.md) · [清单](sensor_sdks/livox_sdk2.provenance.json) |
| sensor_sdks/librealsense | 原生深度相机SDK | [说明](sensor_sdks/README.md) · [清单](sensor_sdks/librealsense.provenance.json) |
| tools/qgc | 外部程序接入资料 | [说明](tools/qgc/README.md) · [清单](tools/qgc/provenance.json) |

每个外部项目保留一个主要位置，其他用途以tags记录；完整上游工程的内部模块不按本项目分类拆散或复制。
自有可复用计算及普通包装位于[algorithms](../algorithms/README.md)，新算法的具体步骤见
[跟踪器/规划器接入指南](../algorithms/integration-guide.md)。算法名称相同不表示代码来源相同。

## 分类规则与保留边界

外部算法沿用preprocessing、detection、tracking、localization、fusion_mapping、planning分类，
按实际源码创建目录，分类父目录不创建package.xml。已有完整ROS工程继续通过话题/会话接入；
普通库通过安装target/module消费。目录分类没有创建自动发现或热切换的算法加载器。

ROS设备驱动保留在communication/c1_drivers，包括Livox驱动和RealSense嵌套仓库；
其gitlink、许可和初始化缺口在catalog中登记，不为算法分类改变驱动所有权。
模型资产保持工作区models位置，ONNX由easyfly_object_detection安装；
其SHA及来源/许可缺口见[模型清单](model-assets.provenance.json)，不与ORT的MIT许可混同。
系统OpenCV/PCL/Eigen保持系统计算依赖，不重复vendoring。

geographic_lib、nlopt、nlohmann_json目前仍为空目录，不代表已集成库。
真正导入时分别归libraries/geometry、libraries/optimization、libraries/serialization并补来源证据；
RealSense私有构建中的JSON源码缓存是另行固定的构建输入，不是这个空nlohmann_json目录。

## 实际选择与当前会话

- ONNX Runtime的ORT_ROOT可显式覆盖，只有easyfly_yolo_onnx拥有运行库/provider/许可安装。
  公开接口隐藏SDK类型；CUDA就绪取决于宿主依赖，必须报告实际provider。TensorRT文件存在不代表实现了该后端。
- Livox头/库通过find_path/find_library解析，RealSense通过find_package(realsense2 2.56)解析。
  源码目录不会自动替换/usr/local安装；验收核对实际编译头和动态库。SDK构建缓存保持忽略，不复用历史路径。
- 用户QGC.AppImage保持src/third_party/QGC.AppImage原位并忽略，程序继续由用户显式选择。
  文档目录不是默认可执行文件目录；离线预览与联网维护各自使用原有UI接入规则。
- FAST-LIO/EGO的旧源码路径可能保留本机忽略的软链接，父目录COLCON_IGNORE排除重复包。
  它们服务当前旧安装/进程，不能作为新源码依赖；旧消费者退役后再清理，分类工作不启停仿真。

[ALG-8.1](../algorithms/alg8-1-validation.md)、[ALG-8.2](../algorithms/alg8-2-validation.md)、
[ALG-8.3](../algorithms/alg8-3-validation.md)、[ALG-8.4](../algorithms/alg8-4-validation.md)保留各批实际验证范围。
[ALG-8.5](../algorithms/alg8-5-validation.md)只收敛索引、指南和兼容审计，
不宣布历史ALG0-B03、semantic退出问题或来源/许可未知项已解决。
