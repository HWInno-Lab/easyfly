# 算法包构建与消费约定

目录分类不决定包名或编译顺序，`package.xml` 才是工作区依赖闭包的依据。算法计算、ROS 适配与飞行执行的职责见[接入指南](integration-guide.md)。

## 包与导出

- 独立包包含 package.xml、CMakeLists.txt、README 和必要测试；C++ 头位于 include/包名，Python 使用独立模块目录。
- 项目维护源码遵循项目许可；导入源码保留上游版本、版权、许可证及本地补丁记录。
- 能力接口不依赖具体实现、ROS 消息、节点、TF 查询或设备 SDK；OpenCV/Eigen 等计算类型按实际需要声明。
- 导出 namespaced target，传递 C++ 标准、安装头路径及必要公共依赖。公开头暴露某依赖时使用 PUBLIC/INTERFACE，私有 SDK 留在实现内部。
- ABI 或计算契约改变时同步适配、文档和消费者重建；不能混用不匹配的头与旧二进制。

## 已有导出目标

| 包 | 头文件/模块 | CMake target / Python import |
| --- | --- | --- |
| easyfly_tracking_api | easyfly_tracking_api/tracker.hpp | easyfly_tracking_api::easyfly_tracking_api |
| easyfly_grid_planner_api | easyfly_grid_planner_api/planner.hpp | easyfly_grid_planner_api::easyfly_grid_planner_api |
| easyfly_sort | easyfly_sort/sort.hpp | easyfly_sort::easyfly_sort |
| easyfly_track_id_fusion | easyfly_track_id_fusion/id_consolidator.hpp | easyfly_track_id_fusion::easyfly_track_id_fusion |
| easyfly_astar | easyfly_astar/astar.hpp | easyfly_astar::easyfly_astar |
| easyfly_coverage_planning | easyfly_coverage_planning/search_geometry.hpp | easyfly_coverage_planning::easyfly_coverage_planning |
| easyfly_motion_prediction | easyfly_motion_prediction/predictor.hpp | easyfly_motion_prediction::easyfly_motion_prediction |
| easyfly_pointcloud_filters | easyfly_pointcloud_filters/filter_core.hpp | easyfly_pointcloud_filters::easyfly_pointcloud_filters |
| easyfly_detection_api | easyfly_detection_api/detector.hpp | easyfly_detection_api::easyfly_detection_api |
| easyfly_single_tracking_api | easyfly_single_tracking_api/tracker.hpp | easyfly_single_tracking_api::easyfly_single_tracking_api |
| easyfly_yolo_onnx | easyfly_yolo_onnx/backend_factory.hpp | easyfly_yolo_onnx::easyfly_yolo_onnx |
| easyfly_tag_pose | easyfly_tag_pose/tag_core.hpp | easyfly_tag_pose::easyfly_tag_pose |
| easyfly_ellipse | easyfly_ellipse/detector.hpp | easyfly_ellipse::easyfly_ellipse |
| easyfly_color_line | easyfly_color_line/detector.hpp | easyfly_color_line::easyfly_color_line |
| easyfly_csrt | easyfly_csrt/track_core.hpp | easyfly_csrt::easyfly_csrt |
| easyfly_kcf | easyfly_kcf/tracker.hpp | easyfly_kcf::easyfly_kcf |
| easyfly_projection | easyfly_projection/projection.hpp | easyfly_projection::easyfly_projection |
| easyfly_projection_py | easyfly_projection_py/core.py | easyfly_projection_py.core（Python） |
| easyfly_obstacle_association | easyfly_obstacle_association/core.py | easyfly_obstacle_association.core（Python） |
| easyfly_ground_segmentation | easyfly_ground_segmentation/ground_fitting.hpp | easyfly_ground_segmentation::easyfly_ground_segmentation |
| easyfly_obstacle_geometry | easyfly_obstacle_geometry/clustering.hpp / dynamic_obstacle_tracker.hpp | easyfly_obstacle_geometry::easyfly_obstacle_geometry |
| easyfly_occupancy_mapping | easyfly_occupancy_mapping/occupancy_grid.hpp / observed_grid.hpp / grid_inflation.hpp | easyfly_occupancy_mapping::easyfly_occupancy_mapping |

C++ 实现包可导出普通 STATIC/SHARED 库，按 PUBLIC/PRIVATE 划分依赖。header-only 包导出
INTERFACE target，不使用 HAS_LIBRARY_TARGET 伪装成运行时动态库。默认导出命名空间为包名。

## 消费方式

```cmake
find_package(easyfly_tracking_api REQUIRED)
target_link_libraries(my_backend PRIVATE easyfly_tracking_api::easyfly_tracking_api)
```

ROS 消费包同时在 package.xml 声明真实依赖。普通 CMake 消费者显式提供相关安装前缀，不猜测源码树位置。Python 核心导出已安装模块，不创建空 C++ target。

公共工厂只传播能力接口与配置依赖，SORT、A*、YOLO、CSRT、KCF 等实现私有链接。消费旧具体算法转发头时，应显式声明对应实现依赖；不能依靠工厂间接暴露实现头。移除公开兼容入口前，先核对仓库及外部消费者并安排兼容迁移。

## 构建与依赖隔离

在加载系统 ROS 构建工具的终端，从工作区根执行：

```bash
colcon build --base-paths src --packages-up-to easyfly_single_object_tracking
```

使用 `--base-paths src` 避免扫描 artifacts 中的复制安装。独立构建/验证使用新的 build/install/log 目录；清理旧 overlay 的环境变量，仅加载明确列出的依赖前缀。

查询真实依赖闭包：

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/algorithms/extension_dependency_inventory.py \
  --backend kcf --kind build
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/algorithms/extension_dependency_inventory.py \
  --backend kcf --kind deployment
```

构建闭包、运行闭包和完整部署闭包分别检查。只存在 SDK 目录、库文件或登记条目，不能作为运行就绪证据。

- ONNX Runtime 通过 `ORT_ROOT` 显式选择；实现包负责自身运行库/provider/许可安装，公开接口隐藏 SDK 类型。
- Livox 显式选择头/库，RealSense 显式选择 `realsense2_DIR`，避免私有前缀被 `/usr/local` 覆盖；核对编译头和实际加载库。
- FAST-LIO/EGO 等完整 ROS 工程保持自己的依赖和消费方式，不要求改造成无 ROS 库。
- PCL/VTK 消费者可能需要同时启用 C、CXX，以满足系统 MPI imported target。
- 模型、配置及词袋等资产按安装路径解析，并保留来源、SHA 与许可记录。

## 安装消费检查

新增或修改后端时验证开发软链接和普通复制安装。消费者使用导出的 target/module；复制安装搬迁后重新配置、编译、运行，并核对编译命令与动态加载路径。

软链接安装允许依赖源码/SDK 链接目标；复制搬迁检查应确认没有意外读取原源码或构建 SDK。算法 target 的搬迁能力不表示 colcon 生成的整套 shell setup 脚本可以任意搬迁。

公共矩阵入口：

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/algorithms/verify_extension_dependencies.py \
  --output artifacts/extensibility/install-new \
  --ort-root src/third_party/runtimes/onnxruntime
```

输出目录必须不存在。该矩阵包含其他检测工厂的 ORT 检查；单独使用 KCF 不要求 ORT。算法专项工具位于 `tools/algorithms/verify_*_install.py`，参数以各工具 `--help` 为准。结果保存到 artifacts，不在算法目录新增分批测试报告。
