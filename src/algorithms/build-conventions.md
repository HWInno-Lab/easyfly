# 算法包构建与消费约定

ALG-1 建立 `tracking/easyfly_tracking_api` 和 `planning/easyfly_grid_planner_api`。
二者均为 ament 导出的 C++17 header-only INTERFACE target，没有 ROS 运行库或具体算法依赖。
ALG-2 新增 `tracking/easyfly_sort` 与 `tracking/easyfly_track_id_fusion` 两个标准库实现包，
同样导出 INTERFACE target；原跟踪功能包现已通过适配器依赖 tracking API 与这两个实现。
ALG-3 新增 `planning/easyfly_astar` 与 `planning/easyfly_coverage_planning`；前者依赖
grid planner API 的 SearchControl，后者仅依赖标准库。全局规划和 combined 协调器
通过工厂调用 A*，搜索任务直接消费覆盖几何；功能包继续拥有执行与运行接纳。
ALG-4 新增 `tracking/easyfly_motion_prediction` 和 `preprocessing/easyfly_pointcloud_filters`，
均仅依赖标准库。节点直接使用已有普通类型，显式传入时间；TF/队列保留在功能包。
ALG-5 新增 detection/single_tracking API 和 YOLO/标志位姿/CSRT 包装。API 使用 OpenCV 图像类型，
不依赖具体实现；YOLO 私有依赖 ONNX Runtime，通过 PIMPL 隐藏 SDK，单独负责运行库安装。
ALG-6 新增标准库 C++ easyfly_projection，以及标准库 Python easyfly_projection_py 和
easyfly_obstacle_association。Python 包由 ament_cmake_python 安装，公开普通模块，不创建空 C++ target。
ALG-7.1 新增 easyfly_ground_segmentation，导出 SHARED target，公开系统 PCL/Eigen 计算类型，
不引入 ROS 消息或 TF 查询。地面拟合与模型在算法包，状态/参考系接纳和地图输出在功能包。
ALG-7.2 新增 easyfly_obstacle_geometry，导出 SHARED target，聚类/AABB 依赖 PCL，
动态跟踪头仅依赖 STL；不包含 ROS 运行依赖。节点保留地图、时效接纳和全局预测点数预算。
ALG-7.3 新增 easyfly_occupancy_mapping，导出 STL-only INTERFACE target；原真实地面模型
仅为测试依赖，不导出 PCL/地面包依赖。节点管理消息、失效协议与历史 reset 时机。
目录分类不决定包名或编译顺序，`package.xml` 才是工作区依赖闭包的依据。

## 1. 包与导出

- 独立包包含 package.xml、CMakeLists.txt、README 和必要测试；C++ 使用 include/包名，Python 使用同名模块目录。
- 自有源码使用根项目 Apache-2.0；导入源码保留原来源与许可，不套用本模板覆盖。
- 可独立替换的实现单独建包，接口包不依赖具体算法，不引入跟踪/规划之外的平台对象。
- 公开头不包含 ROS 消息、节点、TF 或 SDK；Eigen/OpenCV 等计算库仅在实际需要的包声明；图像 API 显式声明 OpenCV。
- 通过 namespaced target 传递 C++ 标准、安装头路径和必要计算依赖；不让消费者手工添加
  相邻源码 include，也不从算法包反向链接 EasyFly 功能包。
- API 以 0.1 开始，当前不承诺跨版本稳定 ABI。改变契约需同步适配、测试和文档，消费者重新编译。

现有 target：

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
| easyfly_csrt | easyfly_csrt/track_core.hpp | easyfly_csrt::easyfly_csrt |
| easyfly_projection | easyfly_projection/projection.hpp | easyfly_projection::easyfly_projection |
| easyfly_projection_py | easyfly_projection_py/core.py | easyfly_projection_py.core（Python） |
| easyfly_obstacle_association | easyfly_obstacle_association/core.py | easyfly_obstacle_association.core（Python） |
| easyfly_ground_segmentation | easyfly_ground_segmentation/ground_fitting.hpp | easyfly_ground_segmentation::easyfly_ground_segmentation |
| easyfly_obstacle_geometry | easyfly_obstacle_geometry/clustering.hpp / dynamic_obstacle_tracker.hpp | easyfly_obstacle_geometry::easyfly_obstacle_geometry |
| easyfly_occupancy_mapping | easyfly_occupancy_mapping/occupancy_grid.hpp / observed_grid.hpp / grid_inflation.hpp | easyfly_occupancy_mapping::easyfly_occupancy_mapping |

C++ 实现包可导出普通 STATIC/SHARED 库，按 PUBLIC/PRIVATE 划分依赖。header-only 包导出
INTERFACE target，不使用 HAS_LIBRARY_TARGET 伪装成运行时动态库。默认导出命名空间为包名。

## 2. 消费方式

```cmake
find_package(easyfly_tracking_api REQUIRED)
target_link_libraries(my_backend PRIVATE easyfly_tracking_api::easyfly_tracking_api)
```

若消费者自身的公开头暴露该 API，应使用 PUBLIC 链接并导出对应依赖；ROS 消费包在
package.xml 声明 `<depend>easyfly_tracking_api</depend>`。具体配置参数由实现包定义，
节点把算法参数、话题/QoS 参数和任务策略分别管理。

纯 CMake 消费者可只把两个包的安装前缀加入 CMAKE_PREFIX_PATH，无须 source ROS 环境。
示例位于 `../../tools/algorithms/consumer/`，包含仅测试用途的两种跟踪和两种规划替身。
这些替身不执行关联或障碍搜索，不属于平台可选生产后端。

## 3. 构建与验证

在加载 ROS 构建工具的工作区中可使用：

```bash
colcon build --base-paths src --packages-up-to \
  easyfly_tracking_api easyfly_grid_planner_api
```

ALG-1 依赖闭包仅含这两个源码包，ament/gtest 来自已安装构建/测试工具。
ALG-2 已同步更新跟踪包的 package.xml 与链接 target，现有 build_layer.bash 的
--packages-up-to 会自动补入对应算法依赖。ALG-3 已按同样方式声明规划/覆盖实现依赖。

完整的可重复验证入口：

```bash
python3 tools/algorithms/verify_api_install.py \
  --output artifacts/algorithms/alg1-new-run
```

该工具拒绝覆盖既有输出。它执行：

1. 用清理过的环境加 `/opt/ros/humble` 发现包和构建，只选择两个 API 包。
2. 分别进行 symlink-install 和普通复制安装，运行包内 GTest。ament 测试包装器使用 ROS
   的构建/测试 Python 环境；这不是算法的 ROS 运行依赖。
3. 将纯 CMake 示例复制到独立目录，在无 ROS overlay、无原工作区安装前缀的环境中构建和测试。
4. 普通安装再将安装根重定位，验证导出 target 和普通头文件可从新路径消费。
   此项只验证 CMake 导出/头文件，不承诺 colcon 生成的所有 shell 环境脚本可重定位。
5. 检查 include 与编译命令使用安装前缀，C++17 由 target 传递；用 ldd 检查示例无 ROS 动态库。
6. 保存命令、日志、JUnit 和源文件 hash，并要求验证期间源文件没有变化。

symlink-install 的安装头自然链接源文件，这是该安装模式的正常行为；消费者仍必须通过
安装 target 定位头文件。普通复制安装的头不得为指向源码的软链接。

## 4. ALG-1 的历史边界

已建立跟踪/规划数据类型、输入校验、时间序列与搜索控制辅助，以及抽象调用接口。
虚接口不自动保证实现遵守契约；实际适配器需处理错误、reset、能力声明、身份回传和输出校验。
本批没有实现 SORT/A* 适配器，没有赋予旧 A* 搜索循环取消/预算能力，没有更改节点或飞行行为。

实际结果见 [ALG-1 验收](alg1-validation.md)。ALG-0 已有两个失败项保留在历史基线，
本批未修改相关源码，也未将其重新报告为通过。

ALG-2 已实现 SORT 适配器、reset 与实际 ROS 回归，结果及安装验证命令见
[ALG-2 验收](alg2-validation.md)。适配器共享库使用标准 CMake find_dependency 导出
公开算法依赖，普通 CMake 消费时无需初始化 ament Python 构建环境。

ALG-3 的真实 A*/覆盖几何、普通 CMake 适配器库与安装矩阵见
[ALG-3 验收](alg3-validation.md)，复验工具为 `../../tools/algorithms/verify_planning_install.py`。
两个规划实现与 grid planner API 的源码依赖闭包仅自身三包，不引入 ROS 运行库。

ALG-4 的预测/过滤核心、双时钟与源时间 TF 回归见 [ALG-4 验收](alg4-validation.md)。
旧 ROS 功能包继续导出其 ROS 元数据；新纯核心可单独 find_package，无需 ROS overlay。

ALG-5 图像/单目标接口和 OpenCV 包装、SDK 私有依赖/许可安装、旧头与搬迁验证见
[ALG-5 验收](alg5-validation.md)。普通 CMake 通过五个核心 prefix 消费，
独立编译不需要 ROS 或 SDK include。ORT_ROOT 显式外部覆盖由普通安装矩阵验证。

ALG-6 同时验证普通 C++ target 和 Python 已安装模块：
[ALG-6 验收](alg6-validation.md)，复验工具 `../../tools/algorithms/verify_fusion_install.py`。
Python 模块位于平台选择的 site-packages/dist-packages；消费夹具按显式 prefix 发现目录，
在 python -I 与清理环境下导入，不读取 ROS overlay 或工作区源码路径。普通安装搬迁后重复消费。

ALG-7.1 纯 CMake 消费显式声明 LANGUAGES C CXX，以满足系统 PCL/VTK 的 C/MPI imported target。
这是本机计算依赖的配置要求，不是 ROS 运行依赖。旧 semantic_map 地面头保留原安装路径，
通过其导出 include/依赖消费；原四组地面测试已迁入实现包。详见 [ALG-7.1 验收](alg7-1-validation.md)。

ALG-7.2 同样使用系统 PCL 的 C/CXX 配置；旧动态头导出全部原公开类型/函数别名。
普通安装根搬迁后用新 prefix 重配、构建和运行纯 CMake 消费者，另在 ROS 环境消费旧头。
复验工具 `../../tools/algorithms/verify_obstacle_install.py`，详见 [ALG-7.2 验收](alg7-2-validation.md)。

ALG-7.3 纯 CMake 消费仅声明 CXX，只提供 occupancy prefix 即可，不需要 ROS、PCL 或地面库。
原 bounded-layer 测试通过 test_depend 使用实际地面类型；colcon源码闭包包含该测试依赖。
旧 observed_grid 头用薄转发保留具体地面指针签名；旧 ROS 功能包配置仍需支持的 ROS/C/PCL 环境。
复验工具 `../../tools/algorithms/verify_occupancy_install.py`，见 [ALG-7.3 验收](alg7-3-validation.md)。

ALG-8.1 FAST-LIO 保留完整 ROS2 工程，包名仍为 fast_lio，位置为
`../third_party/algorithms/localization/fast_lio`；不要求其成为无 ROS 普通库。
CMake FASTLIO_OUTPUT_ROOT 可隔离 Log/PCD输出；配置/消息/算法不变，原许可与 README 随 share 安装。
两种安装均验证原生进程、配置读取和 Pose6D 消息序列化；复制安装搬迁后显式指定 fast_lio_DIR，
并将该 prefix 的 lib 放在本次进程加载路径首位，不依赖旧源码或全局修改 LD_LIBRARY_PATH。
已启动旧安装使用的源码软链接桥接仅本地忽略，并由旧父目录 COLCON_IGNORE 排除。
见 [ALG-8.1 验收](alg8-1-validation.md)，不将完整 ROS 工程冒称为纯 C++ 核心。

ALG-8.2 EGO 的8包整体位于 `../third_party/algorithms/planning/ego_planner`。
plan_env/path_searching/traj_utils_so 通过包导出目标传播依赖；不拼接相邻包源码目录或猜测工作区根。
traj_utils_so 同时保留原消息导出并提供其普通库 target；ament_target_dependencies 已链接的目标
不再通过原始 *_LIBRARIES 重复展开。原优化选项保持。
普通复制安装根搬迁后，以每包的 *_DIR 及进程级 AMENT_PREFIX_PATH/LD_LIBRARY_PATH/PYTHONPATH
显式选择新库、消息和节点，消费端仍需 ROS/PCL 等依赖；不承诺生成的整套 colcon shell 脚本任意搬迁。
复验工具 `../../tools/algorithms/verify_ego_install.py`，见 [ALG-8.2 验收](alg8-2-validation.md)。


ALG-8.3 两份传感器SDK完整源码位于 `../third_party/sensor_sdks`，ROS驱动保持communication/c1_drivers。
目录分类不会自动改变驱动SDK选择：默认仍可解析/usr/local，私有安装需显式LIVOX头/库及realsense2_DIR。
新SDK与复制驱动安装根均实际搬迁，复验以实际编译头依赖和动态库加载为准，不能仅凭源目录存在宣称接入。
SDK原构建及许可证不变；验证工程单独复用固定JSON缓存、关闭固件/联网等无关构建选项并跳过系统ldconfig。
工具为 `../../tools/algorithms/verify_sensor_sdks.py`；范围和复验前提见 [ALG-8.3验收](alg8-3-validation.md)。
