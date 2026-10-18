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
目录分类不决定包名或编译顺序，`package.xml` 才是工作区依赖闭包的依据。

## 1. 包与导出

- 独立包包含 package.xml、CMakeLists.txt、include/包名、README 和必要测试。
- 自有源码使用根项目 Apache-2.0；导入源码保留原来源与许可，不套用本模板覆盖。
- 可独立替换的实现单独建包，接口包不依赖具体算法，不引入跟踪/规划之外的平台对象。
- 公开头不包含 ROS 消息、节点、TF 或 SDK；Eigen/OpenCV 等计算库仅在实际需要的实现包声明。
- 通过 namespaced target 传递 C++ 标准、安装头路径和必要计算依赖；不让消费者手工添加
  相邻源码 include，也不从算法包反向链接 EasyFly 功能包。
- API 以 0.1 开始，当前不承诺跨版本稳定 ABI。改变契约需同步适配、测试和文档，消费者重新编译。

现有 target：

| 包 | 头文件 | CMake target |
| --- | --- | --- |
| easyfly_tracking_api | easyfly_tracking_api/tracker.hpp | easyfly_tracking_api::easyfly_tracking_api |
| easyfly_grid_planner_api | easyfly_grid_planner_api/planner.hpp | easyfly_grid_planner_api::easyfly_grid_planner_api |
| easyfly_sort | easyfly_sort/sort.hpp | easyfly_sort::easyfly_sort |
| easyfly_track_id_fusion | easyfly_track_id_fusion/id_consolidator.hpp | easyfly_track_id_fusion::easyfly_track_id_fusion |
| easyfly_astar | easyfly_astar/astar.hpp | easyfly_astar::easyfly_astar |
| easyfly_coverage_planning | easyfly_coverage_planning/search_geometry.hpp | easyfly_coverage_planning::easyfly_coverage_planning |
| easyfly_motion_prediction | easyfly_motion_prediction/predictor.hpp | easyfly_motion_prediction::easyfly_motion_prediction |
| easyfly_pointcloud_filters | easyfly_pointcloud_filters/filter_core.hpp | easyfly_pointcloud_filters::easyfly_pointcloud_filters |

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
