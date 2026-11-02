# EasyFly 算法目录

本目录存放项目维护的可复用计算实现、算法包装与能力接口。节点接线、消息转换、时间/坐标接纳和任务执行留在对应功能包。

## 开发文档

- [新增算法接入指南](docs/integration-guide.md)：职责划分、登记/工厂/配置、输入扩展和交付检查。
- [接口与行为契约](docs/interface-contracts.md)：公共类型、数值语义、状态、生命周期和平台边界。
- [感知版本与配置契约](docs/extension-contracts.md)：V1/V2、完整身份、来源引用及配置角色。
- [构建与消费约定](docs/build-conventions.md)：包依赖、导出 target、SDK 隔离与安装消费。
- [椭圆与彩线检测](docs/classical-detection.md)：经典图像检测的功能、接口与使用方式。
- [第三方总表](../third_party/catalog.json)与[来源/补丁规范](../third_party/sourcing-and-patches.md)：上游版本、许可和本地修改。

## 能力接口与实现

| 能力 | 公共接口 | 代表实现 |
| --- | --- | --- |
| 检测 | [easyfly_detection_api](detection/easyfly_detection_api/README.md) | [YOLO/ONNX](detection/easyfly_yolo_onnx/README.md) |
| 单目标跟踪 | [easyfly_single_tracking_api](tracking/easyfly_single_tracking_api/README.md) | [CSRT](tracking/easyfly_csrt/README.md)、[KCF](tracking/easyfly_kcf/README.md) |
| 多目标跟踪 | [easyfly_tracking_api](tracking/easyfly_tracking_api/README.md) | [SORT](tracking/easyfly_sort/README.md)、[ID 整合](tracking/easyfly_track_id_fusion/README.md) |
| 二维栅格规划 | [easyfly_grid_planner_api](planning/easyfly_grid_planner_api/README.md) | [A*](planning/easyfly_astar/README.md)、[覆盖几何](planning/easyfly_coverage_planning/README.md) |

其他预处理、检测、预测与融合建图组件见各包 README。完整 ROS 引擎位于 `../third_party/algorithms/`，保持上游工程组织，通过平台适配接入。分类父目录不放 `package.xml`，不创建空算法包占位。

```text
EasyFly 节点 → 适配器/工厂 → 算法库 → 能力接口/计算依赖
```

公共接口不依赖具体实现、EasyFly 消息、设备 SDK 或飞控对象。算法库使用显式输入，不接管 ROS 时钟、TF 查询、消息发布或飞行所有权。消费者使用安装后的 target/module，不跨包引用源码。

历史测试、验收和分批实施 Markdown 已从本目录移出；机器记录、测试脚本及原始证据保留在各自位置。后续开发以 docs 中的指南、各包 README 和实际源码为准。
