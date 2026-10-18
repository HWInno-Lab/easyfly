# easyfly_tracking_api

ROS 运行时无关的 C++17 多目标跟踪接口。ALG-1 建立类型、输入校验、时间序列辅助和抽象接口；
ALG-2 已迁移真实SORT并接入平台适配器，当前生产工厂仍只支持sort。

```cmake
find_package(easyfly_tracking_api REQUIRED)
target_link_libraries(my_tracker PRIVATE easyfly_tracking_api::easyfly_tracking_api)
```

公共头：`easyfly_tracking_api/tracker.hpp`，命名空间 `easyfly::tracking`。
构建导出依赖 ament，安装后的头文件和 target 仅要求标准 C++17；消费者无需 ROS 节点库。

- 输入 Frame 自有检测数组和特征，每个 bbox 是原图像素 xyxy，允许未裁剪但有限的框。
  可选 ImageView 是同步借用，包含编码、步长、尺寸和字节数，update 返回后不得保留其指针。
- Requirements 显式声明图像、特征维数和低置信度检测需求；新增后端不能假设现有检测链
  已保留所需输入。未满足返回 unsupported_input。
- `validate_frame` 检查结构、有限值、需求与调用方资源限制，不管理 ROS 新鲜度、TF、线程或
  后端内部状态。Limits 的默认值是接口校验上限，不替代 SORT 的 max_tracks 等算法配置。
- `reset(epoch)` 清空全部内部历史；新 epoch 可以重新从小时间戳开始。同 epoch 内源时间
  严格递增；FrameSequence 可辅助实现，只有 backend 成功后才 commit。
- 输出同时保留滤波框、最近观测框、原始 ID、生命周期和速度单位。ID 由 epoch 限定；
  调用方重置时也要清理世界观测和 ID 整合历史。稳定 ID 整合不在这个接口内。
- SORT 首轮仍使用 pixels_per_frame；其他后端可以输出 pixels_per_second，消费者必须读取单位。
- ok+空数组表示有效空结果，失败时 tracks 必须为空；默认结果是 backend_error。
- 算法实例串行调用。取消、异步任务、单目标 ROI 管理、服务和飞行策略不属于此接口。

抽象接口本身不能保证任意外部实现遵守契约。真实SORT适配器与回归见[ALG-2验收](../../alg2-validation.md)。
ALG-1测试替身仅验证类型替换和接口含义，不能代替真实跟踪精度或ID稳定性验收。
新增需求及当前节点未提供的图像/特征输入见[接入指南](../../integration-guide.md)。
0.1 接口尚无稳定二进制 ABI 承诺，消费包需随接口变更重新编译。
