# easyfly_yolo_onnx

EasyFly 自有 YOLO ONNX 包装，C++17 SHARED target `easyfly_yolo_onnx::easyfly_yolo_onnx`。
维护模型布局检查、letterbox、BGR→RGB/NCHW、置信度筛选、按类别 NMS、像素坐标还原。
实际推理由第三方 ONNX Runtime 完成；这不是自有神经网络实现。

`createBackend(DetectorParams)` 当前只支持 onnx；未知后端及预留 tensorrt/openvino 明确失败。
输入/输出/线程和 timing 契约见 easyfly_detection_api。后端持有缓冲和上次 letterbox，必须串行调用。
保留原 CUDA 初始化失败回退 CPU 规则，通过 activeProvider() 获取实际 provider。
ORT intra-op threads 仍为 2，不改变全进程 OpenCV 线程设置。

仅本包编译时使用 SDK；PIMPL 隐藏 ORT 头和类型。
ORT_ROOT 默认 src/third_party/runtimes/onnxruntime，可用 -DORT_ROOT=/external/sdk 覆盖。
缺头或主库在 CMake 配置时失败。安装本实现库、真实 ORT 库、动态 SONAME 别名、SDK 提供的 provider 库和许可，
安装 RPATH 为 $ORIGIN。普通 CMake 消费不需要 ROS 环境或 SDK 头；GPU 系统驱动、CUDA/cuDNN 仍由主机提供。

SDK 来源见 third_party/runtimes/README.md。权重属于模型资产，仍保留于工作区 models/；
功能包负责安装默认权重并解析 package:// 路径，算法只接收可读的文件路径。
ALG-5 对固定 RGB 帧分别保存迁移前 CPU/CUDA 实推理结果，比较类别、顺序、置信度、像素框及 caller timing。
这验证软件迁移等价，不代表新的数据集精度评估。
