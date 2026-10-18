# EasyFly ONNX Runtime 集成说明

本目录存放 onnxruntime C++ SDK，因 `lib/libonnxruntime_providers_cuda.so` 单文件 352MB
超过 GitHub 100MB 限制，SDK 载荷被 .gitignore 排除（本集成说明单独跟踪），首次克隆后需手动放置。
easyfly_yolo_onnx 负责构建链接及运行库安装；easyfly_object_detection 通过该算法包消费。运行库和 provider 安装于算法包 lib，RPATH 为 $ORIGIN；CUDA/cuDNN/驱动仍由宿主机提供。

## 当前版本

- onnxruntime **1.23.2** linux-x64 **GPU** (CUDA EP)
- 结构: `include/onnxruntime_cxx_api.h` 等 + `lib/libonnxruntime*.so` (+ LICENSE 等上游文件)

## 获取

```bash
cd src/third_party/runtimes/onnxruntime
wget https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/onnxruntime-linux-x64-gpu-1.23.2.tgz
tar xzf onnxruntime-linux-x64-gpu-1.23.2.tgz --strip-components=1
```

注意:
- CUDA EP 需与本机 CUDA/cuDNN 匹配；不匹配时检测节点自动回退 CPU EP，实际 provider 必须记录。
- YOLO11n 模型文件在 `easyfly_ws/models/yolo11n.onnx`（models/ 已入库）。
- 当前模型的历史导出配置为 opset 12、IR version ≤ 10（见 schedule.md M2）；这不代表 SDK 的通用支持上限。
- 来源、commit、许可与本批指纹审计见 ../README.md；原下载归档 hash 未保存。
