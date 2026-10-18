# 第三方推理运行库

ALG-5 将本地 ONNX Runtime SDK 从 ../onnxruntime 移至 onnxruntime/。
SDK 载荷继续忽略，仅本索引与集成说明受跟踪；模型权重保留在工作区 models/，不属于算法源码。

当前本地 SDK 元数据为 Microsoft ONNX Runtime 1.23.2 linux-x64 GPU，
GIT_COMMIT_ID 为 a83fc4d58cb48eb68890dd689f94f28288cf2278，许可为 SDK 自带 MIT LICENSE，
上游第三方许可保存在 ThirdPartyNotices.txt。获取入口：
https://github.com/microsoft/onnxruntime/releases/tag/v1.23.2 。
现有归档来源文件没有保存，不能据目录内容伪造原下载归档 hash。
本批保存完整文件/符号链接指纹，除本项目 EASYFLY_SETUP.md 更新路径/责任外，载荷未改。

`easyfly_yolo_onnx` 的 ORT_ROOT 默认为此 SDK，可显式选择外部 SDK；缺头/库明确配置失败。
只有实现包编译时读取 SDK 头，公开接口使用 PIMPL，不导出本地 SDK include 路径。
实现包安装实际 libonnxruntime、SONAME 别名及 SDK 中的 providers_*.so，并设置 $ORIGIN RPATH。
检测 ROS 功能包通过算法包使用运行库，不再重复安装 SDK。

CUDA provider 仍要求宿主机匹配的 CUDA/cuDNN/驱动；随包 provider 不等于携带整套 GPU 系统库。
请求 CUDA 时保留原有失败回退 CPU 规则，必须报告实际 provider。
SDK 内含 TensorRT provider 文件不代表 EasyFly 已实现 TensorRT 检测后端。

当前元数据、载荷/符号链接SHA及原下载归档缺口见[onnxruntime.provenance.json](onnxruntime.provenance.json)。
ALG-8.5只补登记，SDK载荷未修改、重取或升级；模型身份与许可另见[模型资产清单](../model-assets.provenance.json)。
