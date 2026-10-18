# easyfly_detection_api

检测计算契约，C++17 INTERFACE target `easyfly_detection_api::easyfly_detection_api`。
仅依赖 OpenCV；公开类型不含 ROS、话题、QoS 或 worker。

`DetectorParams` 只保留模型、类别表、输入尺寸、阈值、provider 和结果上限。
`DetectorBase::detect()` 同步借用非空 BGR8 `cv::Mat`，支持 OpenCV stride；调用者保持像素存活且不并发修改。
输出 Detection 拥有类别字符串和原图像素 xyxy 框，归一化留在节点。
实例由单线程拥有；不承诺并发调用。异常传播给适配器，不能把异常当成功空结果。

DetectionTiming 使用 steady duration 毫秒。算法只写 preprocess/inference/postprocess，
保留调用者的 queue/color_conversion/debug/end_to_end 字段。
新后端继承 DetectorBase，具体工厂属于实现包；API 不依赖 YOLO/ONNX Runtime。
