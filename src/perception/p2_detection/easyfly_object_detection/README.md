# easyfly_object_detection

## ALG-5 算法边界

检测计算已迁入 easyfly_detection_api / easyfly_yolo_onnx；旧头仅转发。话题/QoS、worker、图像所有权和 timing 采集留在本包。默认权重安装于 share/easyfly_object_detection/models，YAML 使用 package:// 路径；绝对/普通相对路径仍按原规则处理。

固定图像的 ROS 数值回归分别检查 CPU 与请求 CUDA 的入口。CPU 就绪预算为 15 秒；
CUDA 首次使用空驱动缓存可能在建立图像订阅前编译内核，因此该分支有界等待 60 秒，
并在 JUnit 中记录实际启动耗时。就绪后的推理仍为 10 秒预算，原类别、置信度、框和
消息头断言保持。这是数值契约测试，不承诺 GPU 空缓存在 15 秒内启动。
