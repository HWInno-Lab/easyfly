# easyfly_single_tracking_api

C++17 INTERFACE target `easyfly_single_tracking_api::easyfly_single_tracking_api`，依赖 OpenCV core。
单目标跟踪接口独立于多目标 Tracker，公开类型不依赖 ROS 或具体跟踪后端。

`Tracker::initialize(bgr,pixel_roi)` 同步借用 BGR8 图与原图像素 ROI；`update(bgr)` 返回 ok、像素框和 optional calibrated_confidence。
调用者保持图像存活且不并发修改。实例串行拥有；后端可保留自己的模型状态，不能持有调用者像素视图。
`reset()` 清空模型，`initialized()` 表示已初始化；新实例或 reset 后必须重新初始化。
false/异常由适配器映射到原有状态原因，未知置信度使用 nullopt，不能以 1 表示成功。

时间戳、generation、图像尺寸/校准变化、源时间/接收时间、TTL 和 ROS 消息归一化由调用方管理。
停止/重定位先使旧 generation 无效，再 reset/重建；算法接口不读取 ROS 时钟或发布状态。
