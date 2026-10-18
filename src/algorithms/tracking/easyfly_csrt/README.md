# easyfly_csrt

EasyFly 自有 CSRT 包装，C++17 INTERFACE target `easyfly_csrt::easyfly_csrt`。
系统 OpenCV tracking 提供实际 CSRT；本包提供初始化/update/reset 生命周期以及原 ROI/pixels 换算。

`make_tracker("csrt")` 返回单目标接口；未知后端抛 invalid_argument。
初始化先清旧状态，要求非空 BGR8 图和图内至少 8×8 像素 ROI；成功才保存模型状态。
update 使用原 OpenCV 默认 CSRT 参数；未初始化或图像格式错误返回 ok=false，OpenCV 运行异常向外传播。
返回像素框，calibrated_confidence 永远 nullopt，因为 OpenCV CSRT 没有提供校准分数。
reset 清空模型和框，重定位需要再次 initialize。实例只支持串行调用，不改变全进程线程策略。

图像只在调用期间借用；ROS worker 继续拥有消息/转换像素。
时间/generation、ROI 服务接纳、重定位、过期/时钟回退和结果发布继续留在 easyfly_single_object_tracking；
兼容消息的 confidence=0 保留原 unknown 表示。旧 track_core.hpp 仅为命名空间别名。
