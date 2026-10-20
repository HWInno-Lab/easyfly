# Classical detector ROS transport

两种独立图像检测节点共享的 ROS 接纳模板，不包含检测算法。导出
`easyfly_image_detection_common::easyfly_image_detection_common` 与 `node.hpp`。

仅支持每进程一个节点、单线程 executor；同步检测，DDS 输入 SensorDataQoS depth=1，
输出 reliable/volatile depth=1，无无界 worker 队列。输入限制为 2073600 像素及 16 MiB
行布局，支持 BGR8/RGB8 与行尾填充；是否允许 mono8 由具体检测器声明。

要求正值、递增的源图像时间和匹配 optical_frame，源时间最多允许提前 50 ms。
ROS 时间和 steady time 双重过期检查，墙钟 timer 在停流/仿真时钟冻结时发送 invalid。
时钟回退先作废旧 epoch 并重置时间水位，后续新图像可恢复；重复/乱序图像作废，不能续期。
处理结束后再次检查新鲜度，过期结果不发布为有效。

valid=true 且 observations=[] 是有效无目标图像；valid=false 表示不可用且始终清空观测。
结果携带原图 header 和尺寸，失效通知保留最后一次源 header。capabilities.ready 表示
最近成功处理图像仍有效（不要求有目标），不是飞行安全认证。启动时没有图像时 ready=false。
所有检测/传输参数只读，修改后需重启节点。
