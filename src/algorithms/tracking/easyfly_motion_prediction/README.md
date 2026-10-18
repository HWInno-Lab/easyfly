# easyfly_motion_prediction

项目维护的 C++17 匀速预测核心，迁自 easyfly_target_prediction。
仅依赖标准库，导出 `easyfly_motion_prediction::easyfly_motion_prediction`。
公开头 `easyfly_motion_prediction/predictor.hpp`，命名空间 `easyfly::motion_prediction`。

保留 Options、Observation、Frame、Prediction、Predictor 和原有参数范围/默认值。
调用者显式传入参考时钟与 steady 时钟的 int64 纳秒值；核心不读取系统/ROS 时钟，
不包含 ROS 消息、节点或 TF 类型。原有 odom、源时间和观测有效性约束保留。
两个时钟可以有不同原点，不能互相相减；输入的时间跨度须可由 int64 纳秒差值表示。

必须有两次严格有序的真实观测才产生有效速度；predict 不生成或刷新真实观测。
空帧、ID 消失、类别变化、跳变、超速、无效输入、过期、时钟暂停/回退的历史规则保持。
位置仍为 observation + velocity * (age + horizon)。variance 为简化模型，未校准为传感器协方差。

节点负责 ROS 消息转换、时钟采样、发布与能力服务；预测结果仍不隐式接入导航或飞控。
旧 easyfly_target_prediction/predictor.hpp 仅含新头与 using 别名。
