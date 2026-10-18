# easyfly_projection

普通 C++17 INTERFACE target `easyfly_projection::easyfly_projection`，仅使用标准库。
Box 保留原 float32 归一化 bbox，DepthView 同步借用 host-endian float32 深度字节和字节 stride；
调用者保证 buffer 存活且不并发修改。输入字节须满足原 float32 对齐前提；本包不保存视图。

medianDepth 保留原夹取、含端点自适应步长采样、严格 min/max 深度和有效比例、upper median；
无结果返回 -1。Options 默认 .3/9.5/.1，功能包继续显式传入正式 YAML .3/19.5/.1。
pinhole 接收整数像素、光学 Z(m) 和显式内参，返回光学 x右/y下/z前三维坐标；调用方提供有效有限内参。
不支持畸变校正：输入是原流程的 aligned depth 与对应像素坐标，不悄悄引入新相机模型。

编码/端序、轨迹与深度配对、CameraInfo/TF 接纳、目标消息/frame contract 和可视化仍在功能包。
原像素节点的 latest TF 回退保留，不等同于雷达相机融合的两源精确时间 TF 契约。
