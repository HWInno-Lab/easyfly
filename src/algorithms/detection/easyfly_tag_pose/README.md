# easyfly_tag_pose

EasyFly 自有标志位姿包装，C++17 INTERFACE target `easyfly_tag_pose::easyfly_tag_pose`。
实际字典检测、角点细化和 PnP 由系统 OpenCV aruco/calib3d 实现；不复制第三方源码。

`detect(gray)` 同步借用灰度8位图，返回拥有 ids/corners 的结果，固定 36h11 和 3 像素 SUBPIX。
`estimate(corners,K,D,size,max_error,ambiguity_margin)` 借用四角点与 CV_64F 内参，
返回光学相机系平移 m、单位四元数 xyzw、RMS 重投影误差 px、valid/reason。
保留原 IPPE_SQUARE 候选、正深度、重投影和歧义拒绝；无效结果的 t/q 不可读取。
尺寸须显式传入，0 表示未配置。OpenCV 异常传播，由适配器处理。
函数无跨帧状态，不持有图像或读取时钟；调用者保证输入在同步调用期间存活。

CameraInfo 接纳、校准 revision、worker/generation、图像消息所有权、源时间、暂停/回退和结果过期继续留在 ROS 功能包。
旧 tag_core.hpp 仅为命名空间别名。算法只输出计算结果，不发布话题、不改变飞行策略。
