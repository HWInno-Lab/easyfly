# easyfly_projection_py

仅依赖 Python 标准库的计算模块 `easyfly_projection_py.core`；ament 只负责构建/安装。
显式标量/tuple 的 camera_model 和 rigid_transform 完成原 K/plumb_bob 校验、四元数与旋转计算；
ImageView 是普通图像描述，不含 header/ROS 对象。k 必须有9元素，d 必须有5元素；点已由调用方解码并过滤。

project 同步借用图像字节/点/相机模型和显式四元数/平移 TransformView，返回 [(source_xyz,(r,g,b))] 与 row-major 光学 Z(m)。
同像素保留最近 Z，等深保留首点；未覆盖深度为 NaN。原 XYZ 不被相机系坐标覆盖。
图像支持 mono8/rgb8/bgr8 与行 padding；保留原像素预算、畸变公式、rounding 和正深度门限。
不保存跨帧状态、不读取时钟、不做 TF 查询；调用者在同步调用期间拥有输入且不并发修改。

ROS PointCloud2 字段/端序/行布局解析、RGB/32FC1 序列化仍在原 projection.py 适配器，旧函数入口继续有效。
节点继续保持 lookup_transform_full 的 image/cloud 两源时间、固定 odom、epoch/worker/校准与发布前过期检查。
纯模块可从安装 site-packages 在无 ROS 环境导入，普通安装搬迁后仍可使用。
