# easyfly_color_line

项目维护的普通 C++17/OpenCV 彩色直线区域检测库，不依赖 ROS 运行库或 EasyFly 消息。

```cmake
find_package(easyfly_color_line REQUIRED)
target_link_libraries(app PRIVATE easyfly_color_line::easyfly_color_line)
```

公开入口 `easyfly_color_line/detector.hpp`，`easyfly::color_line::detect(image, parameters)`。
输入为 BGR8，可带行步长，不修改输入。空图返回空列表；非法类型/尺寸/参数抛出
`std::invalid_argument`；超过 4096 个轮廓抛出 `std::runtime_error`，最大 2073600 像素。

HSV 范围使用 OpenCV 标度：H 为 `[0,179]`，S/V 为 `[0,255]`。默认 H=170..10 检测红色，
下界大于上界表示跨越 179→0；普通范围如 50..70 可检测绿色。可配置形态学开闭核、
最小面积/长度/长宽比及矩形填充率。每个足够细长的连通区域输出一个最小外接矩形中心轴，
按区域面积降序排列并限制数量。孤立小噪声和近圆色块被过滤。

坐标/长度单位为像素，方向是图像 +x 朝 +y 的无向轴角 `[-π/2,π/2)`；start/end 没有行进
方向语义，必要时夹到图像边界。width/length 是拟合矩形尺寸。横向偏移为
`(center_x-(image_width-1)/2)/((image_width-1)/2)`，正值向右。score 为填充率，不是概率。

本实现识别彩色直线段，不推断交叉路口拓扑、曲线路径、多段断线关联或无人机控制指令。
光照、反光和相近背景色需要实际相机标定 HSV 范围。
ROS 接入见 `../../../perception/p2_detection/easyfly_color_line_detection/README.md`。
