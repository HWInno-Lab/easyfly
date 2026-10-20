# easyfly_ellipse

项目维护的普通 C++17/OpenCV 椭圆检测库，不依赖 ROS 运行库或 EasyFly 消息。

```cmake
find_package(easyfly_ellipse REQUIRED)
target_link_libraries(app PRIVATE easyfly_ellipse::easyfly_ellipse)
```

公开入口 `easyfly_ellipse/detector.hpp`，`easyfly::ellipse::detect(image, parameters)`。
接收 BGR8 或灰度图，允许带行步长的 ROI；不修改输入。空图返回空列表，错误类型、
超出 2073600 像素或非法参数抛出 `std::invalid_argument`；超过 4096 个轮廓抛出
`std::runtime_error`。参数在每次调用时检查，不维护跨帧状态。

流程为灰度化、5×5 高斯模糊、Canny、轮廓、OpenCV fitEllipse，按短轴、长短轴比、
归一化径向 RMS 残差、面积一致性及 16 扇区覆盖筛选，再合并同一边缘的近重复拟合。
输出按拟合分数排序并限制数量。长短轴为完整直径，单位像素；方向为长轴沿图像 +x
朝 +y 的弧度角 `[0,π)`。圆的方向没有稳定意义。score 是几何拟合质量，不是分类概率。

只支持基本完整且边缘清晰的椭圆/圆形轮廓；刻意拒绝贴边截断目标，不保证严重遮挡、
复杂纹理或弱光场景精度。轮廓不代表特定物体类别，也不提供米制位置或姿态。
ROS 接入见 `../../../perception/p2_detection/easyfly_ellipse_detection/README.md`。
