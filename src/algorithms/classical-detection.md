# 椭圆与彩线检测

本次将 `easyfly_ellipse_detection`、`easyfly_color_line_detection` 两个预留包实现为可启动的
图像检测节点，计算代码分别放在独立自有算法包，未导入第三方源码。
OpenCV 是已有系统计算依赖，无模型下载或推理 SDK。

| 职责 | 路径 |
| --- | --- |
| 椭圆计算 | [easyfly_ellipse](detection/easyfly_ellipse/README.md) |
| 彩线计算 | [easyfly_color_line](detection/easyfly_color_line/README.md) |
| 椭圆 ROS 接入 | [easyfly_ellipse_detection](../perception/p2_detection/easyfly_ellipse_detection/README.md) |
| 彩线 ROS 接入 | [easyfly_color_line_detection](../perception/p2_detection/easyfly_color_line_detection/README.md) |
| 公共图像传输与时效 | [easyfly_image_detection_common](../perception/p2_detection/easyfly_image_detection_common/README.md) |

## 功能与接口

椭圆使用灰度/模糊/Canny/轮廓/fitEllipse，检查尺寸、轴比、拟合残差、面积及角度覆盖，
合并近重复边缘。输出像素中心、完整长短轴、长轴角和几何拟合质量。
支持 BGR8、RGB8 和 mono8。圆可检出，但圆的方向没有稳定意义；贴边截断目标被拒绝。

彩线使用可配置 HSV 范围、形态学及细长连通区域最小外接矩形。默认检测红色，
支持色相上下界跨 179→0，绿色等普通范围也可配置。输出像素中心、轴端点、无向角、
长度、宽度、面积及图像横向归一化偏移。支持 BGR8/RGB8，拒绝无颜色信息的 mono8。
当前能力为直线段，不包含曲线追踪、交叉口拓扑或跨帧线段关联。

新增 easyfly_msgs 的 `EllipseObservation`、`EllipseObservationArray`、`ColorLineObservation`、
`ColorLineObservationArray` 四种消息；已有消息字段不变。结果使用源图像 header，
`valid=true` 且空列表表示正常无目标；坏输入、错参考系、源时间异常、超时或计算失败则
`valid=false` 且清空观测。尺寸、坐标、角度及 score 语义见各消息注释和算法 README。

节点使用 depth=1 输入队列和同步计算，处理后复查新鲜度；双时钟过期检查可发现停流和
冻结仿真时钟。回退时先使旧结果失效，再允许新 epoch 图像恢复。重复/乱序帧不能续期。
默认最大年龄 0.5 秒、2073600 像素、最多 32 个结果。检测和传输参数只读，重启后应用修改。
公共 ROS 模板不放进纯算法库；算法公开头不引用 ROS、EasyFly 消息或飞控。

## 启动与复验

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to easyfly_ellipse_detection easyfly_color_line_detection
source install/setup.bash
ros2 launch easyfly_ellipse_detection detection.launch.py
# 另一个终端加载同样环境后：
ros2 launch easyfly_color_line_detection detection.launch.py
```

默认输入 `/camera/image_raw`，输出 `/ellipse_observations` 与 `/color_line_observations`。
能力服务分别为 `/platform/ellipse/capabilities` 与 `/platform/color_line/capabilities`。
launch 提供 `config_file`、`use_sim_time`、`input_topic`、`output_topic`。
capabilities.ready 只表示最近成功处理图像仍有效，不要求图中有目标。

隔离构建与两种安装复验，不覆盖工作区主 build/install：

```bash
/usr/bin/python3 -s tools/algorithms/verify_classical_detection.py \
  --output artifacts/algorithms/classical-detection/new-run
```

输出目录必须不存在。工具清理外部 ROS/Python overlay，仅加载系统 ROS；构建两个算法、
两个节点、共享 ROS 模板和消息共六包，运行算法/节点测试和五包全部已注册规范检查。
普通 CMake 消费者只使用两个已安装算法 target；复制安装整体搬迁后重新配置、编译和运行。
检查消费者没有 ROS 头/运行库或算法源码 include，复制安装公开头不是源码软链接。
复制安装搬迁验证针对算法 target，不承诺 colcon 生成的整套 ROS shell 脚本可搬迁。

## 验证边界与剩余工作

2026-10-18 的最终复验使用 `artifacts/algorithms/classical-detection/final-v3/`，
源码/配置/消息/验证工具指纹在整个矩阵期间保持一致，机器结果见
[classical-detection-results.json](baseline/classical-detection-results.json)。

| 验证 | 普通复制安装 | 符号链接安装 |
| --- | --- | --- |
| 六包干净构建 | 通过 | 通过 |
| 算法单元测试 | 10/10 | 10/10 |
| 真实 ROS 集成测试 | 7/7 | 7/7 |
| 已注册 CTest 入口（含规范检查） | 38/38 | 38/38 |
| 无 ROS overlay 的普通 CMake 消费 | 通过 | 通过 |
| 复制安装整体搬迁后消费 | 通过 | 不适用 |

每种安装的 colcon 原始汇总为 `115 tests, 0 errors, 0 failures, 9 skipped`。
其中 9 个 skipped 全部来自本机 cppcheck 2.7 被 ament 默认跳过，不能当作静态分析通过；
17 个功能用例均未跳过。其他已注册规范检查无失败。

算法测试覆盖旋转、尺寸/中心精度、多目标、去重、圆/环、非连续 ROI、输出数量限制、
矩形/截断/部分弧/小目标过滤、确定性噪声、HSV 红色边界与绿色范围、输入和参数拒绝。
ROS 测试通过已安装 launch/config 和独立节点进程验证几何输出、RGB/BGR/mono 接纳、
行尾 padding、错参考系、旧/未来/零时间、损坏 payload、步长/编码/尺寸错误、重复/乱序、
无目标、停流、仿真时钟暂停/回退及恢复、非法启动参数、绿色配置替换与干净退出。

开发期的首次纯 target 导出漏了 sensor_msgs 类型支持链接，及两份新 CMake 文件缺少
版权声明的问题均已修复；早期失败日志保留在 final-v1/final-v2，最终结果不混入这些尝试。
只重建六包依赖闭包，未重新构建整个工作区，未覆盖主 build/install。

合成图像用于验证几何、参数和传输契约，不能替代实景数据集
精度评估。真实相机的光照、背景纹理和 HSV 范围仍需实测；没有执行实机飞行或自动巡线。
两个节点仅发布图像几何信息，没有新增米制位姿、导航 Action 或飞控行为。

当前算法预留包剩 `easyfly_vehicle_reid`、`easyfly_depth_estimation`、`easyfly_swarm_planner`、
`easyfly_trajectory_optimizer` 四个。EGO 的已有内部轨迹优化能力保持不变。
ALG-9 的历史清单及验证结果保留，退出崩溃、DDS 偶发崩溃和旧规范欠账的未通过状态不变。
