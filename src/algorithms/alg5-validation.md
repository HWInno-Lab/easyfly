# ALG-5 检测、标志位姿与单目标跟踪迁移验收

日期：2026-10-18，迁移前提交 `9b441e2`，原生产基线 `bdbc34c`。
本批完成五个算法包、三个功能包接入和 ONNX Runtime 分类；下一阶段为 ALG-6。

## 1. 实际结构与责任

| 包/目录 | 职责 |
| --- | --- |
| detection/easyfly_detection_api | 普通检测参数、像素框、调用契约和分段 timing |
| detection/easyfly_yolo_onnx | 自有 YOLO/ONNX 包装、letterbox/NMS、实际 provider；私有 SDK/PIMPL |
| detection/easyfly_tag_pose | 自有 OpenCV 36h11/SUBPIX/IPPE-square 包装 |
| tracking/easyfly_single_tracking_api | 单目标 initialize/update/reset 和 optional 校准置信度 |
| tracking/easyfly_csrt | 自有 OpenCV CSRT 生命周期包装及原 ROI/pixels 换算 |
| third_party/runtimes/onnxruntime | 原本地第三方 GPU SDK，载荷继续忽略，集成说明受追踪 |

五包导出 namespaced C++17 target，四个 INTERFACE、YOLO 一个 SHARED；API 不依赖具体实现。
OpenCV/ONNX Runtime 完成实际计算，不能将外部库实现记为自有算法源码。
算法公开类型不含 ROS、话题、QoS 或设备对象，YOLO 不导出 SDK 头/类型/本地 include 路径。

功能包保留图像消息/转换缓冲所有权、最新帧 worker、generation、校准接纳、源时间/接收时间、
过期、时钟暂停/回退、消息与能力服务。检测参数已与节点话题/QoS/性能参数分开。
旧头仅转发/别名；原节点名、话题、ROI/重定位服务、结果原因及 unknown confidence=0 保留。
CSRT 模型已从 ROS worker 抽出，算法结果以 nullopt 表示没有校准分数。
公开旧头供重新编译消费，不承诺旧共享库 ABI。

## 2. 等价、契约增强与部署

迁移前直接编译保留的原检测实现，分别实际运行 CPU 和 CUDA，各检出 5 个目标。
两种安装的新后端均对同一模型/图像、同一实际 provider 比较类别 ID/名称、顺序、置信度和像素框：
置信度绝对容差 1e-5，像素坐标容差 1e-3；caller timing 四字段哨兵保持。
CPU 与 CUDA 各对自身基线比较，不要求不同 provider 之间浮点结果相同。
保留 ORT intra-op threads=2、CUDA 失败回退 CPU 规则并报告实际 provider。

原版和新包装另对照 8 个姿态样本、真实 36h11 检测角点、12 帧 CSRT 平移/丢失/重初始化、
ROI 换算。输出逐字节一致，SHA-256：
`41b2d2b2f6aca3013a1446bed379964eb67d28ae70def95b747386c30a3f7069`。
两份原 helper 正文经命名空间/空白/注释归一后保持不变，新包装追加在其后。

新增检测参数有限值/范围及非空 BGR8 检查属于单独的契约增强，不能将非法输入拒绝归为等价移动。
正常输入的模型计算、NMS 和坐标语义保持。迁移和该检查分开提交。

默认权重仍是原 models/yolo11n.onnx，由功能包安装到 share 下并解析 package:// 路径；
显式绝对/普通相对路径保持原规则。模型/固定输入 SHA 保存在结果摘要。
SDK 的 32 个载荷文件/链接与迁移前一致，只有本项目集成说明更新。
本地版本 1.23.2 GPU、commit 与许可来自 SDK 元数据；原下载归档 hash 未保存。

仅 YOLO 实现包安装运行库、provider、实际 SONAME 别名及 LICENSE/ThirdPartyNotices 等说明。
安装 RPATH 为 $ORIGIN。普通安装显式通过外部 ORT_ROOT 别名目录选择同一已审计 SDK，
验证覆盖路径覆盖机制，不代表兼容所有 SDK 版本。TensorRT provider 文件不代表该检测后端已实现。
CUDA/cuDNN/驱动仍由主机提供。

## 3. 实际验证

证据根：`artifacts/algorithms/alg5-20261018/`，最终矩阵 `matrix-v1/`。
受追踪完整摘要：[alg5-results.json](baseline/alg5-results.json)。矩阵期间 146 个源文件指纹稳定。

| 每种安装方式 | 用例数 | 内容 |
| --- | --- | --- |
| 检测 API / YOLO / 标志 / CSRT 核心 | 12 | timing、参数/图像拒绝、NMS/letterbox、实际 PnP、歧义/重投影、CSRT/reset |
| 检测节点与原 contract | 7 | 安装模型的真实 CPU/CUDA 推理、header/坐标/类别；5 个原源码 contract |
| 标志节点 | 16 | 原 CameraInfo/校准、重复、状态和时钟回归 |
| 单目标节点 | 2 | ROI、身份/平移、reset/丢失、重定位与时钟回归 |
| 原降落检测入口 | 7 | 真实标签→降落检测、消息时序及原状态回归 |
| 精准降落观测侧 | 92 | 原配置/几何/标签重放、观测、stream pairing、frame timing |
| 安装旧头消费 / 独立纯 CMake 消费 | 2 | 三功能包旧 include 与五个纯核心 target，实际算法调用 |

每种安装 138 次用例/消费检查，合计 276 次，按 CTest 注册项计 48 次，全部通过、无跳过。
原版/新版视觉和两种 provider 差分另有运行日志，不混入上述用例数。
精准降落只选表内六项测试注册，未运行 simulation/飞行任务测试，也未将旧功能包全包 lint 记为通过。
Python 语法与 git diff --check 通过；独立消费目标以 -Wall/-Wextra/-Wpedantic/-Werror 编译。

普通复制安装根搬迁后，在清理 ROS 环境、只提供五核心 prefix 的条件下重新配置、构建和运行。
编译命令不含源码 include；YOLO/ORT 运行库均从搬迁后的安装 lib 解析，RPATH 不含源码路径。
搬迁后再次实际 CPU/CUDA 推理，并验证模型和全部安装运行库字节相同。
symlink-install 允许安装头链接源码；普通安装头不是源码软链接。
旧功能包保留 ROS CMake 元数据，旧头消费在明确 ROS 构建环境验证；不承诺其脱离 ROS 配置。

发现包数 80→85。五个纯核心源码依赖闭包仅自身五包。
ALG-1/2/3/4 的 13/36/69/29 个受保护源码指纹不变。
ALG-0 冻结 465 文件中 403 个不变、53 个修改、9 个迁出/删除；历史清单不改写。

首轮新增检测测试曾将节点的字符串覆盖参数传为 bool，并在失败后重复清理进程；测试夹具已修正，
失败日志保留，未放宽结果阈值、时限或生产断言。首轮/重试不混入最终矩阵统计。

## 4. 复验与边界

在独立 symlink prefix 构建五核心和三个视觉功能包后运行（工具补建两个降落消费包）：

```bash
python3 tools/algorithms/verify_vision_install.py \
  --symlink-prefix artifacts/algorithms/alg5-20261018/initial/install \
  --baseline artifacts/algorithms/alg5-20261018 \
  --output artifacts/algorithms/alg5-new-run
```

baseline 目录需保留 input-snapshot.json、legacy-detection.json 和原版 legacy/src 头文件/源码。
工具拒绝覆盖旧证据；consumer 是工具复制并补齐 probe 源码后使用的验证夹具。
使用 ROS、主安装及 scene-s3 已安装 underlay，不等同于全工作区 clean rebuild。
搬迁验证覆盖 CMake 导出、原生动态加载和模型资产，不承诺所有 colcon shell setup 可重定位。

所有 ROS 回归在非零隔离 DDS 域运行，使用合成图像/标定/时钟/观测。
真实本机 CPU/CUDA 推理不等于传感器精度或实际飞行验收。没有覆盖主 build/install、停止用户仿真、
启动 PX4/Gazebo 或发送飞行指令。ALG0-B02/B03 和既有 semantic 退出问题仍未修复或重报通过。
