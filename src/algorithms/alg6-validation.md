# ALG-6 投影与障碍类别关联迁移验收

日期：2026-10-18，迁移前提交 `ecaa432`，原生产基线 `bdbc34c`。
本批完成三个计算包与原功能包接入。下一步为 ALG-7.1 地面候选/可信平面状态拆分。

## 1. 实际结构

| 包 | 内容 | 原功能层保留 |
| --- | --- | --- |
| easyfly_projection | 标准库 C++17，中位深度、针孔反投影、借用 DepthView | 编码/端序、深度/轨迹配对、CameraInfo/TF、目标消息/frame contract、标记 |
| easyfly_projection_py | Python 标准库，普通图像/变换描述、K/plumb_bob、投影、遮挡 Z-buffer | ROS cloud 字段/行/端序解析、输出序列化、两源时间 TF、worker/epoch/时效 |
| easyfly_obstacle_association | Python 标准库，expanded-AABB 双向唯一匹配、普通结果索引 | header/geometry/target 快照接纳、独立几何深拷贝、类别消息与 TTL |

C++ 导出 easyfly_projection::easyfly_projection INTERFACE target；Python 导出普通同名 core 模块。
没有新建空 API 包，没有反向依赖 ROS 功能包。分类结果不写回原障碍、占据图或飞行输入。
三包均有实际实现、安装与无 ROS 消费验证；Python 使用 ament_cmake_python 安装，不伪装为 C++ target。

图像/深度缓冲由调用方持有，计算同步借用，不保存历史或读取时钟。
分类接收已接纳的有限几何、唯一目标 ID 和类别快照，只返回匹配关系，不把无关联解释为几何无效。
详细单位、输入前提和所有权见各包 README。

## 2. 计算与平台边界

C++ 保留 float32 bbox 原算术、clamp、含端点自适应采样、严格 min/max 深度、有效比例与 upper median。
原结构默认 max_depth=9.5m，正式 YAML=19.5m，节点继续显式传入，未改配置。
pinhole 返回光学 x右/y下/z前，保留整数中心像素和原公式；仍假设 aligned depth 与有效内参，未新增畸变校正。

Python 投影保留 plumb_bob 五系数、原像素 rounding、正深度、最近 Z、等深首点。
结果带原 source XYZ 与 RGB，深度为相机光学 Z(m)，未覆盖处 NaN；适配层再输出原字节格式/header。
PointCloud2 解析 AST 与原版一致，融合 node.py 完全未改：仍以图像/点云各自时间调用
lookup_transform_full，经固定 odom，缺 TF 不回退 latest；worker、epoch、校准和发布前过期检查不变。

像素定位节点原本存在 latest TF 回退，本轮保留。这是既有平台行为，不能把雷达相机融合的两源时间契约
推广为像素节点已经拥有的能力。恢复新数值调用点后，该节点其余正文与原版相同。

分类计数包括所有候选；一个框本身有多个候选时，仍占用相关目标，双向不唯一就拒绝关联。
原 core.py 的 odom/header/源时间/预算/重复 ID 校验、完整障碍复制、原因字符串、observed_at 和 TTL
组装保持原样，原 ObstacleClassifier 节点和 semantic_map_node.cpp 未改。

C++ 空/短/非法 stride 视图和非有限/不可表示整数的框新增显式拒绝；memcpy 支持非对齐读取。
像素适配层新增 host-endian 与尺寸可表示检查。这些边界增强与等价迁移分开提交。
没有据此宣称全面重做像素节点的内参、bbox、时间或 TF 接纳。

## 3. 原版对照

迁移前保存 55 个原功能包文件及指纹，来自 ecaa432。

| 对照输入 | 数量 | 比较内容 |
| --- | --- | --- |
| 原 C++ 私有函数直接抽取编译 / 安装后新核心 | 240 | 随机深度/NaN/Inf、padding、框、有效比例、中位深度及光学三维坐标，输出逐字节一致 |
| 原 Python 投影 / 安装后适配层 | 240 | 原 RGB/深度字节与数量、NaN、畸变、变换、遮挡、错误类型/原因 |
| 原完整分类函数 / 安装后适配层 | 320 | 全部 ROS 消息字段、几何/速度/header/类别/TTL/原因、无效与歧义；输入不被修改 |

800 组唯一输入在两种安装中各重复一次，共 1,600 次对照，全部一致。
C++ trace SHA-256：`a0e919c9b7238cc13bbcdc2b8e0a5ec6786497f1071c906c6d520a27f522c2ed`。
Python 投影 trace：`d3a55048cfbe858ac5fcbccb422ae62d1b9d2433f0bcf49fa52dd086be8f4b71`。
完整分类字段 trace：`7cf525edfe9bc1e0d0ac63168fb36e0feec5bd07f90053007774f308f36843f4`。

分类既比较原生消息 ==，也比较全部字段的规范 JSON；不将未指定的 CDR 对齐 padding 当作消息契约。
视图/消息合法域由各包文档和原适配器约束；对照不表示任意畸形指针/内参都已获支持。

## 4. 实际验证

证据根：artifacts/algorithms/alg6-20261018/，最终矩阵 matrix-v2/。
受追踪摘要：[alg6-results.json](baseline/alg6-results.json)。最终矩阵 81 个源文件指纹稳定。

| 每种安装方式 | 用例/消费检查数 |
| --- | --- |
| C++ 投影、Python 投影、双向关联核心 | 13 |
| 原 pixel frame contract | 2 |
| 原融合投影、pending/校准失效、实际节点与时钟回归 | 9 |
| 原分类消息与实际节点 TTL/几何独立性 | 6 |
| 原跟踪→三维定位→ID 回流/重置 | 1 |
| 无 ROS C++ / Python 已安装模块消费 | 2 |

每种安装 33 次，合计 66 次，全部通过且无跳过。按进程入口另计为 22 个 CTest 注册调用、
4 次直接 Python 检查调用；800 组差分重复运行另计，不混为 66 条单元测试。

普通安装根搬迁后重新配置、构建、运行 C++ 消费者；编译命令无源码 include，ldd 无 ROS 运行库。
Python 在清理环境、python -I 下，仅由显式安装 prefix 发现 site-packages/dist-packages 后导入，
验证实际模块路径与无 ROS 导入；搬迁后同样通过。普通安装的 .hpp/.py 不是源码软链接。
独立 C++ 消费目标使用 -Wall/-Wextra/-Wpedantic/-Werror；Python 语法和 git diff --check 通过。
没有将旧功能包全包 lint、建图或 shutdown/stress 回归记为本批通过。

发现包数 85→88，三个纯核心源码依赖闭包仅自身三包。
ALG-1/2/3/4/5 的 13/36/69/29/146 个受保护源码指纹均未变化。
ALG-0 冻结 465 文件中 394 不变、62 修改、9 迁出/删除；历史清单保持原样。

初次 C++ 构建有字节指针类型遗留，已修正；初步分类差分曾直接比较 CDR padding，现以完整字段比较；
matrix-v1 在 Python 消费夹具只搜索 site-packages 时失败，已支持实际 dist-packages 路径。
历史日志均保留，初步运行不混入最终矩阵；没有放宽算法输出阈值、时限或原 ROS 测试断言。

## 5. 复验与范围

先在独立 symlink prefix 构建三个核心和三个功能包，保留原版 baseline 后运行：

```bash
python3 tools/algorithms/verify_fusion_install.py \
  --symlink-prefix artifacts/algorithms/alg6-20261018/initial/install \
  --baseline artifacts/algorithms/alg6-20261018 \
  --output artifacts/algorithms/alg6-new-run
```

baseline 需含 input-snapshot.json 和 legacy/src 原文件；工具核对原指纹并拒绝覆盖旧证据。
使用已安装 ROS、主安装、scene-s3 和 ALG-2 underlay；跟踪反馈使用先前验证的 ALG-2 节点与本批像素节点。
不等同于全工作区 clean rebuild。搬迁范围是 CMake 导出与显式 Python 模块路径，不承诺全部 shell setup。

ROS 测试使用非零隔离 DDS 域、合成相机/点云/标定/时钟/TF，没有覆盖主 build/install、停止用户仿真、
启动 PX4/Gazebo 或发送飞行任务。未认证实际传感器精度或飞行能力。
ALG0-B02/B03 和既有 semantic 退出问题仍保持未解决，相关 shutdown/stress 用例未重跑。
