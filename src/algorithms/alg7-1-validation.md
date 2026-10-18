# ALG-7.1 地面计算与可信平面迁移验收

日期：2026-10-18，迁移前提交 `644183b`，原生产基线 `bdbc34c`。
本批完成地面子批次；下一步为 ALG-7.2 当前帧障碍几何/动态关联，不提前移动占据图。

## 1. 实际结构与责任

新增 fusion_mapping/easyfly_ground_segmentation，导出 C++17 SHARED target
`easyfly_ground_segmentation::easyfly_ground_segmentation`，依赖系统 Eigen3/PCL，无 ROS 运行依赖。
包内包含原四个辅助头及从节点实际抽出的 PCL RANSAC 拟合流程，不只迁移 ROI/候选筛选函数。

| 算法包 | 原 semantic_map 功能层 |
| --- | --- |
| below-vehicle 候选筛选、原/严格两次 RANSAC、平面归一化、空间支撑 | 完整当前帧、点云解析/预算、车辆状态与 steady 年龄采样 |
| GroundPlaneTracker 连续初始化、冻结、创新门限、历史与 reset | 状态/位姿接纳、TF 查询与时效、参考系跳变后的 reset |
| 平面方程运输、变换连续性和显式时间数字判断 | 地图失效/版本、低障碍分类、动态关联、占据图、发布和服务 |

fitGroundCandidate 同步只读借用完整地面参考系 PCL XYZI 点云和显式车辆/参数证据；
返回候选及原诊断计数，不修改原点云，不把候选内点作为障碍删除依据。
has_candidate 不表示模型可信；节点仍调用 tracker，再使用可信平面分类当前帧。
具体所有权、单位和合法输入前提见 [包说明](fusion_mapping/easyfly_ground_segmentation/README.md)。

旧四头仅转发/using 别名，保留原传递包含关系，支持旧路径重新编译；不承诺旧二进制 ABI。
原四组测试随实现迁移，新增三项实际拟合测试；剩余 observed_grid 通过 target 消费同一个平面类型。
功能包保留既有头安装位置并补齐导出的 include/依赖元数据，供安装后旧头消费。

## 2. 等价与不变行为

四份辅助头和原四份测试在还原命名空间/include 后逐字相同。
RANSAC 运算与调用顺序保持，包括原系数归一化和严格候选的 float 精度运算。
还原新数值调用点后，节点剩余正文经空白/注释归一后与原版一致。
原 dynamic_obstacle_tracker、grid_inflation、observed_grid、map_validity 四头字节不变。

完整帧分母保持：地面参考系点云只是原完整点云逐点变换，点数不变；不以筛选子集作分母。
保留不少于100点才拟合、原 .12m/200次和严格 .02m/300次拟合、原倾角/净空/连续帧规则。
空间支撑默认 false，正式 YAML 仍显式 true；阈值、配置与场景选择规则均未修改。
保留1–3m环带、.2m独立格、200点/120格/90%及八方向各5格、低处异常格否决和至少五帧初始化。

可信模型只在原状态规则下更新；已解锁冻结、空中重启拒绝、低障碍不被候选错误删除。
map/odom 平面运输不拟合新地面；原节点仍查询/接纳 TF，缺失/过期使地图失效，
相邻变换超过 .25m/.10rad 会 reset，已解锁时不能重建。数学函数不读取 ROS/steady 时钟。
原失效地图的状态转换合并、线程/队列与退出逻辑均未改，没有混入新算法增强。

## 3. 对照与实际验证

证据根：artifacts/algorithms/alg7-1-20261018/，最终矩阵 matrix-v2/。
受追踪摘要：[alg7-1-results.json](baseline/alg7-1-results.json)。保存了36份原功能包文件，
并保存/核对迁移前 ALG-6 安装节点库的 SHA；最终矩阵前后该旧二进制未被覆盖。

从保存的原节点提取实际拟合代码，与原四头独立编译；新版本从安装 target 编译消费。
32帧真实 PCL 拟合覆盖高占比地面、稀疏地面/严格空间支撑、单侧支持、低障碍与空帧；
另有240步连续模型状态输入，覆盖连接/位姿/年龄、armed、异常候选、初始化/冻结/reset。
比较候选法向/offset、分母/内点/全部空间支撑字段、状态、可信模型、低障碍判断与平面变换。
两种安装输出均与原版逐字节一致，trace SHA-256：
`5c8305ad5c12660c0012adecb7eb40456addd179d5d2f395f4ce18ade68f5745`。

新增实际节点测试先对旧二进制通过，再对两种新安装通过：空中启动拒绝、未解锁初始化、
已解锁时拒绝 .3m 假候选对低障碍分类的影响、参考系跳变失效且不能空中重建、
未解锁恢复，以及只有地面 TF 过期时失效。要求自身节点正常退出，强制清理不能算通过。
没有用这些特定正常退出场景宣布旧 shutdown/stress 问题已修复。

| 每种安装方式 | 用例/消费检查数 |
| --- | --- |
| 原 tracker / 空间支撑 / 变换 / 候选筛选 | 33 |
| 新实际 PCL 拟合测试 | 3 |
| 原地图有效性 / 膨胀 / observed-cells / 动态障碍纯计算 | 47 |
| 原分类消息/实际分类节点 | 6 |
| 原实际房间/两种 footprint/高度边界节点回归 | 7 |
| 新实际地面生命周期 | 1 |
| 安装旧头 / 无 ROS 独立 C++ 消费 | 2 |

每种安装99次，合计198次用例/消费检查，按 CTest 注册项计30次，全部通过且无跳过。
旧二进制地面生命周期另有1次基线检查；拟合/模型对照也单列，不混为198条单元测试。
普通复制安装根搬迁后重新配置、构建和运行消费者，新地面库从搬迁后的 lib 解析。
编译命令无源码 include，ldd 无 ROS 动态库；旧功能包头在 ROS 构建环境消费，纯核心单独清理环境验证。
独立消费目标以 -Wall/-Wextra/-Wpedantic/-Werror 编译。普通安装头不为源码软链接。
Python 语法、包 XML 和 git diff --check 通过；未把原全包 lint 记为通过。

最终矩阵期间53份源文件指纹稳定。发现包数88→89，纯核心源码依赖闭包仅自身一包。
ALG-1..5 的13/36/69/29/146份受保护源码均未改变。
ALG-6 的81份中69份不变，12份为本批计划内的 semantic_map 改动或原测试迁移。
ALG-0 冻结465文件中383不变、69修改、13迁出/删除；原历史清单不改写。

初步新 ROS 夹具曾假定重复失效地图会反复发布，并把首次失效之前的在途正常地图计入失效窗口；
已按原状态转换合并和实际收到失效消息后的窗口检查，继续要求有新输入且无后续 ready 地图。
matrix-v1 因该夹具窗口假设失败，日志保留，未修改生产行为、时限或算法结果容差。
原版探针先尝试未版本化 pkg-config 名称失败，最终用系统 PCL 官方 CMake config 构建旧/新消费者。

## 4. 复验与范围

先在独立 symlink prefix 构建新核心与 semantic_map，保留原文件/旧二进制 baseline，然后运行：

```bash
python3 tools/algorithms/verify_ground_install.py \
  --symlink-prefix artifacts/algorithms/alg7-1-20261018/initial/install \
  --baseline artifacts/algorithms/alg7-1-20261018 \
  --output artifacts/algorithms/alg7-1-new-run
```

工具拒绝覆盖旧证据，核对原文件和旧节点库指纹。使用已安装 ROS、主安装、scene-s3、ALG-6 underlay，
不等同于全工作区 clean rebuild。纯 CMake 声明 C/CXX 以支持系统 PCL/VTK 的 C/MPI 导出；
实际头/库来自 /usr 的 PCL/Eigen，没有额外复制 SDK 或改动第三方载荷。
搬迁覆盖 CMake 导出/原生加载，不承诺所有 colcon shell setup 可重定位。

ROS 回归在非零隔离 DDS 域，用合成点云、TF 和车辆状态；不启动 PX4/Gazebo、不发送飞行任务，
不覆盖主 build/install、不停止用户仿真，也不认证实际着地、地面精度或飞行安全。
ALG0-B02/B03 和既有 semantic 退出问题保留未解决；shutdown/stress/capture 注册项在本矩阵明确排除。
