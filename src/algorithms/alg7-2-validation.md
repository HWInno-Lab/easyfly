# ALG-7.2 当前帧障碍几何与动态关联迁移验收

日期：2026-10-18，迁移前提交 `4b70aea`，原生产基线 `bdbc34c`。
本批完成障碍几何子批次；下一步 ALG-7.3 占据投影、膨胀与持久化。

## 1. 实际结构和责任

新增 [easyfly_obstacle_geometry](fusion_mapping/easyfly_obstacle_geometry/README.md)，
导出 C++17 SHARED target `easyfly_obstacle_geometry::easyfly_obstacle_geometry`。
聚类使用系统 PCL/Eigen，动态跟踪头只依赖 STL；算法包无 ROS 运行依赖。
这是 EasyFly 自有实现及 PCL 调用包装，保存在 algorithms/fusion_mapping，未新增第三方载荷。

| 算法包 | semantic_map 功能层 |
| --- | --- |
| 两轮当前帧欧式聚类、动态观测 AABB、输出 AABB 与尺寸/碎片/数值门限 | 当前帧接纳、点云解析/体素化、状态/TF 时效 |
| Hungarian 几何关联、歧义/尺寸/方向/时间门限、EMA 速度及动态确认 | 源时钟/地图失效和 tracker reset、ROS 字段组装与唯一 ID 保护 |
| 有界休眠 ID、带不确定度增长的完整九点截面扫掠及预测内部预算 | 全局点数预算、预测 double→PCL float 转换、地图/服务/消息输出 |

旧 dynamic_obstacle_tracker.hpp 仅转发并为全部原公开类型/函数提供 using 别名；
旧 include 安装路径保留，支持旧调用方重新编译，不承诺旧二进制 ABI。
原34项动态测试随实现迁移；新增5项真实 PCL 聚类和数值边界测试。
地面算法包、旧地面兼容头、grid_inflation/observed_grid/map_validity 和正式 YAML/launch 未改变。
占据图消费预测截面并生成栅格的代码仍在节点，留待 ALG-7.3。

## 2. 等价性与原行为

动态头/原测试还原命名空间/include 后逐字一致。复验工具从保存的原节点独立提取真实
PCL 聚类及两条 AABB 路径，检查计算正文与新库在去除空白/注释后相同。
替换抽离调用点后，节点其余正文与原版相同；状态接纳、失效原因、全局预算和输出均保留。

两条 AABB 保持原有差异，不合并两轮聚类：动态观测使用 infinity 初值、先扩展 double 再加减；
输出使用 ±1e9 初值、先作 float 加减再扩展。原 max_obstacle_size、.05m 碎片门限、
过大/碎片跳过与非法几何整帧失效的处理顺序不变，原 float32 消息字段仍作为输出关联输入。
新增状态枚举只携带原分支结果，最终清图/跳过仍由节点执行。

tracker 同步消费显式源时间及当前帧几何，不查询时钟或持有点云。保留方向/净位移证据、
速度确认、动态框尺寸上限、EMA 和按 UAV 距离分配完整九点截面的预算。
空帧不输出历史障碍/预测并清除速度确认，允许原有有限短期休眠 ID；时间回退 reset。
没有引入历史点云拼接、算法阈值增强或退出逻辑修改。

## 3. 对照与实际验证

证据：`artifacts/algorithms/alg7-2-20261018/`，最终矩阵 `matrix-v1/`。
受追踪摘要：[alg7-2-results.json](baseline/alg7-2-results.json)。迁移前保存33份功能包文件，
保存并前后核对 ALG-7.1 安装节点库 SHA；没有重建或覆盖该原二进制。

原版独立探针使用保存的动态头和从原节点提取的聚类/AABB；新版从安装 target 消费。
每种安装对照80帧真实 PCL 点云和680步 tracker 状态（600步合成几何 + 80步聚类观测），
覆盖移动/静止/歧义/超大框、丢帧、反向、显式 reset、时钟回退和紧预算。
精确比较全部观测、包围盒/状态/float 字段、ID/速度、诊断计数、输出匹配、包络及每个预测点。
两种安装均逐字节一致，trace SHA-256：
`cb2e7cd5b1da909d248018357d87ce11a2699e37b1837be8161c6e48519200ff`。

新增实际节点测试先在迁移前二进制通过，再在两种新安装通过：
未解锁初始化后静止框不误报动态，持续移动形成稳定 ID/正向速度与完整九点扫掠，
地面空帧无历史框/预测，静止重现不继承旧动态确认。要求本测试节点正常退出，强制清理不算通过。
原实际地面生命周期、房间/U型墙、两种 footprint/高度边界和分类节点回归继续通过。

| 每种安装方式 | 用例/消费检查数 |
| --- | --- |
| 原动态跟踪 | 34 |
| 新聚类/PCL/数值边界 | 5 |
| 原地图有效性 / 膨胀 / observed-cells | 13 |
| 原分类消息和实际分类节点 | 6 |
| 原房间/footprint/高度边界实际节点 | 7 |
| 地面生命周期与新移动障碍实际节点 | 2 |
| 安装旧头 / 无 ROS 独立 CMake 消费 | 2 |

每种69次，合计138次用例/消费检查；按 CTest 注册项计24次，全部通过、无跳过。
旧二进制实际动态检查另计1次；差分轨迹不混为单元测试条数。
地面模块源码无改动，本批复用 ALG-7.1 地面安装并跑其节点回归，没有重计其36项核心测试。

普通复制安装根搬迁后重新配置、构建及运行纯消费者，库从搬迁后的 lib 加载。
旧头在支持的 ROS 环境消费；纯核心在清理环境中独立 find_package，不借用 ROS overlay。
编译命令无源码 include，ldd 无 ROS 动态库或缺失库；普通安装头不是源码软链接。
独立消费者以 -Wall/-Wextra/-Wpedantic/-Werror 编译。Python 语法、XML、git diff --check 通过；
未将完整包 lint 或 shutdown/stress/capture 注册项记为通过。

小规模性能检查：同机预热后交错运行原/新完整探针各7次，中位墙钟均 .07s，
最大 RSS 中位31840/32000KiB。GNU time 分辨率 .01s，包含启动和输出格式化；
这只是小点云轨迹的粗粒度检查，不代表生产延迟或真实传感器资源认证。
抽离通过只读引用/ConstPtr 同步调用，没有增加整帧点云复制、历史存储或新线程。

矩阵期间46份源文件指纹稳定；包数89→90，核心源码依赖闭包仅自身一包。
ALG-1..5 的13/36/69/29/146份保护源码均不变；ALG-7.1 的53份中48份不变，
4份为计划内 semantic_map 修改，1份动态测试迁入新包。ALG-6 的81份中67份不变，
其余14份是本批和上批的 semantic_map 改动/测试迁移。ALG-0 冻结465份中381不变、
70修改、14迁出/删除；原历史清单不改写。

## 4. 复验与范围

保留原文件/节点二进制 baseline，在独立 symlink prefix 构建新核心和 semantic_map 后执行：

```bash
python3 tools/algorithms/verify_obstacle_install.py \
  --symlink-prefix artifacts/algorithms/alg7-2-20261018/initial/install \
  --baseline artifacts/algorithms/alg7-2-20261018 \
  --output artifacts/algorithms/alg7-2-new-run
```

工具拒绝覆盖旧证据并检查原文件/二进制指纹。使用已安装 ROS、主安装、scene-s3、ALG-6、
ALG-7.1 underlay，不是全工作区 clean rebuild。纯 CMake 使用 LANGUAGES C CXX 以支持
系统 PCL/VTK 的 C/MPI 导出。目录搬迁覆盖 CMake 导出/原生加载，不保证所有 colcon setup 可搬迁。

ROS 测试在非零隔离域串行使用合成点云/状态/TF，不启动 PX4/Gazebo 或发送飞行任务，
不覆盖主 build/install、不停止用户仿真。ALG0-B02/B03 与历史 semantic 退出问题继续保留未解决。
