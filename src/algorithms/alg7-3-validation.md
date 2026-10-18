# ALG-7.3 占据投影、膨胀与持久化迁移验收

日期：2026-10-18，迁移前提交 `6781277`，原生产基线 `bdbc34c`。
本批完成 ALG-7 最后一个子批次；下一步 ALG-8.1 第三方 FAST-LIO 分类迁移。

## 1. 实际结构和责任

新增 [easyfly_occupancy_mapping](fusion_mapping/easyfly_occupancy_mapping/README.md)，
导出 C++17 header-only INTERFACE target `easyfly_occupancy_mapping::easyfly_occupancy_mapping`。
公开实现仅依赖 STL，不导出 ROS、PCL 或地面算法包依赖。

| 算法包 | semantic_map 功能层 |
| --- | --- |
| GridGeometry、cluster-boxes 膨胀及裁剪 | ROS 栅格元数据/数据容器、消息发布 |
| observed-cells 差分投影、地面以上高度/体素交集筛选 | 当前帧、地面/TF/车辆时效、有界层能力接纳 |
| 完整预测截面转 AABB、GridPersistence 保持/释放与 reset | 动态语义选择、完整预测尾段接纳、失效时调用 reset |
| 原0.70m默认/最小膨胀及归一化 | 地图有效性/源时间/版本、失效未知图、QoS、服务 |

grid_inflation.hpp 迁入算法包，旧头仅 using；observed_grid.hpp 的计算正文迁入模板，
直接消费调用方地面类型的 normal/valid()/zAt()，复用原地面计算，不复制地面实现。
无地面重载只支持无界投影；有界层缺地面仍返回 false。
旧 observed_grid 头是薄转发，保留原具体 GroundPlaneModel 指针签名、默认值与传递包含。
安装旧头消费者额外按原函数指针类型编译调用；兼容范围是源码重新编译，不承诺旧二进制 ABI。

原2项膨胀、9项 observed-cells 测试随实现迁移，新增6项数值投影/预测尾段/持久化边界测试。
原有界投影测试继续使用真正的 GroundPlaneModel，因此 package.xml 仅以 test_depend 声明地面包。
源码 colcon 闭包含 occupancy 和该测试依赖；导出的 target 及纯消费者不需要地面包/PCL。
地面/障碍算法包、旧地面/动态兼容头、map_validity.hpp、正式 YAML/launch 均未改变。

## 2. 等价性与原规则

膨胀头还原命名空间后逐字相同，observed 投影函数正文逐字相同，原测试还原 include/
命名空间/地面类型后逐字相同。复验工具从保存的原节点独立提取真实包围盒投影、持久化、
预测足迹转框代码，核对新实现去除空白/注释后的计算顺序；恢复调用点后节点其余正文相同。

保留 floor/整数裁剪和原 inclusive 上边界、全点方形投影及差分数组资源上限，
不合并 cluster-boxes/observed-cells 语义。动态预测按完整截面转框，Z 仍取每段首点。
有界层保留倾斜地面和体素交集的保守高度筛选，完整3D非地面云不被该二维筛选裁掉。

GridPersistence 保留两段式规则：占据立即置100，连续漏观测不超过配置帧数仍为100，
超过才释放；20帧默认对应第21帧释放，重观测清零计数，尺寸变化和显式 reset 清空历史。
失效触发点留在节点，继续先清历史再按原协议发布未知失效图；有效空帧则正常推进持久化。
未增加点云复制、历史点云、线程或时钟读取；没有修改原参数、阈值或退出行为。
框投影及栅格构造保留原有限/可表示输入前提，本批没有夹带新的恶意配置数值守卫。

## 3. 对照与实际验证

证据：`artifacts/algorithms/alg7-3-20261018/`，最终矩阵 `matrix-v1/`。
受追踪摘要：[alg7-3-results.json](baseline/alg7-3-results.json)。迁移前保存33份功能包文件，
保存并前后核对 ALG-7.2 已编译节点库 SHA；未重建或覆盖原二进制。

原版探针使用保存的两头和从原回调提取的计算，地面类型使用未改动的 ALG-7.1 安装头；
新探针只从 occupancy 安装 target 消费，使用测试用结构化平面，不需要地面/PCL/ROS 头或库。
每种安装对照540帧，覆盖两种投影、膨胀、倾斜地面/高度、图外点与非法观测、完整预测尾段、
失效 reset、格数变化及不同保持帧数。精确比较几何元数据、成功/失败、预测框以及每帧所有格子。
格子输出采用可逆游程编码，不用占据总数替代逐格比较。两种安装均逐字节一致，trace SHA-256：
`f43bec3a39ca2a6c35cfb4fd65841d6ce3ba53bc605596d18b99d65908a2356a`。

新增实际节点测试先在旧二进制通过，再在两种新安装通过：两种 footprint 模式中，
障碍消失后的前20个实际输出帧仍占据，第21帧释放；重新观测后失效图为未知，恢复空帧不继承历史占据。
夹具按源帧时间匹配输入阶段，要求自身节点正常退出，强制清理不能算通过。
原房间/U型墙、两种 footprint/高度边界、地面生命周期、移动障碍/九点扫掠和分类节点回归继续通过。

| 每种安装方式 | 用例/消费检查数 |
| --- | --- |
| 原膨胀 / observed-cells | 11 |
| 新投影、预测尾段、持久化边界 | 6 |
| 原地图有效性 | 2 |
| 原分类消息与实际分类节点 | 6 |
| 原房间/footprint/高度边界实际节点 | 7 |
| 原地面生命周期与移动障碍实际节点 | 2 |
| 新实际保持/释放/reset（两种模式） | 2 |
| 安装旧头 / 无 ROS/PCL 独立 CMake 消费 | 2 |

每种38次，合计76次用例/消费检查；按 CTest 注册项计24次，全部通过，无跳过。
旧二进制持久化实际检查另计2次；540帧差分不混为单元测试条数。
地面/障碍包源码无改动，本批复用其安装并跑节点回归，不重计其核心测试。

普通复制安装根搬迁后重新配置、构建和运行纯消费者；安装头不是源码软链接。
纯 CMake 仅声明 CXX、仅提供 occupancy prefix，编译命令无源码 include，ldd 无 ROS/PCL 或缺失库。
旧头在支持的 ROS 环境消费；其功能包元数据仍会导入地面/PCL，CMake 需要 C/CXX 配置。
独立消费者以 -Wall/-Wextra/-Wpedantic/-Werror 编译。
Python 语法、包 XML 和 git diff --check 通过；未将全包 lint 或已知 shutdown/stress/capture 记为通过。

小规模性能检查：同机预热后交错运行完整540帧探针各7次，原/新墙钟中位 .03/.02s，
最大 RSS 中位均3680KiB。GNU time 分辨率 .01s，包含启动和游程格式化；
仅为粗粒度小栅格检查，不能据此认定生产加速或真实传感器延迟/资源满足。

矩阵期间45份源码指纹稳定，包数90→91。ALG-1..5 的13/36/69/29/146份保护源码不变。
ALG-7.2 的46份中39份不变，5份计划内 semantic_map 修改、2份原测试迁出；
ALG-7.1 的53份中44份不变，ALG-6 的81份中63份不变，差异均为三个建图子批次计划内修改/测试迁移。
ALG-0 冻结465份中377不变、72修改、16迁出/删除；原历史清单未改写。
初步构建漏导入 kMinimumGridInflationRadius，已补齐命名空间引用后通过最终矩阵，失败日志保留。

## 4. 复验与范围

保留原文件和已编译节点基线，在独立 symlink prefix 构建新核心/semantic_map 后执行：

```bash
python3 tools/algorithms/verify_occupancy_install.py \
  --symlink-prefix artifacts/algorithms/alg7-3-20261018/initial/install \
  --baseline artifacts/algorithms/alg7-3-20261018 \
  --output artifacts/algorithms/alg7-3-new-run
```

工具拒绝覆盖旧证据、检查原文件/节点库指纹，并验证两种安装、旧签名和独立消费。
使用已安装 ROS/主安装/scene-s3/ALG-6/ALG-7.1/ALG-7.2 underlay，不是全工作区 clean rebuild。
搬迁验证覆盖核心 CMake 导出和普通消费，不保证所有 colcon shell setup 可搬迁。
ROS 测试在非零隔离域串行使用合成点云/状态/TF，不启动 PX4/Gazebo、不发送飞行任务，
不覆盖主 build/install 或停止用户仿真。ALG0-B02/B03 与历史 semantic 退出问题继续保留未解决。
