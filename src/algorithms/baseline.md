# ALG-0 基线记录

日期：2026-10-18。生产源码基线：`bdbc34c`。

本轮执行算法清单、迁移边界、接口草案及现有软件测试的基线采集。尚未移动算法或第三方
源码，未更改运行接口、算法参数和控制逻辑。新文件是本阶段文档、清单与盘点/测试工具。

## 1. 盘点结果

- `migration-inventory.json` 登记 26 个组件：15 项自有计算/库包装、2 个第三方算法工程、
  9 项明确保留的平台模块；另列 6 项第三方目录重定位和 6 个算法占位包。
- 自有算法按 preprocessing/detection/tracking/localization/fusion_mapping/planning 分类。
  外部算法按同类功能分类，runtime、设备 SDK、通用库和外部工具分别管理。
- 目标名称与现有包无冲突，colcon 仍发现 72 个包；本轮没有创建新的算法构建包。
- [输入指纹](baseline/alg0-inputs.json) 记录来源文件、配置/launch/测试、直接 include/import、
  package.xml 依赖、第三方树摘要、忽略资产 hash、源码引用及上游相对路径。
  最终冻结 26 个来源包的 465 个文件，原先采集的 397 个文件指纹未变化；新增部分为支持依赖。
- [接口草案](interface-contracts.md) 冻结单位、ID、时间、失败、内存和保留职责，并把新增
  reset/取消/预算等能力与当前真实实现分开记录。

## 2. 构建环境与边界

基线产物全部位于工作区本地 `artifacts/algorithms/alg0-20261018/`，被现有 Git 规则忽略。
原 `build/`、`install/` 未被覆盖；原有仿真会话未被停止。所有软件测试串行执行，默认
DDS 域 212，测试自身指定的非零域继续生效；本轮未启动 Gazebo/PX4 或发送实机任务。

独立编译 17 个相关目标包，并额外编译启动支持包 `easyfly_px4_sitl`。使用原安装中的
消息、规划器和其他依赖，并非全工作区依赖的全量干净重建；普通复制安装也未在 ALG-0 验收。
每次命令及实际 AMENT/运行库路径保存在相应 JSON，不能用本轮结论替代 ALG-9 安装矩阵。

构建历史完整保留：

1. `build-command.json` / `build.log`：首轮 14 包成功，飞控因默认 underlay 缺少
   `easyfly_fastdds_compat` 的安装环境文件失败，其后 2 包未执行。
2. `build-completion-command.json` / `build-completion.log`：显式引用既有
   `artifacts/scene-s3-20261016/normal/install/easyfly_fastdds_compat` 后，剩余 3 包成功。
   该前缀的 patch_endpoint.py 和 component_services.xml 与当前源码逐字一致，私有库 hash
   `b56bc087770bf881e100426b16d97acfe2a4fef52f7f0f3d9314a2c7b3fe8c3d` 与安装签名一致。
3. `build-support-command.json` / `build-support.log`：原 underlay 的 Python 启动包缺少
   imu_compatibility 模块，故在独立前缀重新安装当前 `easyfly_px4_sitl` 源码成功。

没有为了补环境而下载/升级中间件、重建 PX4 或修改系统安装。引用的已有兼容库仍是本地
前提，其身份与实际 EGO/桥接二进制 hash 另保存在 `underlay-binaries.json`。

## 3. 软件测试结果

最终统计见 [结果摘要](baseline/alg0-results.json)。计数单位为 CTest 注册项，
一个注册项可能包含多条 GTest/pytest 用例，不以注册项数量冒充内部用例数量。

17 个目标包共 71 个不同注册项：69 通过、2 失败、0 跳过。包含一次环境刷新后的失败项复验，
实际执行 72 次注册项；不能将最新统计改写为全部通过。

测试批次：

- `tests-initial/`：14 包、44 项；43 通过，FAST-LIO 启动契约 1 项失败。
- `tests-fastlio-refreshed/`：支持包刷新后定向复验该契约，仍失败，原因进入下一层 HRT 前提检查。
- `tests-execution/`：飞控、全局导航、任务状态机 27 项，26 通过、停流链 1 项失败。
  其中全局规划 11/11、任务状态机 5/5，飞控 10/11。

全部实际结果、逐项状态、命令、耗时和日志 hash 随最终摘要记录。首轮失败不覆盖、不删除，
复验只因输入环境改变而进行，未通过重试挑选成功结果。

## 4. 基线问题与后续处理

### ALG0-B01：默认安装与当前源码不一致（环境项）

原构建目录缺少当前部分地面/占据测试；默认安装缺 Fast DDS 兼容包环境及新版启动模块。
采用独立编译和明确的已有依赖前缀后完成相关目标包构建，保留失败记录。
后续执行人员应从记录的安装前缀启动验证，不能假设工作区默认安装已经完整同步。

### ALG0-B02：FAST-LIO 启动契约仍依赖完整 SITL 前提（未通过）

首轮报 `ModuleNotFoundError: easyfly_px4_sitl.imu_compatibility`；刷新支持包后，
在 `generate_launch_description()` 内解析 PX4 时拒绝缺失的
`artifacts/algorithms/alg0-20261018/build/easyfly_px4_hrt_compat`。
该软件契约的父启动描述生成过程已经包含 HRT/native IMU 前提，当前测试没有完整隔离这些
依赖。此轮既未配置或重建该 HRT 树，也未绕过启动检查；不能据此判定 FAST-LIO 算法失效，
也不能声称其完整启动链已经通过。

后续单独调整测试夹具的依赖隔离，或在明确记录的完整前提下复验；生产拒绝逻辑保持。
ALG-8 的 FAST-LIO 路径迁移签收前需要解决或给出完整可复现的启动验证。

### ALG0-B03：冻结时钟的停流测试初始就绪超时（未通过）

`test_stop_stream_chain` 在 `test_stop_stream_chain.py:159` 等待 connected、pose_valid、
attitude_valid 及 Action 服务就绪时超时，尚未进入其主要停流断言。
夹具自开始即固定 `/clock=10s`；桥接实现对同一 ROS 时间戳仅发布一次 `/uav_state`。
这两处行为需要单独检查初始发现和状态接纳时序，当前证据不足以唯一归因或宣称停流实现故障。

失败子进程日志已复制到 `stop-stream-failure/`，不依赖临时 pytest 目录长期存在。
本轮未修改桥接、飞控、时钟守卫或该测试。涉及这条链的后续迁移不能报告为无已知失败基线。

### ALG0-B04：现有验证与接口缺口（范围项）

- SORT 没有独立 reset，缺完整 raw/stable ID 回流专项 ROS 测试，ALG-2 补齐。
- A* 核心没有预算/取消参数，节点 deadline 回归不等于搜索循环已可中断，ALG-3 独立增强。
- 当前检测注册项是静态源码契约，本轮未重跑模型精度/实推理验收；历史 m2 JSON 不算新结果。
- pixel_to_3d 当前注册核心测试覆盖 frame/header，不足以替代中位深度和反投影数值测试。
- semantic_map 的既有退出问题继续保持未关闭；本轮退出/压力回归通过不等于缺陷修复。
- 第三方原始基点和许可缺口沿用来源记录，不以目录规划或新 hash 伪造补齐。

## 5. 已冻结的路径风险

1. 检测默认 ORT_ROOT 的源码相对路径、SONAME/provider 安装和忽略规则需一同迁移。
2. EGO plan_manage 通过 `../../..` 推导根，bspline_opt 从 plan_env_DIR 推导 include；
   整体重定位也必须验证安装消费，不能只保证内部相邻文件还在。
3. 当前 Livox CMake 实际解析 SDK include 为 `/usr/local/include`，库为
   `/usr/local/lib/liblivox_lidar_sdk_shared.so`；RealSense 解析到
   `/usr/local/lib/cmake/realsense2`。源码移动不等于运行依赖已切换。
4. SDK 的 build、ONNX、FAST-LIO 演示资源原有 Git 忽略规则需映射；用户 QGC 二进制不自动移动。
5. 上游引用明细和字节身份见输入指纹，完整第三方工程结构及补丁保留。

## 6. 复现入口

在工作区根目录加载 ROS、记录的兼容依赖及独立 overlay 后，可用以下工具重新采集；
输出必须选择新路径，工具拒绝覆盖已有证据。历史 build-command JSON 保留实际构建命令。

```bash
python3 tools/algorithms/capture_inventory.py --output artifacts/alg0-repeat/inputs.json
python3 tools/algorithms/run_baseline.py \
  --build-base artifacts/algorithms/alg0-20261018/build \
  --output artifacts/alg0-repeat/tests
```

第二条运行清单内既有软件回归，仍可能复现上述失败；不调用 SITL 验收脚本。
来源文件/hash、直接依赖与测试发现不是功能正确性的替代证据，结果须结合各批次环境读取。
