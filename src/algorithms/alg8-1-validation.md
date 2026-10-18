# ALG-8.1 FAST-LIO 第三方分类迁移验收

日期：2026-10-18，迁移前提交 `0eead7a`，原算法盘点基线 `bdbc34c`。
本批只迁移 FAST-LIO，下一步 ALG-8.2 EGO；SDK/QGC 不提前移动。

## 1. 实际结果

完整工程由历史 `src/third_party/slam/fast_lio` 移至
`src/third_party/algorithms/localization/fast_lio`，ROS 包名仍为 fast_lio。
ikd-Tree/IKFoM、内部相对路径、消息/配置/launch/版权通知及原外参保护均保留。
父分类目录没有 package.xml，colcon 仍发现91包，fast_lio 仅从新路径发现。
EasyFly 的 launch/TF 实现继续位于 easyfly_fast_lio_integration，不将完整 ROS 工程改成普通算法库。

46份原受追踪 vendor 文件中45份逐字节相同；唯一变化 CMakeLists：
FASTLIO_OUTPUT_ROOT cache 可覆盖可写 Log/PCD 根（默认仍为原源码布局），配置时创建目录；
原 LICENSE/README 随既有 share 资源安装。算法数值、外参校验、话题、配置、消息和编译优化规则未改。
两种验证构建分别指定 artifacts 中自己的输出根，避免新进程覆盖当前仿真日志。
新安装二进制不含旧源码根字符串；安装消息、配置/launch/rviz、LICENSE 与输入逐字节核对。

27份128MB演示资源随目录实际移动，hash 前后不变，新忽略规则覆盖 doc/；未误入提交。
来源/逐文件身份见 [来源清单](../third_party/algorithms/localization/fast_lio.provenance.json)
和 [接入说明](../third_party/algorithms/localization/README.md)。上游 URL 仅来自现有 README，
精确 fork/branch/import commit 仍未知；7455e06/b52a4fd 是 EasyFly 本地历史，不冒充上游版本。
根 GPL v2 文本与 package.xml 的 BSD 声明差异原样登记，没有重新标记或推断整体许可。

## 2. 当前会话兼容

迁移时用户仿真已有 FAST-LIO 进程，其 ROOT_DIR 和主安装软链接仍指向旧源码路径。
目录移动后在本机保留一个被 Git 忽略的旧路径软链接，指向唯一的新源码；
旧 slam/ 父目录中的本地 COLCON_IGNORE 排除它，实际扫描已验证没有重复包。
该软链接和 marker 均不提交，不是公共接入接口或第二份源码。

主 build/install、旧 FAST-LIO/TF 二进制未被重建或替换，前后 SHA 不变；旧进程验收结束仍在。
当前会话可继续使用原资源与输出句柄；新构建/测试使用新目录及独立输出根。
从仓库重新检出只得到新分类目录；应使用新源码重新配置构建主安装，确认旧消费者结束后再清理本地桥接。
不以本批隔离构建宣称主安装已切换。

## 3. 启动契约及 ALG0-B02

保留原测试并用当前支持包重放：原夹具仍在生成父启动描述时被缺失的 HRT 前提拒绝，
日志包含 SITL refused / no unpatched fallback。生产拒绝逻辑没有被绕过或删除。

本批修正的是启动参数 scoping 夹具的依赖隔离：只在该夹具导入的临时父模块上替换
PX4目录解析和 native IMU preflight，finally 恢复；不执行父启动描述，不启动 PX4/Gazebo。
仍实际执行原 GroupAction/Include/子节点参数展开，覆盖 legacy config_file、父检测配置隔离、
延后 map→odom 组件的布尔时钟冻结。生产 launch、TF 和 HRT/native IMU 源码字节不变。
另跑现有 HRT selection/native IMU 的13项 fail-closed 测试，全部通过。

ALG0-B02 的软件启动契约夹具阻塞由此解决；这不等于新的完整 SITL 启动或飞行验收。
历史 ALG-0 失败证据/冻结清单保留，不改写为当时通过。
初步默认支持包缺 imu_compatibility，故在隔离前缀刷新当前 easyfly_px4_sitl；最终矩阵明确使用该支持包。

## 4. 验证结果

证据根 `artifacts/algorithms/alg8-1-20261018/`，最终矩阵 `matrix-v3/`。
摘要：[alg8-1-results.json](baseline/alg8-1-results.json)。迁移前归档51份 vendor/集成文件，
记录原二进制和演示资源身份；最终矩阵61份输入指纹稳定。

| 每种安装 | 检查数 |
| --- | --- |
| 原外参 GTest | 3 |
| 新实际 map→odom TF，默认与自定义参数 | 2 |
| 原启动语义脚本（内含3组场景，按1个注册项计） | 1 |
| 安装 Pose6D C++ 序列化/反序列化消费者 | 1 |
| 实际 FAST-LIO 初始化/正常退出及三类非法外参拒绝 | 4 |

两种安装合计22次用例/消费检查，按 CTest 注册项计8次，全部通过。
13项 HRT/native IMU 回归和旧安装 TF 的2项基线检查另计，没有混入22次。
原夹具的预期 HRT 失败单独保留，不冒充通过用例。

新 TF 测试先在旧安装通过，再在两种新安装通过：静态 map→slam 恒等、无 odom 不输出估计、
实际变换乘逆、平移平滑/旋转 slerp、自定义坐标系和源时间戳；要求正常退出。
FAST-LIO 实际进程从安装 share 读取 mid360.yaml，确认原 Livox/IMU 订阅和 map_save 服务注册，
不调用保存服务或发布传感器；短平移、短旋转和非有限旋转均按原逻辑受控退出1，正常启动退出0。
未进行新的点云建图精度、实时性或完整 SLAM 资格验收；源算法保持以字节身份核对。

复制安装根搬迁后重新配置、构建并运行 ROS C++ 消息消费者，实际 typesupport 从新 prefix 加载；
实际 FAST-LIO 也从搬迁后的二进制/配置运行，依赖无缺失。消费者编译命令不引用源码 include，
使用 -Wall/-Wextra/-Wpedantic/-Werror。此阶段保留 ROS/PCL 依赖，不将 ROS 工程冒称无 ROS 核心。
实际 PCL 来自系统，Livox 消息来自原主安装；SDK未移动或覆盖，构建未复用旧 FAST-LIO build/install。
搬迁范围是 CMake/原生消息及进程加载，显式选择当前 prefix 的库；不承诺任意 colcon shell setup 可重定位。

初步验证工具有两个检查器问题：package-local setup 位置写错；误要求节点链接未使用的自有消息库。
已修正检查器并保留失败日志。节点仍检查依赖无缺失；真正使用 Pose6D 的独立消费者仍严格要求新 prefix 的 typesupport，未放宽生产断言。
完整包 lint 未计为通过；Python语法、XML、git diff --check 通过。
ALG-1..5 的13/36/69/29/146份保护输入与 ALG-7.3 的45份输入均不变，
ALG-6/7.1/7.2 沿用上一批已登记差异，本批没有新增差异。

## 5. 复验与范围

保留原文件/二进制基线，在独立前缀构建 fast_lio、集成包和当前支持包后运行：

```bash
python3 tools/algorithms/verify_fastlio_install.py \
  --symlink-prefix artifacts/algorithms/alg8-1-20261018/initial/install \
  --baseline artifacts/algorithms/alg8-1-20261018 \
  --output artifacts/algorithms/alg8-1-new-run
```

工具拒绝覆盖旧证据。使用已有 ROS/主安装/scene-s3 underlay 与当前支持包，不是全工作区 clean rebuild。
本机旧路径兼容链接/排除 marker 是此运行前提，复验工具会核对，不能在新检出中假设它存在。
测试使用非零隔离 DDS 域，不启动新的 PX4/Gazebo、不发送飞行任务、不停止用户仿真。
ALG0-B03 和历史 semantic shutdown/stress 问题继续保留未解决。
