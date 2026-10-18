# ALG-8.4 QGC 接入资料分类验收

日期：2026-10-18；迁移前提交 `5d84150`，原算法盘点基线 `bdbc34c`。
`src/third_party/qgc/README.md` 迁至 `src/third_party/tools/qgc/README.md`，
桌面使用说明同步更新引用；增加外部工具索引及QGC身份/接入范围记录。
本批只有资料分类与引用调整，下一步 ALG-8.5 接入规范收敛。

## 1. 文件归属与调用边界

QGC目录原先只有一份接入README，没有QGC源码或安装程序。
用户本地 `src/third_party/QGC.AppImage` 保持原位置，仍由既有文件选择器、
保存的连接配置或 `--qgc-path` 显式选择，不按文档目录猜测可执行文件。
本批不更改运行代码、启动参数、二进制忽略规则或任何CMake/install规则。
分类父目录不含package.xml；colcon包集合、名称、构建类型与迁移前一致，仍为91包。

README保留原二进制身份与许可观察，补充当前已实现的联网维护入口：
离线隔离预览与联网维护各自遵守原启动/会话条件，文件选择本身不会启动或联网。
原README中的“联网待另行集成”描述更新为当前连接管理器的文档链接；实现仍在UI层。
见 [QGC说明](../third_party/tools/qgc/README.md)和
[身份与范围清单](../third_party/tools/qgc/provenance.json)。

本机样本180816376字节，SHA256为
`06969c67ef58ea063def0a8271447a1cc385438c4a7df36813315b4475146737`，
与既有README登记相同。前后inode、大小、权限、mtime及SHA保持；文件继续被Git忽略。
原先观察的5.0.8、Qt6.8.3、嵌入GPL-3.0元数据只对应这个样本，不代表其他用户选定文件，
也不是签名或下载来源认证。本批未执行、解包、下载、移动或重新发布该AppImage。

## 2. 验证结果

证据根 `artifacts/algorithms/alg8-4-20261018/`，摘要
[alg8-4-results.json](baseline/alg8-4-results.json)。迁移前归档2份资料，
109份UI实现/既有测试二进制/仿真程序输入逐字节不变，算法和SDK实现未改。

复用已有测试二进制，仅运行相关原用例，没有新增测试或重建UI：

| 原有用例 | 数量 | 实际范围 |
| --- | --- | --- |
| Tools | 3 | 非法路径/格式拒绝；含空格路径按参数传递；QGC无网络回退及真实fixture的私有网络隔离 |
| ConnectionProfile | 4 | 配置往返/非法输入；私有原子保存；仿真/串口/维护参数约束；旧配置迁移 |

共7项通过，0失败/禁用/跳过。实际离线运行的是原测试AppImage桩，
没有启动用户的QGC、连接飞控或启动PX4/Gazebo。
不将这些结果称为新的QGC窗口、心跳、MAVROS转发、真实联网维护或飞行验收。
测试程序与桩的SHA随输入记录，直接执行GTest并保存XML，没有计入CTest注册项。

测试使用offscreen和非零DDS域226，配置夹具只写其临时目录。
原FAST-LIO、EGO、traj_server验收结束仍在，已有主build/install未被重建或替换。
历史ALG0-B03、semantic shutdown/stress及第三方来源缺口继续保留。

## 3. 复验与静态核对

加载已有ROS和主安装后，以现有测试程序执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1 \
  build/easyfly_desktop/test_tools \
  --gtest_filter=Tools.MissingInvalidPathAndSandboxFailClosed:Tools.CommandsUseArgumentsAndQgcHasNoNetworkFallback:Tools.OfflineQgcWrapperCannotReachHostLoopback
QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1 \
  artifacts/ui-reliability-phase12-20261017/build/easyfly_connection_manager/test_connections \
  --gtest_filter=ConnectionProfile.StrictRoundtripAndUnsupportedInputs:ConnectionProfile.PrivateAtomicStorePreservesInvalidData:ConnectionProfile.SimulationSerialAndMaintenanceContracts:ConnectionProfile.LegacyProfilesMigrateWithoutStartingOrOverwriting
```

以上依赖本机既有二进制；新检出须先按各UI包原流程构建，不是本批新的clean build证明。
本批检查新文档相对链接、JSON、Git忽略和提交范围，git diff --check通过。
运行源码不再引用旧资料路径；历史ALG-0清单、原盘点工具及历史证据中的旧路径保留为历史。
原二进制路径引用与.gitignore规则继续有效，不将它们替换成新的资料目录。
