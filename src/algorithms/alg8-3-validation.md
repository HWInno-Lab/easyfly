# ALG-8.3 传感器 SDK 分类迁移验收

2026-10-18；迁移前提交 `6d4115e`，原算法盘点基线 `bdbc34c`。
Livox SDK2 与 librealsense 整体移至 `src/third_party/sensor_sdks/`。
本批完成 SDK 分类与消费者核对；下一步 ALG-8.4 QGC 接入资料分类。

## 1. 完整源码与实际版本

Livox 的220份、RealSense 的3202份受追踪文件全部逐字节相同，共3422份，无 SDK 源码或 CMake 补丁。
目录内原 build 缓存随原目录移动并继续被忽略，SHA保持；新验证没有使用这些缓存。
RealSense 原有19份被全局忽略的源文件也实际保留，未作为新增文件误入提交。
原许可、NOTICE及内嵌依赖布局保留；精确上游导入 commit 仍未知，本地c707cd3不是上游版本锁。
逐文件 SHA 与已知来源分别在
[Livox清单](../third_party/sensor_sdks/livox_sdk2.provenance.json)、
[RealSense清单](../third_party/sensor_sdks/librealsense.provenance.json)。

Livox公共头声明和实际系统/新编译库查询均为 **1.3.0**；CHANGELOG最新条目仍为1.2.5，
sdk_core CMake 中的库版本变量仍为0.0.2。分别登记，不把旧交接描述当作当前版本。
RealSense头、包元数据及两份实际库API均为2.56.5（API整数25605）。
Livox原LICENSE.txt对列出的文件范围授权MIT；RealSense根Apache-2.0、NOTICE及内嵌依赖各自许可原样保留。

两个 SDK 仍分别是 colcon 的 livox_sdk2（cmake）与 librealsense2（ros.cmake）包。
前后91包集合、名称与构建类型不变，只修改两项源码目录；分类父目录不含 package.xml。
现有安装未发现指向旧 SDK 源码根的资源软链接，因此没有新增旧路径桥接。
历史 CMake 缓存中的旧绝对路径不改写；重新构建必须使用新的 build 目录。

## 2. 驱动与安装选择

Livox ROS驱动及RealSense ROS嵌套仓库均留在 communication/c1_drivers，256份受追踪驱动文件字节不变。
RealSense ROS gitlink、实际HEAD仍为d69f05c1b4a3f0d5740e25d6429d4417c7f3d75d，工作树干净。
根.gitmodules缺该映射的问题继续保留；本批不修改或重新初始化该仓库。

| 验证模式 | SDK解析与运行 | 驱动安装 |
| --- | --- | --- |
| default-symlink | 自动发现/usr/local/include、/usr/local/lib、cmake/realsense2 | 独立软链接安装 |
| explicit-copy | 显式选择新源码编译且已搬迁的SDK安装根 | 独立复制安装，再搬迁根 |

每种都构建 livox_ros_driver2、realsense2_camera_msgs、realsense2_camera。
从清理后的环境只加载 /opt/ros/humble，避免旧主工作区的SDK前缀干扰默认发现。
检查CMake缓存、编译器实际头依赖文件及ldd加载路径，不只查看配置参数。
消费者编译命令不含工作区源码include；两种驱动安装的35份config/launch/examples资源各自与源码SHA一致。
新SDK安装的3份Livox头和29份RealSense头与源码逐字节相同。
原主build/install、/usr/local头/库和现有仿真二进制没有重建或覆盖；43份安装输入哈希验收前后相同。

## 3. 实际软件验证

证据根 `artifacts/algorithms/alg8-3-20261018/`，最终矩阵 `matrix-v1/`；
摘要见 [alg8-3-results.json](baseline/alg8-3-results.json)，附加头依赖/资源核对为 post-validation-audit.json。

- 从新SDK目录独立配置、编译、安装两份真实原生库，不复制旧库充当构建结果。
- 每种模式运行真实SDK版本查询、512组RealSense反投影→正投影和外参平移检查，以及解析FOV检查。
  512组输入在系统库与新编译库上的17位精度输出逐字节一致；涵盖无畸变针孔几何，不冒称覆盖全部畸变模型。
- 每种模式以真实类型支持做Livox CustomMsg、RealSense RGBD的CDR往返，共4次消息往返。
- 每种模式通过class_loader实际加载两份驱动共享库并核对NodeFactory注册，共4次组件库加载；
  不创建工厂实例或设备节点，不初始化Livox SDK、枚举设备、启动采集或发送设备命令。
- 新SDK安装根先实际搬迁，显式驱动从搬迁根构建；复制驱动安装再搬迁后重新配置/编译并运行消费者。

两种模式共4次独立消费者进程运行，均通过。本批没有运行上游占位的2+2断言GTest，
也没有将硬件、rosbag、网络数据下载或完整包lint计为通过。
驱动BUILD_TESTING=OFF，避免上游在配置阶段枚举相机和注册硬件夹具；生产源码和默认值未改。

## 4. 离线验证配置与边界

RealSense原始构建会联网检查并获取JSON；本批用只作用于验证工程的 sdk_offline.cmake 项目hook，
关闭联网探测，复用本机已存在的 nlohmann/json 源码缓存。
缓存commit为9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03，1090份受追踪输入SHA已记录；
没有重新取上游或升级依赖。原生库保留Release编译与默认Linux V4L2后端，使用系统USB/udev依赖。
验证显式关闭固件/更新、GUI、工具、DDS及硬件测试，不能代表所有SDK构建选项均已验收。

初版hook把JSON放在独立输入位置，SDK其他模块的固定binary-tree include路径因此缺头，初步编译失败。
修正验证hook，在新build树中按上游约定放置同一份已记录源码；SDK源文件未改，失败日志保留。
原生库最终完整构建通过；没有删除或放宽生产校验来绕过编译问题。

上游安装脚本无条件执行ldconfig。本批只在该私有安装进程的PATH前置显式no-op，日志记录跳过系统缓存更新；
未运行sudo、安装udev或覆盖/usr/local，未更改SDK默认安装脚本。
复制重定位验证使用显式CMake包路径及进程级库加载顺序，不承诺任意colcon setup脚本或pkg-config文件可搬迁。

矩阵运行期间3689份输入指纹稳定。结束后仅修正SDK README中的验收文档相对链接，前后SHA另行登记；
测试代码、构建配置、SDK和驱动输入保持原样，未为文档链接重复运行构建或软件测试。
ALG-1..5、ALG-7.3受保护输入全不变；ALG-6/7.1/7.2沿用已登记差异。
ALG-8.1/8.2只更新共同的.gitignore及third_party根索引，FAST-LIO/EGO实现不变。

## 5. 复验

使用保留的迁移前输入/JSON缓存及本机环境，选择全新证据目录：

```bash
python3 tools/algorithms/verify_sensor_sdks.py \
  --baseline artifacts/algorithms/alg8-3-20261018 \
  --native-build artifacts/algorithms/alg8-3-recheck/native \
  --default-prefix artifacts/algorithms/alg8-3-recheck/default/install \
  --output artifacts/algorithms/alg8-3-recheck/matrix
```

工具拒绝覆盖旧证据。该环境依赖已有ROS/系统SDK/USB/udev及归档输入，不是完整工作区clean rebuild。
原FAST-LIO、EGO、traj_server进程验收结束仍在；未新启动PX4/Gazebo或发送飞行任务。
未验证真实雷达/相机采集、校准、设备固件及实机吞吐。
ALG0-B03和历史semantic shutdown/stress问题继续保留，不以SDK分类认定其已解决。
