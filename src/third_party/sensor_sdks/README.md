# 传感器 SDK

`livox_sdk2/` 与 `librealsense/` 分别保留完整 Livox/RealSense SDK 源码及内部依赖。
它们仍被 colcon 识别为 livox_sdk2（cmake）和 librealsense2（ros.cmake），不是新的 EasyFly 算法包。
ROS 驱动保持在 communication/c1_drivers；RealSense ROS 嵌套仓库/gitlink保持原位和原提交。

| SDK | 本地源码证据 | 驱动 |
| --- | --- | --- |
| Livox | 公共头声明1.3.0；CHANGELOG最新1.2.5；sdk_core CMake库变量0.0.2，分别记录 | livox_ros_driver2 |
| RealSense | rs.h/package.xml声明2.56.5 | realsense2_camera |

逐文件身份与来源缺口见 [Livox来源清单](livox_sdk2.provenance.json)、
[RealSense来源清单](librealsense.provenance.json)。3422份受追踪文件全部逐字节搬迁，无SDK补丁或版本升级。
本地导入点c707cd3不代表上游版本锁定；精确导入commit未知。
Livox原LICENSE.txt按其列出的MIT范围保留；RealSense根Apache-2.0、NOTICE及所有内嵌依赖通知保留。
不能以SDK根许可替代每份内嵌第三方代码的许可。

## 实际选择的安装

目录分类不会改变驱动解析：Livox使用find_library/find_path，RealSense使用find_package(realsense2 2.56)。
当前默认解析到/usr/local/include、/usr/local/lib及其cmake/realsense2；源码存在不表示消费者使用它。
在独立前缀安装本目录源码后，显式选择：

```text
-DLIVOX_LIDAR_SDK_INCLUDE_DIR:PATH=<prefix>/include
-DLIVOX_LIDAR_SDK_LIBRARY:FILEPATH=<prefix>/lib/liblivox_lidar_sdk_shared.so
-Drealsense2_DIR:PATH=<prefix>/lib/cmake/realsense2
```

同步检查CMake缓存、编译头路径和实际加载库。运行时仅为该进程设置prefix/lib的加载顺序，
不覆盖系统库或给所有ROS进程修改环境。驱动配置、设备IP/序列号、固件、USB权限/udev仍由设备接入层管理。

## 构建与验收范围

原build缓存随目录移动并继续忽略，其中绝对源码路径仍是历史路径；不要用它们当作新目录构建结果。
新构建使用artifacts独立目录/安装根；主build/install及/usr/local保持。
本批没有旧SDK软链接，旧安装资源没有指向这些源码根的链接，colcon包集合仍为91。

验证工具为工作区 `tools/algorithms/verify_sensor_sdks.py`，
完整命令见 [ALG-8.3验收](../../algorithms/alg8-3-validation.md)。
两种驱动安装分别验证默认系统SDK与显式选择的新编译SDK；普通复制安装和SDK根实际搬迁后重新消费。
运行版本查询、512组RealSense几何正反投影/外参变换/FOV、两类ROS消息CDR往返及真实组件库加载。
不初始化Livox SDK、不创建相机/雷达节点，不枚举设备；不以软件验证认定实机或飞行已验收。

RealSense独立验证显式关闭联网、固件/更新、GUI、DDS和硬件测试，使用已有JSON源码缓存的固定字节。
`tools/algorithms/sdk_offline.cmake`仅供该验证工程使用，不修改SDK源文件或生产默认设置；
安装进程内跳过上游ldconfig命令，避免更新系统缓存。完整SDK默认构建可能下载依赖、固件并执行安装动作，
需要按部署环境单独配置，不能将本批私有前缀命令等同于设备部署。

RealSense ROS原有.gitmodules映射缺口、SDK上游来源缺口及历史ALG0-B03/semantic退出问题继续保留。
