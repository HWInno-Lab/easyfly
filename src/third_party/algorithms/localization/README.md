# 外部定位与建图算法

`fast_lio/` 是 ROS2 FAST-LIO 完整工程，包名仍为 `fast_lio`，包含原 ikd-Tree/IKFoM。
EasyFly 的 launch、map→odom TF 和平台接纳仍在 `easyfly_fast_lio_integration`。
不在分类父目录创建 package.xml，不将其内部模块拆成自有算法或改为普通 C++ API。

来源/字节清单见 [fast_lio.provenance.json](fast_lio.provenance.json)。README 中的上游 URL
只是当前源码可确认的来源线索；精确 fork/branch/import commit 仍未知，未联网拉取或升级。
本项目历史导入点 `7455e06`，外参保护等本地补丁见 `b52a4fd`。这些是 EasyFly 提交，不能当作上游基点。
根 LICENSE 为 GPL v2 文本，而 package.xml 仍声明 BSD，差异原样登记，未重新标记许可；
嵌入代码保留自身通知，来源/许可缺口不由目录移动补齐。

新构建按 colcon 包名发现：

```bash
colcon build --packages-select fast_lio easyfly_fast_lio_integration
```

需要已有 ROS/PCL/Eigen 和 livox_ros_driver2。mid360.yaml、话题、消息、外参校验及算法数值保持。
CMake 的 FASTLIO_OUTPUT_ROOT 可显式指定可写日志/PCD根，配置时创建 Log/ 和 PCD/；
默认仍使用上游源码根布局。独立构建/验证必须指定自己的输出根，避免覆盖运行会话的文件。
安装 share/fast_lio 继续提供 config/launch/rviz，并增加原 LICENSE 和 README 安装。

旧 `src/third_party/slam/fast_lio` 路径只是迁移前布局。当前机器上可能保留一个被 Git 忽略的
本地软链接，由旧父目录 COLCON_IGNORE 排除，用于已启动的旧二进制 ROOT_DIR 和旧安装软链接。
它不是第二份源码，也不是可提交/可依赖的接入接口；从当前仓库重新检出只会得到新目录。
旧主安装/构建缓存没有被这次验证覆盖；切换主安装应使用新源码重新配置构建，确认旧会话结束后再清理本地桥接。

验收和复验入口见 [ALG-8.1](../../../algorithms/alg8-1-validation.md)。

## ORB-SLAM3（仿真验收进行中）

`orb_slam3/` 固定官方提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`，
以 `easyfly_orbslam3_core` 导出安装 target；`easyfly_orbslam3_integration` 提供原生
Gazebo RGB-D 接入。私有 Pangolin v0.6 源码在 `orb_slam3_dependencies/pangolin/`，
通过 CMake 单独构建和安装，再由 `find_package(Pangolin)` 解析；不导入预编译库。
来源、逐文件哈希、归档与有序本地补丁见 [来源清单](orb_slam3.provenance.json)。
`tools/algorithms/verify_orbslam3_source.py` 可零模糊重建并逐文件核对。

固定轴/机体标定后，原始 SLAM 位姿通过共享定位对齐和显式导航适配器接入。
默认不授权导航；地图坐标代次变化锁存重置，不自动对齐后继续任务。
ORB 协方差明确不可用。线程正常停止后才能保存图谱，旧图复用必须实际重定位。
惯性内核的 `EASYFLY_ORB_GRAVITY` 默认与登记 Gazebo 世界一致，为 9.8 m/s²；
该编译参数随安装 target 导出，RGB-D/IMU 接入同时核对 YAML 的 `IMU.Gravity`
和实际加载内核的重力值，防止新配置混用旧的 9.81 内核。原始 IMU 数据不作缩放。
原生静止预积分回归覆盖这个标定差异，不能代替完整惯性回环的精度验收。
[实施及验收记录](../../../../docs/development/orbslam3-simulation-acceptance.md) 区分
真实回环、旧图复用、导航与故障验证。实机/目标机未验收。
