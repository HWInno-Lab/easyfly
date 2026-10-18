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
