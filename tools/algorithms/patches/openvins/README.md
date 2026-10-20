# OpenVINS ROS2 生命周期兼容补丁

基点：官方 `rpng/open_vins` 提交 `69488123ed9362dd44b6f28e7f4680abbff1442b`。
应用顺序：`0001-ros2-visualizer-lifetime.patch`，再应用 `0002-install-notices.patch`。
第二项仅将原始 LICENSE 和 ReadMe.md 安装至三个 ROS2 包的 share，不改变计算行为。

补丁涉及上游文件，保留其原有 GPLv3-or-later 通知；EasyFly 根目录的 Apache-2.0
不覆盖这些上游文件或其派生补丁。Python 验证工具为 EasyFly 自有代码。

原始版本在 ROS2 Humble 的无输入订阅节点上，收到 SIGINT 后进入全局析构，
`ROS2Visualizer` 的 `image_transport::Publisher` 释放触发 SIGSEGV。
同一环境中，合成运行器在配置错误后的全局析构路径中阻塞于 Fast DDS 的发布器销毁。
实际 GDB 栈与原始失败记录位于本地：

```text
artifacts/new-algorithms/openvins-build-20261019/
  verification/runtime-1/
  debug-shutdown-3/gdb.log
  debug-native-2/gdb.log
```

补丁改变四个文件：

- `ROS2Visualizer.h/.cpp`：拥有图像发布线程，停止并 join 后再释放成员，替代 detach。
- `run_subscribe_msckf.cpp`：正常结束和配置错误时显式释放可视化、估计器与节点；正常结束同时移除执行器节点。
- `run_simulation.cpp`：正常结束和配置错误时显式释放可视化、估计器、模拟器与节点。

补丁不修改滤波、特征、初始化数值计算或标定值。它覆盖本轮启动/退出验证的资源路径，
不代表已经验证真实相机输入下所有异步更新线程的关闭行为。

验证工具通过 `--patch` 在输出目录创建独立 Git 副本，原始 checkout 保持干净；
记录基点、补丁 SHA256、构建输入与日志，拒绝未登记的源码差异。
完整复现与边界见本地 `docs/development/openvins-isolated-build.md`。

生产导入只包含 `ov_core`、`ov_init`、`ov_msckf`、配置与根许可/说明。
来源清单为 `src/third_party/algorithms/localization/open_vins.provenance.json`；
`verify_openvins_source.py --upstream PRISTINE_CHECKOUT` 可重建并逐文件验证两项补丁。
