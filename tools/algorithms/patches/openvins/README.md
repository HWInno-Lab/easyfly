# OpenVINS ROS2 生命周期兼容补丁

基点：官方 `rpng/open_vins` 提交 `69488123ed9362dd44b6f28e7f4680abbff1442b`。
应用顺序：`0001-ros2-visualizer-lifetime.patch`、`0002-install-notices.patch`、
`0003-owned-update-and-init-workers.patch`、`0004-zupt-update-readiness.patch`、
`0005-source-stamped-tracking-diagnostics.patch`、`0006-strict-native-input-boundary.patch`。

`0006` 为显式单目仿真导航增加默认关闭、启动后只读的原生接收检查。启用时使用
有界可靠 IMU 队列 1000、图像队列 32；在滤波前检查原有 25 ms / 250 ms 源间隔、
500 ms 源年龄/接收停顿及 200 ms 未来容差。缺样、重复或过期正常结束定位上下文，
要求重启完整定位会话，不补齐、外推或改写传感器。观测/上游独立入口默认行为不变。
发现期间尚未接纳该流且算法未初始化时，丢弃过期 writer 历史；不将它送入算法或
建立流时期。接纳过数据后，同样错误仍结束会话，没有运行期重新择窗或恢复旧状态。
原生暂停测试曾复现 SensorDataQoS 的五条 IMU 历史丢样，恢复后进入 Propagator
完整积分时长断言；新边界在状态计算前拒绝这些数据，保留上游断言和滤波计算。

`0005` 在已有测量更新线程中比较真实相邻图像的特征 ID，发布保留特征数和初始化状态，
使用该图像的原始 IMU 时基源时间。诊断不改写滤波、特征或测量，不自行授予导航权限。

`0004` 仅在真实零速度滤波更新已成功后更新最后更新时间。上游 `initialized()` 同时
检查该时间；此前仅普通视觉更新写入它，导致静止初始化后持续成功的 ZUPT 无法发布
已经计算出的状态及协方差。补丁不改变初始化判断、测量、噪声、状态值或协方差。
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

生命周期补丁 `0001` 改变四个文件：

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
`verify_openvins_source.py --upstream PRISTINE_CHECKOUT` 可重建并逐文件验证完整补丁链。
