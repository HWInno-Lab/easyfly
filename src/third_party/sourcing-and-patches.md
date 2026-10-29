# 来源、版本与本地补丁登记

更新日期：2026-10-28。当前入口是[catalog.json](catalog.json)，共 15 个登记对象：12 个第三方项目、保留原位的 2 个 ROS 驱动和 1 组模型资产。
它记录当前算法分层涉及的边界，不是整个平台的完整软件物料清单，也不参与运行时后端发现。
项目维护的SORT/A*等实现由algorithms管理；系统OpenCV/PCL/Eigen不额外复制为vendored源码。

## 1. 当前来源与缺口

| 组件 | 可确认版本/身份 | 原许可证据与缺口 | 详细记录 |
| --- | --- | --- | --- |
| FAST-LIO及内嵌ikd-Tree/IKFoM | 官方 ROS2 59b082f0＋精确 ikd-Tree e2e3f4e9 可逐字节重建；历史 checkout 非唯一 | 包元数据已记录根 GPL-v2 与文件 BSD 通知；旧 ikd-Tree 缺独立许可仍保留 | [清单](algorithms/localization/fast_lio.provenance.json) |
| OpenVINS | 官方提交 `69488123ed9362dd44b6f28e7f4680abbff1442b`，运行所需三包及配置子集 | 原 GPLv3 文本、源文件 GPLv3-or-later 通知保留并安装；六项有序补丁（生命周期、许可安装、工作线程、ZUPT 就绪、源时间诊断、原生输入边界） | [清单](algorithms/localization/open_vins.provenance.json) |
| EGO完整8包 | 原本地 fork b757687b 的 102 个 Git 文件＋7 个工作区文件与首次导入完全一致；原始选集及补丁随源码交付 | 用户已确认 ROS 2 移植作者身份；恢复上游 GPLv3 文本，逐文件登记旧版权头检查范围；移植部分公开授权选择待确认 | [清单](algorithms/planning/ego_planner.provenance.json) |
| FAST-LIVO2 | 固定 ROS2 导入 `837b7bbc1431cb04cf936528e52c83c835efba8e` | VIKIT 的 BSD-only 元数据已按同一分叉父提交及文件原声明纠正，完整 GPLv3／LGPL2.1／MIT 文本随包安装；FAST-LIVO2 GPLv2 与 VIKIT GPLv3 的组合分发兼容范围仍待闭环 | [清单](algorithms/localization/fast_livo2.provenance.json) |
| ORB-SLAM3 / Pangolin | ORB 固定提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`，选定输入可重建 | ORB GPLv3；DBoW2 文本已从其指定 ORB2 来源恢复并安装，Pangolin 分别登记 | [清单](algorithms/localization/orb_slam3.provenance.json) |
| GCOPTER | 官方提交 `e0444f6d47b84f972ced91746b05feb36ce1fd4f` | MIT，原文保留安装；一项 C++17 比较器补丁 | [清单](algorithms/planning/gcopter.provenance.json) |
| SUPER | 官方提交 `2ad3419c127a617c6d7df6925e81a14175a9c096` | 当前包按源文件 LGPL-3.0-or-later 记录，完整 GPL／LGPL 文本随包安装；原上游 BSD 元数据留在原始归档 | [清单](algorithms/planning/super.provenance.json) |
| FUEL | 官方提交 `662dd23c7b52b258d3c4a0155ff6632118e8984f` | 保留主项目及嵌入 LKH 各自通知，LKH 元数据／分发范围另列 | [清单](algorithms/planning/fuel.provenance.json) |
| ONNX Runtime | 官方 1.23.2 归档重新获取，全部 32 项载荷与本地一致 | MIT 与 ThirdPartyNotices；记录官方 URL／归档 SHA，未声称取得独立归档签名 | [清单](runtimes/onnxruntime.provenance.json) |
| Livox SDK2 | 原本地 MID-360S SDK_ROS 目录的 220 个文件完全一致，可从随附选集重建；三种版本字段分别保留 | LICENSE.txt按列出的范围授权MIT；准确导入commit未知 | [清单](sensor_sdks/livox_sdk2.provenance.json) |
| librealsense | 3,221 个现有文件与官方 v2.56.5／38a41441 完全一致；13 个未交付 CI 文件明列 | Apache-2.0、NOTICE 及内嵌通知保留；当前源码基点已验证，历史导入日志未知 | [清单](sensor_sdks/librealsense.provenance.json) |
| QGC资料/本机样本 | 文档不含源码/程序；同一SHA样本此前观察为5.0.8 | 嵌入GPL-3.0元数据不等于下载来源认证；不覆盖其他选定文件 | [清单](tools/qgc/provenance.json) |
| Livox ROS Driver2 | package.xml版本1.0.0，与SDK版本分开 | 原LICENSE.txt的MIT文件范围；上游导入基点未知 | [原许可](../communication/c1_drivers/livox_ros_driver2/LICENSE.txt) |
| RealSense ROS | 官方父提交 `bafc21080c5c8e259dadbb309797949aee0dd950`＋本地构建适配 `d69f05c1` | Apache-2.0；完整 Git bundle、明确官方远端及离线恢复工具固定准确 gitlink，不依赖官方远端存在本地提交 | [恢复说明与清单](drivers/README.md) |
| 模型资产 | PT 与官方 YOLO11n v8.3.0 发行资产逐字节一致；ONNX 等价重导出已验证 | AGPL-3.0 文本、COCO 类别与导出配置随包交付；历史命令／训练日志不可追认 | [清单](model-assets.provenance.json) |

目录中有源文件、SDK或权重只表示文件可见；编译、动态加载、模型/provider和设备就绪分别验证。
SDK GIT_COMMIT_ID、本地导入提交、原始归档SHA、签名认证是不同证据，不能相互替代。
模型是独立资产：默认ONNX由easyfly_object_detection安装，ORT运行库由easyfly_yolo_onnx安装。

## 2. 已知本地变更入口

下列是已确认的本地历史线索，包含导入、路径、构建和接入变更，不能声称是相对于未知上游的完整patch集。

| 范围 | 本地提交/记录 | 变更边界 |
| --- | --- | --- |
| FAST-LIO | 7455e06、b52a4fd、c3e41af | 导入/保护；ALG-8.1目录分类、隔离输出根及许可安装；算法文件身份见清单 |
| FAST-LIO 本地退出修复（2026-10-26） | [原生信号回归](algorithms/localization/fast_lio/test/test_signal_shutdown.py)、[说明](algorithms/localization/fast_lio/README.md) | 删除在 POSIX SIGINT 回调中同步调用 ROS shutdown 的处理器，保留 rclcpp 专用信号线程；真实旧进程持锁中断死锁对照，当前两安装通过。未修改估计或传感器数据；小时故障的具体锁未捕获；后续 V-08 普通完整小时复验已通过，不追认该历史事故唯一同因。 |
| FAST-LIO 地图发布修复（2026-10-26） | [点集回归](algorithms/localization/fast_lio/test/test_map_snapshot.cpp)、[范围说明](algorithms/localization/fast_lio/README.md) | 用原生局部 ikd-tree 的互斥快照替换重复追加扫描；后台重建忙时跳过可视化发布，不阻塞定位。地图保存改为最近的局部快照，传感器历史需显式录制。两安装原生回归和实际地面输出对照已完成，修复后 V-08 第三轮普通完整小时通过；保留前两轮原始失败。 |
| EGO | 7455e06及来源清单中的本地安全/会话/轨迹历史；6d4115e | 保留既有补丁；ALG-8.2仅目录、构建导出和测试路径调整 |
| ORT包装/部署 | [ALG-5](../algorithms/alg5-validation.md)，54dca69、ecaa432 | EasyFly封装、provider/SONAME/许可安装及本地说明；不称为SDK算法补丁 |
| 两份SDK/驱动 | c707cd3；5d84150 | 原导入；SDK文件全部原样分类，驱动/嵌套仓库保持 |
| QGC接入资料 | 01de492、2b69165 | 本地UI接入与文档分类；用户二进制未改 |

路径移动后的定位例子：

```bash
git log --follow -- src/third_party/algorithms/localization/fast_lio/CMakeLists.txt
git show b52a4fd -- src/third_party/slam/fast_lio
git show 6d4115e -- src/third_party/algorithms/planning/ego_planner
git -C src/communication/c1_drivers/realsense_ros show d69f05c1
```

历史提交要使用当时路径；不修改ALG-0清单或旧证据假装一直是新目录。
本机旧FAST-LIO/EGO软链接仅供仍运行的旧安装使用，被Git忽略并由COLCON_IGNORE排除。
不把桥接作为新消费者接口，也不在当前会话中为“清理目录”而删除它。

## 3. 新项目的登记字段与更新规则

增加一个外部项目时，在主要目录旁保存来源清单，再在catalog追加一条记录；多用途通过tags表达，不复制源码。
至少记录主要路径/类别、真实上游URL、精确revision或明确unknown、版本证据类型、下载归档及解包文件SHA、
许可证文件和内嵌依赖范围、本地修改、安装消费者、配置选择、获取/构建命令及实际验证证据。
模型还应记录导出来源/设置和标签；SDK/工具要区分本地可选载荷与仓库内可重建输入。
unknown字段使用null并说明原因，不填写猜测的tag、提交、许可或下载校验和。

有可信上游基点时，保存可重放patch并记录应用顺序、基点/patch SHA及验证结果。
没有上游基点时，保存导入文件清单和本地提交，只称“已知本地修改”；逐步补证，不能生成虚假的完整上游差分。
构建补丁和算法/行为增强分别登记；升级依赖或改变默认行为单独评审和验证。

更新清单时先核对源码/载荷身份，再更新版本与证据，不通过改SHA掩盖未说明的代码变化。
算法封装保留原通知，自有文件的Apache-2.0不重新标记外部文件或权重。
发布/重建还需覆盖完整依赖、许可证范围、嵌套仓库初始化及模型获取缺口；本批分类不会自动解决这些缺口。

## 4. 兼容与剩余验收

[兼容层审计](../algorithms/compatibility-audit.md)登记旧公开头和本地桥接为何保留。
已有转发头、包装和平台接纳有不同职责，不按“文件同名”删除。
[新增算法指南](../algorithms/integration-guide.md)给出跟踪器/规划器的实际改动边界。
ALG-8 分类与 ALG-9 两种安装构建均有历史记录；当前软件／仿真结论以 2026-10-25／26 的 V 轮为准。
B-06／B-10／B-11 等已完成对应 DDS 缺陷对照与修复；部分独立历史异常、硬件及来源／许可缺口继续保留。
具体见[当前未完成清单](../../未完成.md)，不将历史失败或未知来源改写为已确认。
