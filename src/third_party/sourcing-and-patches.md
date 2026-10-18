# 来源、版本与本地补丁登记

当前入口是[catalog.json](catalog.json)，涵盖6个已分类项目、保留原位的2个ROS驱动和1组模型资产。
它记录当前算法分层涉及的边界，不是整个平台的完整软件物料清单，也不参与运行时后端发现。
项目维护的SORT/A*等实现由algorithms管理；系统OpenCV/PCL/Eigen不额外复制为vendored源码。

## 1. 当前来源与缺口

| 组件 | 可确认版本/身份 | 原许可证据与缺口 | 详细记录 |
| --- | --- | --- | --- |
| FAST-LIO及内嵌ikd-Tree/IKFoM | 完整本地ROS2导入；精确上游基点未知 | 根GPL-v2文本与包BSD元数据差异保留；嵌入组件基点未知 | [清单](algorithms/localization/fast_lio.provenance.json) |
| EGO完整8包 | 本地ROS2导入，原上游/fork/commit未知 | TODO/MIT/BSD包元数据及文件通知保留；没有统一根许可 | [清单](algorithms/planning/ego_planner.provenance.json) |
| ONNX Runtime | 本地SDK元数据1.23.2，GIT_COMMIT_ID有记录 | MIT与ThirdPartyNotices；原下载归档/签名未保存 | [清单](runtimes/onnxruntime.provenance.json) |
| Livox SDK2 | 公共头/运行库1.3.0；CHANGELOG1.2.5、CMake变量0.0.2分别保留 | LICENSE.txt按列出的范围授权MIT；准确导入commit未知 | [清单](sensor_sdks/livox_sdk2.provenance.json) |
| librealsense | 公共头/包/运行库2.56.5 | Apache-2.0、NOTICE及内嵌依赖各自通知；导入commit未知 | [清单](sensor_sdks/librealsense.provenance.json) |
| QGC资料/本机样本 | 文档不含源码/程序；同一SHA样本此前观察为5.0.8 | 嵌入GPL-3.0元数据不等于下载来源认证；不覆盖其他选定文件 | [清单](tools/qgc/provenance.json) |
| Livox ROS Driver2 | package.xml版本1.0.0，与SDK版本分开 | 原LICENSE.txt的MIT文件范围；上游导入基点未知 | [原许可](../communication/c1_drivers/livox_ros_driver2/LICENSE.txt) |
| RealSense ROS | gitlink/本地HEAD为d69f05c1；本地适配提交 | Apache-2.0；根.gitmodules映射缺失，不能当作可自动恢复的上游锁 | [原仓库](../communication/c1_drivers/realsense_ros/README.md) |
| 模型资产 | models中ONNX/PT两份文件以SHA固定 | 原下载来源、导出命令、训练与许可证据未登记，不从文件名或运行库许可推断 | [清单](model-assets.provenance.json) |

目录中有源文件、SDK或权重只表示文件可见；编译、动态加载、模型/provider和设备就绪分别验证。
SDK GIT_COMMIT_ID、本地导入提交、原始归档SHA、签名认证是不同证据，不能相互替代。
模型是独立资产：默认ONNX由easyfly_object_detection安装，ORT运行库由easyfly_yolo_onnx安装。

## 2. 已知本地变更入口

下列是已确认的本地历史线索，包含导入、路径、构建和接入变更，不能声称是相对于未知上游的完整patch集。

| 范围 | 本地提交/记录 | 变更边界 |
| --- | --- | --- |
| FAST-LIO | 7455e06、b52a4fd、c3e41af | 导入/保护；ALG-8.1目录分类、隔离输出根及许可安装；算法文件身份见清单 |
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
ALG-8分类/文档工作完成，ALG-9整体构建与受管场景回归尚未完成；
ALG0-B03、历史semantic shutdown/stress及来源/许可缺口继续保留。
