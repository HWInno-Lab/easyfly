# 第三方代码分类与迁移索引

2026-10-18：ALG-5 已分类 ONNX Runtime；ALG-8.1/8.2 已分类 FAST-LIO/EGO；ALG-8.3 已分类 Livox/RealSense SDK；ALG-8.4 已将 QGC 接入资料分类到 tools/qgc。下一步为 ALG-8.5 接入规范收敛。
ALG-0 migration-inventory.json 保留历史布局，不将历史清单改写为当前布局。
自有算法实现及包装见 `../algorithms/`；外部项目、库、SDK、模型各自保留原许可。

## 分类与目标

| 当前路径 | 目标路径 | 类型 / 批次 |
| --- | --- | --- |
| `algorithms/localization/fast_lio/` | 已完成 | 雷达惯性定位建图 / ALG-8.1；见 algorithms/localization/README.md |
| `algorithms/planning/ego_planner/` | 已完成 | 局部轨迹规划 / ALG-8.2；见 algorithms/planning/README.md |
| `runtimes/onnxruntime/` | 已完成 | 本地推理运行库 / ALG-5；见 runtimes/README.md |
| `sensor_sdks/livox_sdk2/` | 已完成 | 外部雷达 SDK / ALG-8.3；见 sensor_sdks/README.md |
| `sensor_sdks/librealsense/` | 已完成 | 外部相机 SDK / ALG-8.3；见 sensor_sdks/README.md |
| `tools/qgc/` | 已完成 | 外部应用接入资料 / ALG-8.4；见 tools/qgc/README.md |

第三方算法分类与自有算法一致：preprocessing、detection、tracking、localization、
fusion_mapping、planning。新项目按实际引入创建，不能以空目录表示已经集成。
`geographic_lib`、`nlopt`、`nlohmann_json` 当前为空，实际引入时分别归入
libraries/geometry、libraries/optimization、libraries/serialization。

每个项目保留一个主要源码位置，其他用途登记标签。FAST-LIO 内嵌的 ikd-Tree/IKFoM 和
EGO 内部规划/地图包随上游工程保留，不复制或按本项目功能类别拆散。
ROS 消息/设备适配器放 EasyFly 功能包；可复用普通计算包装放 algorithms/。系统依赖 OpenCV/PCL 不为了目录分类而额外 vendoring。
外部 ROS 驱动继续保留在通信层，尤其不顺带移动 RealSense 嵌套仓库。

## 已确认的路径和构建注意事项

- ALG-8.1 保留 FAST-LIO 完整 ROS 工程与包名，字节/来源登记见 algorithms/localization/fast_lio.provenance.json。
  FASTLIO_OUTPUT_ROOT 可隔离日志与 PCD；本机旧路径桥接仅用于运行中的旧安装，并以 COLCON_IGNORE 排除。
- ALG-5 已更新 ORT_ROOT 默认路径与忽略规则。easyfly_yolo_onnx 独占运行库/provider/许可安装，
  动态读取实际 SONAME，PIMPL 隐藏 SDK，支持外部 ORT_ROOT 和 $ORIGIN 安装 RPATH。
- ALG-8.2 已清除 EGO 的 PROJECT_ROOT_DIR/src/planner、相邻包源码和 plan_env_DIR include 猜测；
  使用导出依赖/target，普通复制安装搬迁后验证消费者与节点。完整8包及本地会话/停流补丁保留。
- Livox 驱动当前使用 `find_library(... /usr/local/lib)` 和 `find_path` 找 SDK，RealSense
  驱动使用 `find_package(realsense2 2.56)`。移动源码不表示消费者已经改用那份源码编译的库；
  验收需记录真实解析出的头文件和动态库，不覆盖系统安装来掩盖依赖问题。
- ALG-8.3 两个 SDK 的3422份受追踪文件整体分类且字节不变；外部ROS驱动/嵌套仓库保持。
  新目录的独立编译、默认系统SDK和显式私有前缀选择分别验证；旧build缓存保留忽略且不复用。
- SDK 构建产物、ONNX SDK 和 FAST-LIO 演示资源原有忽略规则应迁到新路径。
  不能复制旧 build/install 当作新目录构建结果。
- `QGC.AppImage` 是被忽略的用户本地文件，继续通过显式路径选择；此轮只迁移 qgc 资料，
  不自动移动、下载或替换该二进制。
  离线预览与联网维护的代码均保留在EasyFly UI层，分别遵守原有启动和会话规则。

## 来源登记要求

每个项目记录真实上游 URL、版本/tag/commit 或明确未知项、归档/文件 hash、许可和版权、
本地修改、构建/获取方式、消费者以及验证范围。现有详细来源记录在工作区本地
`docs/development/third-party-provenance.md`；未来完善的可复现登记随受追踪项目保存。
FAST-LIO/EGO 原始导入基点和部分许可仍有缺口，ALG-0 盘点与目录移动都不能补足这些证据。

实际迁移清单和指纹见 `../algorithms/migration-inventory.json` 与 `../algorithms/baseline/`。
运行库存在不代表模型、provider 或飞行能力已经就绪；能力由相应接入层报告。
