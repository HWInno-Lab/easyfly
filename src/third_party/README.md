# 第三方代码分类与迁移索引

2026-10-18 / ALG-0：本文件记录当前布局和目标分类，尚未执行目录迁移。
自有算法实现及包装见 `../algorithms/`；外部项目、库、SDK、模型各自保留原许可。

## 分类与目标

| 当前路径 | 目标路径 | 类型 / 批次 |
| --- | --- | --- |
| `slam/fast_lio/` | `algorithms/localization/fast_lio/` | 雷达惯性定位建图 / ALG-8.1 |
| `planners/ego_planner/` | `algorithms/planning/ego_planner/` | 局部轨迹规划 / ALG-8.2 |
| `onnxruntime/` | `runtimes/onnxruntime/` | 本地推理运行库 / ALG-5 |
| `livox_sdk2/` | `sensor_sdks/livox_sdk2/` | 外部雷达 SDK / ALG-8.3 |
| `librealsense/` | `sensor_sdks/librealsense/` | 外部相机 SDK / ALG-8.3 |
| `qgc/` | `tools/qgc/` | 外部应用接入资料 / ALG-8.4 |

第三方算法分类与自有算法一致：preprocessing、detection、tracking、localization、
fusion_mapping、planning。新项目按实际引入创建，不能以空目录表示已经集成。
`geographic_lib`、`nlopt`、`nlohmann_json` 当前为空，实际引入时分别归入
libraries/geometry、libraries/optimization、libraries/serialization。

每个项目保留一个主要源码位置，其他用途登记标签。FAST-LIO 内嵌的 ikd-Tree/IKFoM 和
EGO 内部规划/地图包随上游工程保留，不复制或按本项目功能类别拆散。
适配器放 EasyFly 功能包；系统依赖 OpenCV/PCL 不为了目录分类而额外 vendoring。
外部 ROS 驱动继续保留在通信层，尤其不顺带移动 RealSense 嵌套仓库。

## 已确认的路径和构建注意事项

- 检测包默认 ORT_ROOT 使用源码相对路径，并安装固定 SONAME/provider 库；移动 SDK
  必须与消费方、运行库安装责任和 `.gitignore` 一起修改。
- EGO `plan_manage/CMakeLists.txt` 通过 `../../..` 推导 PROJECT_ROOT_DIR，并追加
  install/src/planner 搜索路径。这些对原布局的假设必须在重定位时审计，不能只移动目录。
  bspline_opt 还有基于 plan_env_DIR 的 include 推导，需验证安装消费。
- Livox 驱动当前使用 `find_library(... /usr/local/lib)` 和 `find_path` 找 SDK，RealSense
  驱动使用 `find_package(realsense2 2.56)`。移动源码不表示消费者已经改用那份源码编译的库；
  验收需记录真实解析出的头文件和动态库，不覆盖系统安装来掩盖依赖问题。
- SDK 构建产物、ONNX SDK 和 FAST-LIO 演示资源原有忽略规则应迁到新路径。
  不能复制旧 build/install 当作新目录构建结果。
- `QGC.AppImage` 是被忽略的用户本地文件，继续通过显式路径选择；此轮只迁移 qgc 资料，
  不自动移动、下载或替换该二进制。

## 来源登记要求

每个项目记录真实上游 URL、版本/tag/commit 或明确未知项、归档/文件 hash、许可和版权、
本地修改、构建/获取方式、消费者以及验证范围。现有详细来源记录在工作区本地
`docs/development/third-party-provenance.md`；未来完善的可复现登记随受追踪项目保存。
FAST-LIO/EGO 原始导入基点和部分许可仍有缺口，ALG-0 盘点与目录移动都不能补足这些证据。

实际迁移清单和指纹见 `../algorithms/migration-inventory.json` 与 `../algorithms/baseline/`。
运行库存在不代表模型、provider 或飞行能力已经就绪；能力由相应接入层报告。
