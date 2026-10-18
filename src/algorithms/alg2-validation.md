# ALG-2 SORT 与 ID 整合迁移验收

日期：2026-10-18，原生产源码基线 `bdbc34c`。本批迁移跟踪计算并接入现有 ROS 节点，
ALG-3 的 A* 迁移尚未开始。

## 1. 实际结构

| 部分 | 当前归属 | 内容 |
| --- | --- | --- |
| SORT 核心 | `tracking/easyfly_sort` | 原卡尔曼、匈牙利、IoU/中心距离关联、生命周期 |
| 世界坐标 ID 整合 | `tracking/easyfly_track_id_fusion` | 类别/距离/TTL 墓园与稳定 ID 映射 |
| 统一接口 | `tracking/easyfly_tracking_api` | ALG-1 的 Tracker、Frame、UpdateResult |
| 算法适配与工厂 | `easyfly_multi_object_tracking/src/adapters` | 普通类型转换、校验、reset 和唯一 raw ID |
| ROS 功能节点 | 原 `tracking_node.cpp` | 消息、配置、类别过滤、缓存、整合、发布和调试图 |

两个新算法包只依赖标准库，导出 namespaced C++17 INTERFACE target。
SORT 保留原实现计算；旧包四个算法头现在只包含新头和 using 别名，没有第二份算法。
适配器导出普通共享库，用标准 CMake find_dependency 声明其公开算法依赖；独立消费不需
初始化 ROS/ament，旧 include 变量和公开头路径继续提供。

原 SORT 的 6 条用例、原 ID 的 10 条用例随实现迁移。ID 新增 5 条回归，适配器新增 5 条。

## 2. 行为边界及必要修复

### 2.1 保持的正常行为

原 YAML 的 .15 IoU、45 丢失帧、min_hits=3、max_tracks=100、距离门限、类别列表、
半径 3m、TTL 10s 继续生效。原 C++ 默认值也保持。原 raw/stable/调试话题、QoS、
归一化框和 px/frame 速度含义不变，三维定位仍消费 `/tracks_raw`。

### 2.2 延迟世界坐标回流修复

原整合器仅在新 raw ID 第一次出现时查墓园。但真实链在第一次 raw 发布后，才能由
pixel_to_3d 返回该 ID 的世界坐标；下一帧已存在的自身映射只刷新坐标，不再关联。
使用原 HEAD 头文件独立编译的复现程序得到新 ID `12`，未继承期望的旧 `7`。

新实现在“自身映射且从未取得世界观测”的情况下，允许首次回流时做一次原墓园匹配。
已有观测身份或已继承旧 ID 的轨迹不再分配。旧类别、半径、TTL 和墓园移除规则继续使用；
新增过期与不同类别的延迟反例。相同复现得到 `7`。
这是明确的功能修复，不将其混称为纯目录重定位或所有旧输出都不变。

### 2.3 接口与节点检查

新增只读启动参数/YAML `tracker_backend`，当前仅 sort，未知后端启动失败。
新增 `~/reset` 清除算法、整合器、图像和世界缓存并递增内部 epoch。
因 ROS TrackInfo 没有 epoch 字段，原始 ID 在同一进程内跨 reset 不复用，避免旧坐标别名。
ID 空间耗尽会拒绝，不能通过 reset 重新复用消息 ID。

新 API 检查非正、重复/回退时间、无效几何/分数与资源上限，拒绝时发布空 raw/stable，
不推进成功时间戳或 KF 步长。后端数值结果异常会清除相关历史。
回流检查 frame/时间一致性、重复 ID 和有限有效坐标；world_frame 默认 odom，可随定位配置调整。
没有增加世界观测时效认证或上游重放认证，进程重启不由内部 epoch 提供跨进程唯一性。

## 3. 实际验证

完整命令、输入 hash、JUnit 和日志位于
`artifacts/algorithms/alg2-20261018/`，最终安装矩阵位于 `install-validation-final/`。
受追踪的最终摘要为 [alg2-results.json](baseline/alg2-results.json)。

| 验证 | 结果 |
| --- | --- |
| 200 帧原 HEAD 与迁移核心精确对照 | 84,522 字节 trace 完全相同，覆盖运动、遮挡、删除、类别与再出现 |
| 原生核心与 API 适配器 100 帧对照 | 框、原始框、ID、类别、置信度、速度和生命周期逐字段一致 |
| 原延迟 ID 回流反例 | 旧输出 12，新输出 7，单独记录修复 |
| SORT 6 条、ID 整合 15 条、适配器 5 条 GTest | 两种安装分别通过 |
| 真实 tracking + pixel_to_3d ROS 回归 | 两种安装分别通过，包含未知后端、真实反投影、延迟继承、reset 与重复帧 |
| 纯 CMake 已安装库消费 | 两种安装通过，消费者不加载 ROS overlay、不链接 ROS 动态库 |
| 普通安装的 target/库/头文件重定位 | 通过；不扩大为所有 shell setup 脚本可重定位 |
| 跟踪包 8 项 lint | 两种安装分别通过，无跳过或文件排除 |
| pixel_to_3d 原 frame 契约 2 条用例 | 通过，定位节点源码未修改 |

每种安装 26 条 GTest、1 项真实 ROS 回归、1 项独立消费测试，共 28 次；两种安装 56 次，
另有 2 条定位 frame 用例，共 58 次用例/消费检查。lint 单列为两种安装合计 16 项。
按 CTest 注册项计，最终两种安装各 13 项，加定位 1 项，共 27 项。
ROS 用例在域 213 使用合成相机内参、2m 深度和静态 TF，验证目标位置 `(8,2,2)`、源时间
与 odom header、raw ID 回流和稳定 ID；两节点均须正常退出，强制清理和异常退出不能算通过。

包发现由 74 增至 76。两个核心与跟踪 API 的源码依赖闭包只包含自身三包，
未引入 PCL、OpenCV、推理库或 ROS 运行依赖；节点仍按原职责依赖 OpenCV 画调试图。

## 4. 迭代与证据

首轮包检查暴露许可头、include 顺序和原 launch 文档标点问题，已补完整 Apache 许可头并修正。
首次普通消费暴露旧 ament 导出宏会初始化 ament Python 构建环境；改为标准 CMake 依赖导出后解决。
历史日志及中间安装矩阵保留。最终补充延迟 TTL/类别反例及节点退出断言后，执行完整最终矩阵。

没有放宽算法阈值、等待时间或测试断言获取通过。原基线的 FAST-LIO HRT 前提与冻结时钟停流
失败不涉及本批源码，未修复或重报通过；没有启动 PX4/Gazebo、发送飞行任务或认证实际传感器精度。

## 5. 复验与下一步

先按清单在独立 prefix 构建跟踪 API、两个核心、跟踪节点和 pixel_to_3d，再运行：

```bash
python3 tools/algorithms/verify_tracking_install.py \
  --symlink-prefix artifacts/algorithms/alg2-20261018/install \
  --output artifacts/algorithms/alg2-new-run
```

工具拒绝覆盖历史证据。正常安装运行时使用当前明确的消息/定位 underlay，随后只用已安装
CMake target 验证独立消费；不覆盖主工作区安装或停止用户会话。
下一阶段 ALG-3 迁移二维 A* 和覆盖路线，预算/取消增强仍须独立验证。
