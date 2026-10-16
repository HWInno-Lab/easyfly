# 场景登记（S0–S3）

现有 `empty.world`、`perception_demo.world`、`precision_landing.world` 保留根路径。
`scenes/` 是场景登记，`schemas/` 使用 JSON Schema Draft 7，兼容 Jammy 的 python3-jsonschema。
YAML 是数据，不执行脚本；禁止重复键、锚点、别名和非有限数值。

## 字段与状态

- `id` 为稳定标识；`category` 是一个环境主分类，跨用途通过 `tags` 表达。
- `world` 指向包内相对文件，并锁定 SHA256；目录调整不意味着重新验证。
- `asset_sets` 引用 model 包 `manifests/` 中的集合。
- `runtime` 单独登记默认飞机、传感器、系统插件及额外资源；不覆盖独立 D5 fixture。
- `legacy_profiles` 仅记录既有代码对应关系；`supported_profiles` 是本轮已验证的兼容范围。
- `spawn` 使用 world 坐标。`basis` 说明来自代码或实测；未知值为 null。
- `navigation` 使用 odom；未测量的 world/odom 变换和参考范围为 null。
- `provenance` 分开记录声明、证据和核实状态。world 许可不覆盖所引用模型。
- `verification` 是对应修订的历史记录，不是实时起飞许可；具体状态以每份清单及证据为准。

`draft` 清单可登记但不能宣称可启动。S1 已接入公开
`scene` 参数与 ROS list/validate 命令，草稿选择会被拒绝。当前 `precision_landing` 未填写出生点，
保留独立 D5 包装的原行为。

## 离线校验核心

`easyfly_px4_sitl.scene_catalog.SceneCatalog`（保留 launcher 兼容导入）接收显式的 `package_shares`、
`external_roots`、`plugin_dirs`。`validate(id).as_dict()` 分别报告：

1. `scene_resources_complete`：world 和场景模型资源检查。
2. `runtime_resources_complete`：默认飞机、传感器及场景资产组合检查。
3. `licenses_complete`：来源/许可登记是否齐备。
4. `registration_complete`：出生点、对齐与 profile 等登记是否完成。

`resources_complete` 是场景与默认飞机两类资源检查的合取；不能仅凭
`runtime_resources_complete` 判定整个场景完整。

`runtime_verified` 始终为 false。格式错误在加载时抛出 `CatalogError`；
资源或登记缺口汇总在报告中，不改写 YAML 验证状态。

包 share 与外部根由调用者信任；检查器不读取任意环境变量、不联网、不执行插件。
对 symlink-install 中指向源码的文件，调用者必须显式传入对应包源码目录到
`trusted_sources`，不能把整个工作区作为通用越界例外。
安装适配器通过 ament 的 package.xml 源码链接和包名核对识别单个受信源码包。
外部根使用 `gazebo_share`（如 `/usr/share/gazebo-11`）、`px4_models`
（当前 PX4 Classic models 目录）。这些只用于文件检查，不替代 PX4 兼容性验证。

支持 SDF/world/model.config URI、DAE image 引用、OBJ/MTL 材质引用和模型依赖。
相对纹理候选仅为源文件目录及所属模型的 `materials/textures/`；多候选拒绝歧义。
OGRE 仅解析 SDF 明确选中的简单 material 块及其纹理引用；继承、shader/program、
动画纹理等高级语法或未指定 material 名称时报告“依赖审计待完成”，不以文件存在冒充闭包完整。
检查插件文件存在，不验证其 ABI、动态库传递依赖或运行输出。

`licenses/PROJECT-LICENSE` 是根项目 LICENSE 的原文副本，适用于本项目 world
的声明记录；其他资产仍按各自来源登记。

## AWS 仓库（S2）

`worlds/indoor/warehouse/` 保存固定上游版本的完整屋顶和无屋顶 world，
分别登记为 `warehouse_aws_v1`、`warehouse_aws_no_roof_v1`，两者均保持上游原字节。
模型在 model 包的 `third_party/aws/warehouse/`，MIT-0 原文与逐文件来源锁单独保留。
两个场景均登记支持 `navigation_dev`，显式出生点为 world `(0, 0, 0.10)`，yaw=0。
原始日志、ULog 和完整点云保留在工作区 artifacts；安装包只保存轻量证据。

导航任务使用会话 odom 坐标，地面模型使用 map 坐标；实时检查要求新鲜的 SLAM TF。
每张地图的 `calibration-s2-final.json` 记录两次独立冷启动、静止坐标测量、
传感器及七个组件观测、实际运行库指纹，以及每次 24/24 子进程正常退出。
两个场景分别测量，不互相继承通过记录。离线校验始终不构成实时起飞许可。

参考任务安装在 `easyfly_mission_scripts/missions/scenes/<scene_id>/aisle_out_and_back.yaml`：
起飞至 1.5m、悬停、沿中央通道前往 `(1.5, 0, 1.5)`、返回 `(0, 0, 1.5)`、降落。
连续任务验收要求同一仿真内执行两轮；每轮检查真实导航结果、终点误差、会话和所有权释放，
并在降落后连续检查静止状态、地图新鲜度和 map 中的返回位置。
场景 `verification` 记录当前修订已取得的证据；正常退出也必须通过才可签导航验收。

2026-10-16 的 v10 正式矩阵中，两场景各两次独立冷启动及各两轮连续任务全部通过，
`navigation_tested` 均为 true。每个运行的 24 个进程正常退出，Gazebo IO 管理器归零，
无强制清理或所属进程残留。`mission-s2-final.json` 保存真实任务/导航 UUID、终态、
到位误差、落地恢复、所有权释放及运行库指纹，并与冷启动证据交叉核对。

历史失败仍保留：v8 屋顶两轮任务成功但 Gazebo 退出残留，v9 冷启动 bridge 退出崩溃；
两者均未计入通过成绩。已复现并修正的原生缺陷不能唯一解释所有旧失败，
本次签收只说明 v10 所记录代码、配置和中央通道任务满足验收门限。

`evidence/warehouse_aws_v1/native-fixes-s2.json` 汇总原生传感器、ODE 射线、传输和 DDS
生命周期修复及对应回归。模型包的 `imu-noise-reference.json` 与 `imu-sampling-reference.json`
分别记录非零噪声密度、原生采样和 FIFO 限幅。调试器运行只用于定位问题，不作为正式通过证据。
历史 `bootstrap.json`、`mission.json`、`mission-imu-density.json` 等保留其各自版本结论，
不覆盖当前清单。任何有界任务通过也只覆盖所记录路线、代码和配置，不代表整个仓库已验收。

## AWS 住宅与书店（S3）

`house_aws_v1`归类为`indoor/residential`，`bookstore_aws_v1`归类为`indoor/commercial`。
模型与运行传感器分开管理，继续复用同一Iris、GPS、MID360及相机配置。
每张图有独立的来源、几何测量、出生位置、静止坐标标定和参考任务证据。

住宅出生点为客厅world `(1,0.8,0.10)`；书店为北侧宽横通道`(-3,5.4,0.115)`。
`geometry-s3.json`记录真实Gazebo/ODE碰撞测量：每条候选路线17个位置，
40个高度（0.1～4.0m）、32个方位，以及地面和向上射线。书店的细书架通道未纳入参考路线。

两图资源闭包与原生几何检查已通过，并登记支持`navigation_dev`。
`navigation.grid_footprint_mode: observed_cells`显式为这两图选择可信非地面点足迹，
保留0.7m膨胀、半体素裕量、持久化及原有失效检查；其他清单默认仍为簇包框模式。
初次任务被包框占据拒绝的证据保留为`mission-s3-rejected-v1.json`。
原生心跳由清单显式选择5Hz，并在同一自有PX4的实际状态输出中确认；
控制器的3墙秒解锁确认截止时间保持不变。
2026-10-17 的 v7 正式矩阵中，两图各两次独立冷启动及各两轮连续任务全部通过，
`navigation_tested`均为true。每张图有四个成功导航UUID、两次落地恢复，
到位误差均小于0.15m；六个运行共144次进程正常退出，launch退出0、Gazebo IO归零，
无强制清理或所属进程残留。`calibration-s3-final.json`和`mission-s3-final.json`
逐图记录真实状态、UUID、传感器、运行库指纹及原始证据位置，普通安装与软链接安装分别核对。

早期失败和修订前标定保留。两次退出段错误通过kernel故障PC及真实端点并发用例
定位到Fast DDS读端discovery监听器竞争；`dds-discovery-fix-s3.json`记录精确复现、
修复与六条并发路径回归。另一次Gazebo rendering启动异常尚未取得原生栈完成归因，
不归入DDS修复结论；原失败仍保留。v7通过仅覆盖所记录环境及客厅/宽横通道参考路线，
不代表窄书架通道、穿门、整张室内地图或长时间运行已验收。

`scene_sitl --bootstrap`仅收集无飞行Goal的传感器/静止证据；已注册场景使用受管入口，
草稿检查才显式传入raw-world、受管模型根与几何测量出生点。
