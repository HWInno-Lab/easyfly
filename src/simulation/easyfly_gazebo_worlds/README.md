# 场景登记（S0–S2）

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
`evidence/<scene_id>/bootstrap.json` 安装轻量观测摘要，原始运行日志保存在工作区 artifacts。

`warehouse_aws_v1` 已登记支持 `navigation_dev`：采用显式室内地面空间支撑策略，
连续地图、静止坐标检查和中央通道 1.5m 往返、降落均实测通过。
最终版本两段到位误差约 0.075m / 0.066m，24 个进程正常退出，无残留。
证据为 `registration.json` 与 `mission.json`；旧 `bootstrap.json` 保留历史缺口。
此结论只覆盖该参考任务与记录的代码、配置，不代表整个仓库或长期飞行已验证。

无屋顶版本保持 draft，不能继承带屋顶版本的导航通过标记。
其历史 EGO 关闭阶段 exit -11 以及本轮导航协调节点关闭 exit -11 尚未定位；
定向调试取证未复现不代表修复。
导航使用会话 odom 坐标，参考变换需在静止后复核，检查器不把离线登记当作实时起飞许可。
可选 `navigation.ground_model_frame` 声明地面模型参考系（map/odom，省略为 odom）。
仓库声明 map 以稳定保存地面，要求新鲜连续的 SLAM TF；不覆写规划参数。
