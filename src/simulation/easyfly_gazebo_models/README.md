# 现有模型与资产快照（S0）

保留现有模型根路径，新增 `manifests/` 记录资源集合，避免全目录安装缓存或开发产物。
清单格式定义在 world 包的 `schemas/asset-set-v1.schema.json`。

每个集合记录一个直接包含模型目录的 `model_root`、各模型的入口、逐文件 SHA256、
来源/许可状态和修改记录。`files` 是本轮文件快照；未登记的新增文件或哈希变化会报错。
`storage: existing` 表示原已存在的资源，不表示本轮导入或完成来源审查。
S0 保留原模型、纹理和材质；S2 的运行模型改动单独记录如下。

集合包括场景模型、MID360、默认 Iris、Gazebo 内置 sun/ground_plane、PX4 GPS。
外部集合仅登记文件指纹，不将外部资源复制进本包；路径由调用者明确提供。

当前 package.xml 声明 MIT，但不能据此确认全部 mesh/texture 的许可。
遗留模型集合与逐模型 `provenance.status` 均保留为 `unverified`，并引用
`licenses/legacy-audit.md` 中的待办；不新增猜测的第三方 LICENSE。

遗留问题包括消防车的 `suv` 配置/材质引用，以及高级 OGRE 脚本的深层资源审计。
校验器应报告实际缺口，不删除依赖、不放宽规则来产生 PASS。

## S2 独立资源集

`third_party/aws/warehouse/` 是固定 MIT-0 仓库快照的 13 个模型。
`licenses/aws_warehouse/` 保存原许可与 81 份 world/model 文件的上游、本地指纹；
未引用 DeskC 与 9 个 `.DS_Store` 不安装。可用 `tools/scenes/import_aws_warehouse.py`
从固定指纹的本地归档复现导入。

`easyfly/runtime_models/` 仅含原版 PX4 GPS 与项目 MID360 的圆柱外观版本。
MID360 传感器、IMU、质量和惯量保持原值；来源不清的外观 mesh 没有导入该集合。
`easyfly_airframe_v1` 引用项目维护的 Iris；其 SDF 显式修正非零 IMU 噪声密度，
并接入 1000Hz 原始样本的四点均值与 PX4 FIFO 量程限制。详见
`licenses/runtime/imu-noise-reference.json` 和 `imu-sampling-reference.json`。
原始 `/imu_native` 保留，`/imu` 供原 250Hz MAVLink 采样使用；不改 PX4 参数。
`gazebo_builtin_v1` 单独引用系统模型。四个新增清单有独立来源及版权证据，
避免同时暴露完整 PX4 与系统同名模型。`storage: vendored` 标明本轮提取资源。
整理脚本 `tools/scenes/prepare_runtime_assets.py` 核对指定 PX4 修订与原 mesh 指纹，
拒绝覆盖已有整理目录。旧模型及旧清单的许可缺口继续保留。

## S3 住宅与书店资源集

`third_party/aws/house/` 与 `third_party/aws/bookstore/` 分别包含固定官方提交的
63/26种MIT-0模型，布局中的多个实例复用同一模型目录。分类与场景任务在world包登记。
对应`licenses/aws_house/`、`licenses/aws_bookstore/`保留原许可、原始world、逐文件
上游/本地指纹与修改记录；可用`tools/scenes/import_aws_indoor.py`从固定本地归档复现。

住宅补齐实际引用的18张源JPEG（19份模型内副本），调整COLLADA图像路径；
图像字节与mesh几何数组保留。住宅world字节保留，所有布局实例保持static。
书店修正SDF路径协议和Spotlight mesh文件名大小写；world只移除高度约343km的
异常静态BookE_01_009，原始world与差异保留。未引用模型、照片及上游启动代码不安装。
模型目录共约126MB；下载归档及完整诊断点云保存在安装目录外。
## Optional archive cache (S5)

`manifests/aws_house_v1.yaml` also locks a data-only recipe for the original
MIT-0 AWS house archive at revision `ff9631ca6d1db9c1ba656498151464b5ab74aafe`.
The recipe reproduces all 363 reviewed model files, including the existing
model-local portrait photo relocations. It does not change any model bytes,
geometry, original license or the default vendored resource selection.

Prepare it with `ros2 run easyfly_sim_launcher scene_catalog fetch house_aws_v1`.
Select it with `EASYFLY_SCENE_ASSET_MODE=cache`; optionally set the absolute
`EASYFLY_ASSET_CACHE_ROOT`. Cache storage is external, versioned and validated;
it is not copied by this package's install rules. Future `storage: cached`
assets can ship only a locked manifest and license evidence. Their payloads
must be prepared explicitly before launch. See easyfly_sim_launcher/README.md
for offline import, validation, interruption and version behavior.

`tools/scenes/lock_aws_house_archive.py` is a maintainer's read-only-source recipe
builder/checker against the already reviewed archive and origin records. Runtime
fetch consumes its declarative output and never executes this or upstream code.
