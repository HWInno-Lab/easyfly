# 现有模型与资产快照（S0）

医院资源草稿 `aws_hospital_v1` 选择固定 AWS 提交中的 26 个模型，按 MIT-0
单独记录。43 个 Fuel 模型各有独立的缓存清单、版本、归档和逐文件 SHA256；
逐项 API 原文均声明 CC-BY-4.0，保存在 `licenses/hospital_fuel/metadata/`。
`ATTRIBUTION.md` 与 `origins.json` 保留原作者、描述和第三方来源署名。
这 43 份 ZIP 中没有 LICENSE，缓存 `LICENSE` 保存显式锁定的外部发布者声明，
不套用 AWS 许可或旧下载脚本生成的 CC-BY-3.0 数据库声明。
6 份 AWS model.config 仅转义描述中的裸 `&`，原始/本地哈希和替换计数均锁定；
几何、纹理和全部 Fuel 文件字节不变。707 份模型文件只进入外部缓存，
安装包包含清单与轻量证据。复现入口为 `tools/scenes/import_aws_hospital.py`。
场景分类、原 world 的异常高度和草稿状态见 world 包的医院登记。

`aws_hospital_repaired_v1` 与两份 `fuel_hospital_*_repaired_v1` 是独立的派生缓存，
修复五张原肖像图的模型内路径、两条原Normal PNG材质引用，并按独立CGMClassic v4
来源补入同轮子图集的原法线TGA。原始707文件集合继续保留，新选择组合共713文件；
图片像素和碰撞/UV几何不改。`licenses/hospital_repaired/` 保存逐项修改及来源署名，
跨来源归档和各自声明在同一资产版本中完整保留，整体原子发布。资源完整仍不代表可飞行。

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
初始 S2 整理保留 MID360 的传感器参数；2026-10-19 修正了运行版的 IMU 噪声格式和
采样坐标系，实际安装位姿、质量和惯量保持原值。来源不清的外观 mesh 没有导入该集合。
运行版使用 SDFormat 1.6 的六轴 Gaussian 噪声，每样本标准差为角速度
`0.00017 rad/s`、加速度 `0.017 m/s²`；固定初始偏置标准差为 `0.00001 rad/s`、
`0.001 m/s²`，未配置动态偏置扩散。`/livox/imu` 声明 `livox_imu_frame`，
由统一标定发布其相对 `livox_frame` 的真实原点。旧 mesh 模型与历史快照保持原样。
可重建配方为 `tools/scenes/prepare_runtime_assets.py:configure_mid360_imu`；
变更前后 SHA256 记录于 `licenses/runtime/ORIGINS.json`。
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

## S6 Pine Tree cache asset

`osrf_pine_tree_v1` selects the six unchanged `pine_tree` files from the official
Open Robotics `osrf/gazebo_models` snapshot at
`8163eb4b5e7e21985c6591d1c0bfb56468c0093f`. The repository declares CC-BY-3.0;
the original notice, model author Louise Poubel, source, license link and
per-file origins are retained under `licenses/osrf_pine/`. This review concerns
this selected model, not every model in the upstream repository.

The locked official GitHub archive is 227,118,253 bytes; only six files (about
621 KB) are extracted into the external model cache. The full original archive
and license remain cache evidence, outside ROS installation. All forest layouts
using this asset share one cached model version. Prepare it explicitly with
`scene_catalog fetch forest_pine_v1`; `--archive` and `--offline` behave as for
the existing S5 cache. Startup never downloads missing assets.

`tools/scenes/import_pine_forest.py <archive> <forest_gen-source-root> --check`
reproduces the fixed asset manifest, notices, derived world and layout record
from local source bytes. It neither executes upstream code nor overwrites files.
The derived world preserves ETHZ's twenty original poses/scales but uses these
separately licensed pine meshes; original TurboSquid tree assets are not imported.

## S6 Willow Garage office cache asset

`osrf_willowgarage_v1` selects fourteen unchanged `willowgarage` files from the
same official Open Robotics snapshot, `8163eb4b5e7e21985c6591d1c0bfb56468c0093f`.
The repository declares CC-BY-3.0; original author Nate Koenig, copyright notice,
source and license links are retained under `licenses/osrf_willowgarage/`.
Both visual/collision meshes, seven textures and every SDF version referenced by
`model.config` are locked. No upstream ROS package or plugin is imported.

Prepare it explicitly with `scene_catalog fetch office_willowgarage_v1`.
The selected payload is 3,341,870 bytes. The 227,118,253-byte official archive,
original license and extracted model stay in the external cache, outside ROS
installation. `--archive` can reuse the same reviewed archive already used for
the pine model, without another network download. Subsequent office layouts can
reuse this one model asset. Missing/corrupt caches are rejected before startup.

`tools/scenes/import_osrf_office.py <archive> --check` reproduces the asset lock,
attribution, project-authored wrapper world and import record from local bytes.
Original inch units, mesh transforms and model pose `(-20,-20,0)` remain intact.
The collision DAE includes a distant line marker (`ID635`); it is retained and
documented in the office geometry evidence, not silently removed or rescaled.

## S6 urban building cache asset

`osrf_urban_buildings_v1` selects three unchanged OSRF models: `apartment`,
`office_building` and `parking_garage`. All fifteen files (including every
config-listed SDF version, three DAE meshes and four JPEG textures) are locked
to the same official `8163eb4b5e7e21985c6591d1c0bfb56468c0093f` snapshot.
Original CC-BY-3.0 notice and author credits are retained in
`licenses/osrf_urban_buildings/`; the new EasyFly street world uses Apache-2.0.

The selected payload is 3,352,376 bytes and stays outside the ROS installation.
Prepare it with `scene_catalog fetch urban_block_osrf_v1`; the single-source
`--archive` option can reuse the already reviewed 227,118,253-byte official
archive. Missing or changed cached bytes are rejected; launch never downloads.
The street scene now has independent sensor/map, cold-start alignment and two
bounded reference-mission acceptances. Other layouts using this asset require
their own scene qualification; the asset itself does not grant flight readiness.

`python3 tools/scenes/import_osrf_urban.py <archive> --check` reproduces the
manifest, original notice, attribution and file inventory without downloading,
executing upstream code or overwriting outputs. Source inch units and internal
transforms remain unchanged; only new world instance positions are authored.


The curated MID-360 IMU uses the project-local `libeasyfly_gazebo_ros_imu.so` from
`easyfly_gazebo_plugins/safe_imu`. It publishes coherent measurement snapshots
with their own source timestamps and a bounded reliable writer history. Legacy
MID-360 source assets are retained; updated curated hashes and the reproducible
recipe are recorded in `licenses/runtime/ORIGINS.json`.
