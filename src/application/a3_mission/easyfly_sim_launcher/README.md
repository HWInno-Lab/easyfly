# EasyFly 仿真启动与场景目录

构建并加载安装环境后，可以查询或离线校验场景：

```bash
ros2 run easyfly_sim_launcher scene_catalog list
ros2 run easyfly_sim_launcher scene_catalog list --json
ros2 run easyfly_sim_launcher scene_catalog validate empty --json
```

命令只读取清单、资源和许可记录，不启动节点、仿真、任务或下载。
`list` 返回 0，如实展示不完整的场景。`validate` 全部检查通过返回 0，
资源/许可/登记不完整返回 1，未知 ID、无效清单或配置错误返回 2。
`runtime_verified` 始终为 false，离线通过不表示可起飞。

Humble 的 `ros2 run` 会在非零退出时向 stdout 追加自己的错误提示。
自动化需要严格 JSON 时使用同一安装模块：

```bash
python3 -m easyfly_sim_launcher.scene_cli validate empty --json
```

可在子命令前指定 `--px4-dir /path/to/existing/PX4-tree`、
`--px4-models /path/to/models` 或 `--gazebo-share /usr/share/gazebo-11`。
这些选项仅用于离线文件检查，不执行该目录中的代码。
默认读取已安装资源和现有兼容性目录；缺失输入会在结果中报告。

## 统一启动

```bash
# 原默认行为保留，不发送飞行 Goal
ros2 launch easyfly_sim_launcher profile.launch.py profile:=base_sim

# 指定完成登记的场景；当前 empty 仍是草稿，会明确拒绝
ros2 launch easyfly_sim_launcher profile.launch.py \
  profile:=base_sim scene:=empty headless:=true
```

`scene` 选择环境，`profile` 选择能力，`headless` 只影响显示。
不指定 scene 时保留原 world、模型路径、出生位姿与能力参数。
指定后必须通过资源、许可、出生点、坐标登记和 profile 兼容检查；
profile 与两个 SITL 入口共用解析器，在会话锁及进程之前检查。
飞机入口、模型根和出生 xyz/yaw 来自清单，不修改原 profile 的规划和安全契约。

同时提供非空 scene/world 会拒绝。公共 profile 和一键包装不接受 raw world；
既有 `px4_sitl.launch.py world:=... model_sdf:=...` 保留为低层调试入口。
低层 world 参数以空字符串表示未指定，实际默认仍为 `empty.world`；
感知入口默认仍为 `perception_demo.world`。独立 D5 包装保留原环境与飞机 fixture。

原三张地图均为 draft，没有新增运行兼容性记录。感知场景缺 `suv`；
PX4 与系统 Gazebo 根目录存在 sun/ground_plane 同名冲突，资产许可与坐标对齐待补。
查询和受管启动分别报告状态，不为启用命令而修改验证标记。

## 资源解析

共用核心为 `easyfly_px4_sitl.scene_catalog`；launcher 原导入路径保留兼容入口，
避免 SITL 反向依赖 launcher。安装环境从 ament 索引解析包。
symlink-install 下，通过安装 package.xml 的源码链接和包名核对，仅信任该包源码根，
不信任任意资源链接或整个工作区。离线工具也可显式传入受信根。
启动仍须通过原有 PX4 HRT 兼容性检查；离线文件选项不提供启动旁路。

## 仓库检查（S2）

新增 `warehouse_aws_v1` 与上游独立无屋顶版本 `warehouse_aws_no_roof_v1`。
两者资源和许可检查通过，均已登记支持 `navigation_dev`。S2 v10 分别完成两次独立冷启动、
同一仿真内两轮中央通道 1.5m 往返与降落，每项24个进程正常退出，证据分别保存在场景目录。
仓库用 SLAM map 系保存可信地面，导航/点云输出仍为 odom；旧默认入口保留原 odom 行为。
仿真 LiDAR 使用原生 Gazebo 扫描时间，消费者仍拒绝时间戳重复/回退。
已复现的原生缺陷及回归记录见场景证据；历史未保留栈的退出异常不作唯一归因。

完成构建并加载含本轮世界、语义地图、plan_env 和启动包的安装环境后：

```bash
ros2 run easyfly_sim_launcher scene_catalog validate warehouse_aws_v1 --json
ros2 launch easyfly_sim_launcher profile.launch.py \
  profile:=navigation_dev scene:=warehouse_aws_v1 headless:=true
```

启动只加载环境和能力；飞行任务仍需显式提交。支持范围限于清单内参考任务与证据，
不能将该有界检查视为任意仓库路径已验证。

显式未解锁诊断命令如下（先构建并加载安装环境及已验证的 HRT 目录）：

```bash
mkdir -p artifacts/my-warehouse-bootstrap  # 必须是新的空目录
ros2 run easyfly_sim_launcher scene_sitl --scene warehouse_aws_v1 --bootstrap \
  --px4-dir /path/to/reviewed/PX4-source \
  --output-dir "$PWD/artifacts/my-warehouse-bootstrap" --domain 128 --gazebo-port 11428
```

bootstrap 禁止所有飞行 Goal；已注册场景使用其受管入口，草稿检查才使用低层 raw world
与显式 `debug_model_path`。该参数不由公共 profile 转发，也不能与 scene 混用。检查分别记录资源/传感器观测、
连续地图、坐标及退出状态。`passed` 仅表示所选检查流程通过；必须另看
`mapping_ready`，不能将未解锁检查通过当成导航通过。
真实深度是 `/camera/aligned_depth_to_color/depth/image_raw`（32FC1），
`/camera/aligned_depth_to_color/image_raw` 是可视化图像，不能代替米制深度。

检查工具同时核对实际加载的 LiDAR、语义地图和 plan_env 库路径与 SHA256。
坐标采样前要求未解锁、状态健康、估计速度连续三秒 <=0.05m/s；
飞行检查保留相对场景参考的 0.2m / 0.1rad 偏差门槛。
修改后请用 `colcon build` 完成安装和环境钩子生成；单独 `cmake --install` 可能覆盖
colcon 的 package.dsv 钩子，造成选中了新包前缀却加载旧库。
完整原始证据与当前缺口见工作区 `schedule-v5-scene.md` 的 S2 记录。

`scene_sitl --cycles 2` explicitly runs the registered bounded reference mission
twice in the same owned simulator. Every cycle requires two new real navigation
UUIDs, successful terminal results within 0.15 m, closed sessions and released
ownership. After landing, it waits for healthy disarmed state, speed ≤0.05 m/s,
position within 0.15 m of the initial stationary position in the declared ground
reference frame (map for the registered indoor scenes), and fresh occupancy and
non-ground observations continuously for three wall seconds. This also applies
to the default single cycle. The postlanding observation budget is 120 wall
seconds; a recorded 60-second observation ended while the estimator was only
approaching the continuous stationary gate. The initial preflight budget remains
60 seconds. A map reference requires a finite, fresh map←odom transform; drift in
odom must not be mistaken for physical motion. Raw odom state and the measured
basis are retained alongside the transformed observation. No physical admission
threshold is relaxed. Before cycle two, the
scene's independent world/odom reference is checked again. A timeout prevents
the next goal and fails acceptance. `--bootstrap` continues to forbid all goals.

## S3 indoor scenes

`house_aws_v1` (`indoor/residential`) and `bookstore_aws_v1`
(`indoor/commercial`) use the same managed `navigation_dev` entry point.
After building and sourcing the matching installation, inspect a scene with:

```bash
ros2 launch easyfly_sim_launcher profile.launch.py \
  profile:=navigation_dev scene:=house_aws_v1 headless:=false
```

Replace the scene ID with `bookstore_aws_v1` for the bookstore. Opening a scene
does not submit a flight mission. The reviewed missions are
`missions/scenes/house_aws_v1/living_room_out_and_back.yaml` and
`missions/scenes/bookstore_aws_v1/cross_aisle_out_and_back.yaml` in
`easyfly_mission_scripts`. Each is a bounded 1.5m out-and-back in odom, with
takeoff, hover and landing through the existing executor. The house route is in
the living room; the bookstore route is in its north wide cross-aisle.
Narrow shelves, door crossings and the full interiors need separate tasks.

Both manifests explicitly select `observed_cells` occupancy footprints. The
probe verifies the actual semantic-node parameter and loaded libraries before
sending any goal. Independent cold-start and sequential mission evidence is
stored per scene in `easyfly_gazebo_worlds/evidence`; historical rejected runs
are retained alongside later acceptance records.

The v7 matrix passed on 2026-10-17: each scene has two independent cold starts
and two consecutive reference missions, with all 24 processes exiting normally
in every run. These results cover the named routes and recorded runtime only.
The scene README retains the separate unresolved historical Gazebo rendering
startup fault; it is not attributed to the fixed DDS reader discovery race.

## S4 primitive scene generation

`scene_generator` is an explicit offline tool. It reads a strict JSON parameter
document and writes a new bundle containing a world, draft scene, canonical
parameters, generation record and project license. It does not launch simulation
or submit flight goals. Bounds, obstacle count/size/height/spacing, route clearance
and a fixed seed define the layout. V1 uses static boxes/cylinders on a flat z=0
floor, yaw=0 and spawn z=0.1; the reserved route runs along world +X.

```bash
ros2 run easyfly_sim_launcher scene_generator \
  /path/to/primitives_sparse_seed42_v1.json --output-dir /path/to/new-bundle
ros2 run easyfly_sim_launcher scene_generator \
  /path/to/primitives_sparse_seed42_v1.json --output-dir /path/to/new-bundle --check-layout
```

The output directory must not exist. Copy reviewed bundle resources into the
world package without overwriting an existing scene revision, then build and use
`scene_catalog validate`. Parameters live in `easyfly_gazebo_worlds/generators/`;
their schema is `schemas/generator-v1.schema.json`. A specified SplitMix64 stream
and integer-millimetre placements make layouts reproducible without a random
package dependency. The record locks parameter, generator-source, layout and
world hashes. Collision and visual primitives have identical geometry.

A reserved route capsule includes both endpoints and the vertical takeoff area.
A bounded placement search must satisfy the exact obstacle count, separation and
clearance; failure writes no partial bundle. `primitives_blocked_seed42_v1.json`
is a deliberate rejected input, not a registered or flyable scene. Generation
never supplies measured world/odom alignment or marks sensors/navigation tested.
Qualification adds separate native geometry, cold-start and mission evidence.
`--check-layout` checks immutable fields and generated resources while preserving
measured qualification metadata; it always reports `runtime_verified=false`.

The first complete S4 sample is `primitives_sparse_seed42_v1`: two managed
cold starts and two sequential reference missions passed, including normal
process exit. The corridor v1 remains draft after insufficient floor support;
corridor v2 remains draft after the second-mission heading check rejected it.
Their evidence is retained. A generated or geometrically valid layout does not
inherit the sparse sample's navigation qualification. S4 as a whole is pending.
