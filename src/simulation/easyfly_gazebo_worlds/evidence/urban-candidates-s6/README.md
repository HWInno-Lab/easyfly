# S6 城市、园区候选审计（2026-10-17）

本轮完成来源筛选及首选资源的离线闭包检查。首选为 OSRF 的三种静态建筑，
用于下一轮创建 EasyFly 自有的小型城市街区。它不是现成校园地图，
也不把完整 CitySim 城市的资源、兼容性或飞行能力算作通过。
生产目录仍为 16 项、10 registered、6 draft；本轮没有新增可启动场景。

## 选择结果

| 候选 | 固定修订 | 本轮结论 |
| --- | --- | --- |
| [OSRF 静态建筑](https://github.com/osrf/gazebo_models/tree/8163eb4b5e7e21985c6591d1c0bfb56468c0093f) | `8163eb4b5e7e21985c6591d1c0bfb56468c0093f` | 首选：apartment、office_building、parking_garage，15 文件、3,352,376 字节，资源闭包通过 |
| [OSRF CitySim](https://github.com/osrf/citysim/tree/3928b08e2598f5ead2e9b24640fe1bde262a136d) | `3928b08e2598f5ead2e9b24640fe1bde262a136d` | 暂缓：原图含人物、车辆控制和渲染插件，完整静态派生与逐资产审计尚未完成 |
| [CMU campus](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/bf0cba71365271ebff09831a05afd78578150300) | `bf0cba71365271ebff09831a05afd78578150300` | 暂缓：校外下载包未锁内容与模型授权，原 world 另有零重力、零磁场和历史时钟 |
| [Clearpath inspection](https://github.com/clearpathrobotics/cpr_gazebo/tree/509afbeec942c6aad376a5a76a24330f1c56dc17/cpr_inspection_gazebo) | `509afbeec942c6aad376a5a76a24330f1c56dc17` | 暂缓：实际地形由 ROS1 Xacro 另行生成；完整网格、纹理与配件闭包仍待检查 |

首选建筑来自此前已固定的官方 `gazebo_models` 归档，不重新跟随默认分支。
上游 [LICENSE](https://github.com/osrf/gazebo_models/blob/8163eb4b5e7e21985c6591d1c0bfb56468c0093f/LICENSE)
声明 CC-BY-3.0；逐项读取所选 model.config、各版本 SDF、DAE 和纹理清单，
未发现所选文件的独立许可例外。导入时保留该声明以及原模型与网格作者署名，
新布局/world 单独使用项目 Apache-2.0，不能将第三方建筑重标为项目许可。

完整 CitySim 有 546 个直接 include、55 个模型名称，其中 53 个来自独立 OSRF 归档；
按 SDF/config 继续展开得到 57 个模型名称、362 文件、87,493,422 字节。
名称均可找到不等于完整纹理、材质和许可闭包通过。其 22 个 actor 和 6 种插件
需要另行处理；海面含 GLSL 材质及 texturelib.com 独立纹理条款，不能仅凭根 Apache-2.0
统一发布。后续若制作静态派生图，必须显式记录排除项和新的 world 修订。

CMU 的 `vehicle_simulator/package.xml` 只有 BSD 标签，仓库内没有 campus 模型许可；
velodyne 子目录的 LICENSE 不能推导校园网格授权。只读取下载脚本，没有执行或下载外部包。
Clearpath 的包内 `docs/LICENSE.txt` 存在 BSD 三条款式声明；完整仓库归档在 200MiB
审计预算处停止，改用固定 Git tree 和选定文本核对，完整载荷大小和哈希保持未知。
其 Noetic world 中水流插件已注释；真正待处理的是 Xacro、纹理来源和地形碰撞。

## 可复核证据

- [candidate-audit.json](candidate-audit.json)：四个候选的版本、归档/文本指纹、许可范围、阻碍与接入顺序。
- [building-resources.json](building-resources.json)：三建筑全部 15 文件 SHA256、作者、单位、五份 SDF 版本检查、资源闭包与负例结果。
- 本地原始快照和发现过程：`artifacts/scene-s6-urban-audit-20261017/`。
- 原官方建筑归档：`artifacts/scene-s6-forest-20261017/upstream/gazebo_models-8163eb4b.tar.gz`。

重复离线审计：

```bash
python3 tools/scenes/audit_osrf_urban.py \
  artifacts/scene-s6-forest-20261017/upstream/gazebo_models-8163eb4b.tar.gz \
  --output artifacts/scene-s6-urban-audit-20261017/osrf-building-resources.json
```

维护工具先核对完整归档 SHA256，仅在临时目录取出所选模型，复用真实 SceneCatalog
检查全部 SDF 版本、DAE 图像引用和纹理字节，并调用本机 Gazebo 11.10.2 的
`gz sdf -k` 检查五份 SDF。缺失纹理与篡改纹理均被拒绝；错误归档哈希另有拒绝核对。
临时 fixture 的 runtime 只是同一个资源 world，未检查飞机运行依赖；
默认重叠放置不构成街区布局。生产目录未被修改，fixture 持续保持 draft、无出生点、无对齐。

## 下一轮接入边界

拟建 `urban_block_osrf_v1`，分类 `outdoor/urban`，标签考虑 building、road、inspection、static。
先登记独立可选缓存和署名，再创建项目自有道路/地面及建筑布局，从 draft 开始。
三份 DAE 均声明 inch / meter=0.0254，必须保留原始变换，通过原生 Gazebo 测量
实际尺寸、地板和碰撞后再冻结布局、出生点及参考路线；不能按顶点值猜测净空。

尚未执行原生 world 加载、ODE 射线、传感器/建图、两种安装冷启动或连续双轮飞行。
`registration_complete`、`native_world_loaded`、`navigation_tested` 均为 false。
本轮只完成候选审计，不新增导航验收，也不将 S6 写成整体完成。
