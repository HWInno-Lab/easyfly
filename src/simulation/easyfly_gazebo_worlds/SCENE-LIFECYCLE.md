# 场景保留与替代决策

2026-10-28，D-08：本版本保留 20 个场景清单，其中 14 个正式登记、6 个草稿。
以下六个草稿的处理决定已明确；它们不再笼统作为“还要修复的六个世界”。
`draft` 仍表示不通过通用受管启动准入，不表示原始 world 文件需要删除。

| 清单 ID | 本版本决定 | 使用入口与保留理由 |
| --- | --- | --- |
| `empty` | 保留历史最小世界，不晋升通用飞行场景 | 原出生点 0.83 m 尚未验证且缺少导航资源／参考任务；基础仿真选择已有 `base_sim` 登记场景，例如 `primitives_sparse_seed42_v1`。不得继承旧出生点作为新导航初始化条件。 |
| `perception_demo` | 保留兼容与独立感知回归夹具 | 旧感知／导航测试依赖该 world；保留明确的旧入口。通用桌面场景选择使用登记场景，目标检测／跟踪专项按对应包说明执行，不把专项通过合并为通用场景登记。 |
| `precision_landing` | 保留精降专项世界，不晋升当前通用飞机清单 | 使用已安装的 `easyfly_sim_launcher d5_simulation`，由该入口生成／核验专用飞机、下视相机、标志板及出生点。通用场景清单对应的基础飞机不等于 D5 夹具；硬件与移动板能力仍未验收。 |
| `hospital_aws_v1` | 历史原始输入，由 `hospital_aws_v2` 替代 | 保留异常物体高度等原始证据，日常受管启动选择已登记 v2；不原地改写 v1 的 world 或 SHA。 |
| `primitives_corridor_seed1042_v1` | 历史原始输入，由 `primitives_corridor_seed1042_v2` 替代 | 保留原生成参数／几何证据，使用已登记 v2 的出生点、坐标和任务。 |
| `racetrack_aws_day_v1` | 历史原始输入，由 `racetrack_aws_day_v2` 替代 | 保留原资产与坐标审计，使用已登记 v2；赛道完整任务仍要求已验证的硬件图形配置。 |

登记查询：

```sh
ros2 run easyfly_sim_launcher scene_catalog list --json
ros2 run easyfly_sim_launcher scene_catalog validate hospital_aws_v2 --json
ros2 run easyfly_sim_launcher d5_simulation --help
```

命令中的资源校验仍需按本包 README 提供 PX4／Gazebo 根目录。
静态登记与历史验收不替代每次启动的资源、会话、状态、传感器和任务准入。

今后如需将前三个草稿开放为通用场景，应创建独立修订并补齐资源闭包、明确出生点、
世界到导航坐标、支持 profile、参考任务及完整飞行／恢复／退出验证。
当前决定不扩展它们的准入范围，也不删除旧证据。
