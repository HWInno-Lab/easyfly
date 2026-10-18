# ALG-8.2 EGO 第三方分类迁移验收

日期：2026-10-18；迁移前提交 `c3e41af`，原算法盘点基线 `bdbc34c`。
本批将整个 EGO 工程分类到 `src/third_party/algorithms/planning/ego_planner`。
内部8包、包名、话题、消息和平台会话协议保留，EasyFly 仍通过 easyfly_local_planner 接入。
SDK 分类留待 ALG-8.3；没有新建第二套 EGO 搜索、建图或优化实现。

## 1. 文件与依赖边界

143份原受追踪 vendor 文件整体搬迁，136份字节相同；7份变化为4份 CMake 和3份 Python 测试/诊断脚本。
全部算法 C++/头文件、消息、配置、launch、包元数据及已有通知保持字节身份。
适配包只有 CMake 注释路径变化，正式 local_planner.launch.py 和 package.xml 字节不变。
构建优化选项和所有安全/速度/碰撞/失效/退出断言没有放宽。

依赖调整使用包已声明的 plan_env/path_searching 和导出目标，删除工作区根推测、
src/planner 拼接、相邻包源码 include 及 plan_env_DIR 向上推算。
traj_utils_so 增加其库的 namespaced target 导出；bspline_opt 和主节点通过现有
ament_target_dependencies 消费导出目标，删除再次展开的原始库列表。
这同时避免导出接口保留旧安装绝对路径及 ROS 库重复展开；系统 PCL 等依赖继续保留。

信号测试默认从安装索引选择 ego_planner_node，并从安装 share 读取正式适配参数；
仍支持显式指定可执行文件以验证旧安装。测试场景、断言和超时原样保留。
两个调试 probe 改为向上定位工作区及读取新源码路径，本批未执行调试压力场景。

来源和逐文件前后 SHA 见 [来源清单](../third_party/algorithms/planning/ego_planner.provenance.json)，
接入说明见 [外部规划算法](../third_party/algorithms/planning/README.md)。
原上游仓库/fork/branch/commit未知，本地导入与补丁提交不冒充上游版本。
多数包的 TODO 许可及 MIT/BSD 元数据原样保留；没有统一根 LICENSE，未推断整个工程许可。

## 2. 本机旧安装兼容

原主 build/install 和 scene-s2 安装没有被覆盖。旧 planners/ego_planner 路径保留本地忽略的
软链接，指向唯一的新源码；旧父目录的 COLCON_IGNORE 排除重复发现。
软链接与 marker 均不提交。重新检出只有新目录，应重新配置构建；确认旧消费者结束并切换后再清理桥接。
隔离构建不代表用户现有主安装已经切换。

验收前后核对旧 EGO、traj_server 和 plan_env 三份二进制身份及原运行进程。
colcon 前后包集合仍为91，仅8个 vendor 包的源码路径变化，父分类目录不生成包。

## 3. 验证与证据

证据根：`artifacts/algorithms/alg8-2-20261018/`；最终矩阵 `matrix-v2/`，实际计数见
[alg8-2-results.json](baseline/alg8-2-results.json)。输入归档146份文件：143份 vendor 加3份适配文件。

旧安装先运行独立 B-spline/消息消费者，保存200组确定性样条计算轨迹；
随后旧安装的导航会话、受管故障、3类真实优化/精化场景、traj_server 退出共6项通过。
旧 EGO 信号退出另计1个测试，包含3阶段×2时钟×2信号的12个子场景，正常退出。

每种新安装均构建8个 vendor 包和 easyfly_local_planner，并运行以下原有测试：

| 包 | CTest 注册项 | 范围 |
| --- | --- | --- |
| plan_env | 6 | 膨胀安全/并集、可信云保持、odom 时效、虚拟顶棚、深度投影内存 |
| bspline_opt | 1 | 碰撞采样 |
| ego_planner | 10 | 6项 Python 运行/采样测试，4组 GTest 安全测试 |

保留 inflation_union 的 ASAN/UBSAN 及深度投影真实实现的 ASAN 配置。
真实 ROS 夹具验证导航会话归属/故障与正常、占据起点、超速边界三类优化场景；
EGO 与 traj_server 的退出测试要求进程正常退出，强杀清理不计成功。
这里统计注册项，不把 unittest subTest 或内部多个场景冒充独立 CTest 项。

两种安装合计34次 CTest 注册项执行全部通过；每种实际包含92个 GTest 用例和8个 Python unittest 方法，合计200次用例执行。
另有2次独立安装消费者直接运行，400组样条差分全部相同；两种消息各完成两次往返。
154份矩阵输入指纹稳定，AST检查28份、包XML检查9份及 git diff --check 通过。
ALG-1..5 的13/36/69/29/146份保护输入、ALG-7.3 的45份全部不变；ALG-6/7.1/7.2沿用既有差异。
ALG-8.1 的61份输入中59份不变，仅共享 .gitignore 和 third_party 根索引更新。
ALG-0冻结465份文件按两次vendor搬迁映射后，369份未改、80份修改、16份已迁移/删除；冻结记录本身不改写。

复制安装根实际搬迁后，再配置、编译消费者并运行节点测试。
消费者仅使用安装头文件，直接调用 B-spline 并解析实际 AStar 库符号，
traj_utils/Bspline 与 quadrotor_msgs/PositionCommand 完成 CDR 往返，
200组位置/导数/时间拉伸结果与旧库的17位精度输出逐字节比较。
检查相关动态库及运行用 Python 消息/节点索引均选择本次前缀，不靠旧安装替代。
这里只验证 AStar 链接与既有规划场景，不另称已完成新的三维搜索精度或性能资格验收。

首次隔离构建在主节点链接耗时超过8分钟后主动中断，保留 initial-build.log；
确认原始库列表与 modern target 重复展开后删除冗余链接，未修改算法或编译优化。
第一轮矩阵的8个 vendor 包构建通过，适配包因缺 ALG-4/5 已迁移核心的安装前缀而停止；
最终矩阵补显式 pointcloud_filters/tag_pose 支持前缀，未绕过生产依赖。
完整包 lint 不计为通过，也没有新的完整工作区 clean rebuild 或 SITL 飞行验收。

## 4. 复验与保留问题

```bash
python3 tools/algorithms/verify_ego_install.py \
  --symlink-prefix artifacts/algorithms/alg8-2-20261018/initial/install \
  --baseline artifacts/algorithms/alg8-2-20261018 \
  --output artifacts/algorithms/alg8-2-new-run
```

工具拒绝覆盖既有证据；需要归档输入、旧库对照输出及本地忽略桥接。
构建使用已有 ROS/主安装/scene-s3 支持 underlay，显式选择新安装的9个包。
复制安装重定位验证范围是 CMake 导出、原生库、消息、节点/资源加载；
不承诺任意生成的 colcon shell setup 脚本可搬迁，也不将完整 ROS 工程冒称无 ROS 算法库。

夹具使用非零 DDS 域，不启动新的 PX4/Gazebo、不向用户仿真发送飞行任务、不停止现有仿真。
ALG0-B03 冻结时钟停流夹具初始就绪失败及历史 semantic shutdown/stress 问题仍未解决；
本批通过的原有 EGO/轨迹退出测试不等于这些历史问题已修复。
