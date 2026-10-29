# 仿真图形部署与资源边界

赛道 `racetrack_aws_day_v2` 的完整参考任务目前支持硬件 GLX 渲染。
同场景的软件渲染未通过连续就绪门限，不能把“world 加载完成”当作任务就绪。
书店、住宅的软件渲染通过记录不外推到赛道。相机、雷达、物理步长和状态新鲜度要求保持不变。

## 部署检查

1. 使用 Ubuntu 22.04、ROS Humble、Gazebo Classic 11 的项目构建环境。
   安装包依赖中的 `mesa-utils` 提供 `glxinfo`；它只查询实际 OpenGL 上下文。
2. GPU 驱动的内核模块与用户态 OpenGL/GLX 库必须匹配。NVIDIA 主机同时检查
   `/sys/module/nvidia/version`、`nvidia-smi` 与 `glxinfo -B`，不要仅凭设备存在或
   `nvidia-smi` 成功推断 Gazebo 正在使用 GPU。
3. 在启动 Gazebo 的同一终端、同一 `DISPLAY` 和动态库环境中执行：

   ```bash
   ros2 run easyfly_px4_sitl check_graphics --require-hardware-rendering
   ```

   退出 0 表示图形预检通过，JSON 中 `flight_ready` 始终为 false。
   缺失显示服务、间接 GLX、软件渲染或已识别的 NVIDIA 版本冲突退出 2。
   工具不修改驱动、环境变量或运行中的仿真，不执行解锁和飞行。
   软件渲染适用场景可以省略 `--require-hardware-rendering`，报告仍明确 renderer。
4. 再用 `scene_sitl` 对目标场景执行完整参考任务，检查原始连续就绪、传感器新鲜度、
   任务终态和正常退出。图形预检不能代替此验收。

## 当前已测资源配置

2026-10-24 的赛道任务在 i5-13600KF（14 核／20 线程）、32 GiB 内存、
RTX 5060 Ti、直接 GLX 下通过；单个 PX4/Gazebo 会话运行，未降低原传感器规格。
这是已测部署配置，**不是经过资源递减实验得到的最低硬件规格**。
更低配置、其他 GPU、容器转发及远程显示必须重新执行完整参考任务。
可用 CPU/内存不能单独证明实时性；应以原任务的新鲜度和连续就绪门限验收。

Mesa 默认使用 `LP_NUM_THREADS=1`，显式配置会保留；不要为使赛道通过而放宽门限、
隐藏传感器或减少地图内容。赛道纯软件支持与最低硬件资源测定仍未承诺。

## 受限 CPU 的输入采集配置

纯 CPU 渲染需要显式选择原生传感器／渲染锁步；在已经配置 PX4、显示服务和 ROS 环境的终端使用：

```bash
LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa LP_NUM_THREADS=1 \
  ros2 launch easyfly_px4_sitl perception_sitl.launch.py \
  headless:=true detection_use_gpu:=false gazebo_lockstep:=true
```

2026-10-28 在四个 CPU、两路各 75% 竞争负载下，两次独立冷启动完整地面栈
各采集至少 120 墙钟秒。原始图像与 IMU 均保留源时间；没有裁剪启动数据、
降低传感器规格或重打时间戳。585／582 帧通过原生与 ROS 像素一致性核验，
最大图像源时间间隔均为 34 ms（要求不超过 250 ms），IMU 不超过 25 ms；
无重复时间对应不同图像，进程普通退出且没有遗留。

该负载下仿真速度约为实时的 **0.16 倍**。这验证的是完整地面栈的输入时间质量，
不承诺实时吞吐、纯软件赛道飞行或其他 CPU 配置。未启用锁步的历史四核纯 CPU
运行曾出现 268／257 ms 缺口，仍不属于本项支持配置；默认 `auto` 不自动改为锁步。
原始记录与签收摘要为 `artifacts/completion-steps1-4-20261028/v16-cpu-signoff.json`。

## 本机故障与复现证据

本机曾加载 NVIDIA 内核模块 `595.91.07`，用户态库却为 `595.99.02`。
2026-10-24 使用匹配的官方 `595.91.07` 包**只解包到项目证据目录**，
在子进程环境中选择对应 vendor 库后，GLX 和完整赛道任务恢复。
此本机兼容路径不是安装包的默认依赖，也不应硬编码进部署启动脚本。
正式部署应由系统包管理维护一致的驱动栈。

本轮追加检查时，主机内核和系统用户态已均为 `595.99.02`（本任务未修改驱动）。
直接使用系统 GLX、没有加入旧 vendor 库路径，赛道参考任务在 218.68 秒内完成，
最终上锁且所属进程正常退出；见 `b12-system-preflight.json` 与
`sitl/b12-racetrack-system-gpu/result.json`。因此不再把旧私有目录写成运行必需条件。
私有 Xvfb/llvmpipe 的硬件要求负例明确退出 2，仍保留纯软件赛道不受支持的边界。

历史来源、归档校验值及任务日志在工作区
`artifacts/test-full-debug-v1-round2-20261024/graphics-compat/provenance.json`
与 `sitl/racetrack-driver-compatible/`；本轮追加结果位于
`artifacts/b05-b14-20261024/`。这些路径用于审计，安装包运行不依赖它们。
