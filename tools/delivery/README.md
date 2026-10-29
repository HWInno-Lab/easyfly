# 隔离离线安装包

该工具安装一个带完整 Ubuntu 用户空间的版本目录，并使用 bubblewrap 运行。
宿主机要求 Ubuntu 22.04 amd64、Python 3、bubblewrap、util-linux 的 nsenter 和可用的用户命名空间；
图形界面另需 X11、可读的 XAUTHORITY。运行环境使用 Mesa CPU 渲染和独立网络空间，
容器内使用普通用户 UID／GID 1000，以支持桌面 RViz／QGC 的嵌套隔离；宿主身份不变。
这是本机软件／仿真交付形式；实体远端 DDS、USB 设备、GPU 直通及其他发行版不在此配置范围。

交付应同时提供 `easyfly.tar.gz`、`bundle.json`、本目录工具、源码／依赖物料清单和验收摘要。
清单 SHA 用于完整性检查，不能代替发布者签名或第三方授权。
第三方尚未解决的许可范围见源码 `src/third_party/sourcing-and-patches.md`；
创建本地验收包不表示这些许可问题已经关闭。

## 安装、启动、升级和卸载

以下命令中的包路径使用本次交付文件。安装前缀由当前用户独占，无需 sudo，
不要使用系统目录或已有 ROS 安装目录。

```sh
python3 tools/delivery/manage_bundle.py install \
  --prefix "$HOME/easyfly-offline" --archive easyfly.tar.gz --manifest bundle.json

# 只读资源检查；不启动仿真或提交飞行任务。
python3 tools/delivery/manage_bundle.py run --prefix "$HOME/easyfly-offline" -- \
  ros2 run easyfly_sim_launcher scene_catalog list --json

# 使用宿主机当前 DISPLAY / XAUTHORITY 打开桌面。
python3 tools/delivery/manage_bundle.py run --prefix "$HOME/easyfly-offline" --gui
```

使用更新的归档和清单再次执行 `install` 即可升级。每版必须有不同的 `release_id`；
文件先解压并验证到新的独立目录，成功后才原子切换 `current`。
已有同名版本不会覆盖。运行期间持有共享锁，升级和卸载会拒绝；先正常退出所属应用。
损坏归档、越界路径、硬链接、特殊文件和穿越符号链接的写入会被拒绝。

桌面关闭后，已有后台会话可按产品原有流程继续运行；启动器会等待这些进程正常退出。
在另一终端再次执行同一前缀的 `run --gui` 会加入原来的隔离环境，支持原会话查询与重开，
不会另建一个无法发现原后台的网络空间。PID、启动时刻、版本及根目录均需匹配。
若首次以无图形方式运行，活动环境不能中途增加 X11 绑定；先正常结束该环境再以 `--gui` 启动。

```sh
# 版本 ID 从该版 bundle.json 的 release_id 读取，逐版明确指定。
python3 tools/delivery/manage_bundle.py uninstall \
  --prefix "$HOME/easyfly-offline" --release RELEASE_ID
```

卸载只删除所选版本，保留其他版本和 `state/`。若删除当前版本，需重新安装一个新版本
才能启动；工具不会擅自选择旧版本。要保留回退能力，应先验证升级再卸载旧版本。

## 配置与用户文件

容器内 `/home/easyfly` 对应安装前缀的 `state/home`，`/var/lib/easyfly` 对应 `state/runtime`。
连接配置位于 `state/home/.config/easyfly/connections.json`，桌面偏好位于
`state/home/.config/EasyFly/Desktop.conf`。升级和卸载均不改写这些文件。

从原工作环境迁移时，先关闭 EasyFly，备份原文件，再将明确选定的连接配置复制到上述位置；
目录需为 0700，配置文件需为 0600，目标已有文件时不要覆盖。
连接管理器支持 schema 1 的只读加载，只有显式保存才转为 schema 2；
未知版本或非法字段继续拒绝。迁移不会自动启动连接、开放控制或改写冻结会话。
原配置内的宿主机路径不能在容器里直接使用；在 UI 中重新选择随包 PX4 输入：
`/opt/easyfly/inputs/px4-compat/px4-input`。其他文件放入 `state/home` 后通过对应容器路径选择。

当前配置不依赖宿主 ROS、CUDA、私有 Pangolin／Ceres 前缀或历史 artifacts。
CPU 渲染完整感知请显式选用 `gazebo_lockstep=true`，其验收速度约 0.16 倍实时；
它不授予实时飞行或纯 CPU 赛道资格。具体支持范围见安装包的
`share/easyfly_sim_launcher/ACCEPTANCE.md` 和 `share/easyfly_px4_sitl/GRAPHICS.md`。

## 构建与物料记录

`build_offline.sh` 在准备好的隔离根文件系统内运行，固定使用 `/opt/easyfly/source`、
`/opt/easyfly/inputs`、`/opt/easyfly/dependencies/pangolin` 和 `/opt/easyfly/install`。
先构建 Livox／librealsense，再构建其余包；始终只扫描 `src`，启用测试、媒体和内嵌 RViz。
源码选集、模型、SDK、PX4 兼容输入和原始输入、JSON/Fast DDS 离线源码均需在构建前准备齐全。
构建必须在关闭外部网络的命名空间内执行；脚本本身不是网络隔离工具。

在保留构建目录的独立用户空间做功能回归时，使用普通 UID／GID 1000 和
`/opt/easyfly/bin/environment` 进入测试命令，使嵌套工具隔离及审核后的 PX4 输入路径生效。
RealSense 上游回放测试还要求提前把两份官方录像放在 `/home/easyfly/realsense_records/`：
`outdoors_1color.bag` 和 `D435i_Depth_and_IMU_Stands_still.bag`；
本次输入 URL、长度和 SHA 另随 `realsense-test-inputs/manifest.json` 交付。
录像用于断网回归，不是运行时依赖，也不算实体相机验收。规范检查仍按 V-02 单独报告。

Fast DDS 2.6.11、RViz 11.2.26、MAVROS 2.14.0 及 Gazebo 11.10.2 的精确版本／指纹守卫
保持启用。普通 apt 的最新候选版本不能替代这些已审核输入。
若交付使用已安装包重建的本地 deb，必须标记为 payload snapshot，逐文件验证原 dpkg
校验记录并提供 SHA；不得宣称它是原软件仓库签名的 deb。

```sh
python3 tools/source/generate_inventory.py \
  --source /opt/easyfly/source --packages /opt/easyfly/inputs/packages.tsv \
  --install /opt/easyfly/install --inputs /opt/easyfly/inputs \
  --system-packages --output /opt/easyfly/inventory.json

# 在宿主机对已完成验证的独立根目录打包，不把宿主 / 当成输入。
python3 tools/delivery/pack_bundle.py --rootfs PREPARED_ROOTFS \
  --release-id RELEASE_ID --archive easyfly.tar.gz --manifest bundle.json
```

归档保留运行库、源码、许可文本及版本清单，排除构建目录、个人主目录、运行状态、apt 缓存和日志。
原始构建／仿真／失败证据另行保存。构建通过、安装成功和飞行验收是不同结果，分别记录。
