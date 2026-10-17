# EasyFly 仿真与连接管理

Ubuntu 22.04 / ROS 2 Humble / Qt5 的图形入口。安装后的日常使用可在界面完成：
保存连接配置、选择仿真场景、启动 Gazebo/PX4/EasyFly、连接 DDS、配置飞控
UDP/TCP/串口、开启 `gcs_url` 转发、打开联网 QGC，以及正常关闭和保留后台。
不需要编辑 launch/YAML、手动 source 环境或在终端启动仿真。

本批优先完成仿真使用链。串口配置和错误路径已提供，真实飞控/串口、远端 QGC、
真实双机 DDS 没有由本机仿真验收替代。高级 DDS 接口/peer/discovery server 预设仍后置。
联网 QGC 是双向维护模式，具有控制能力；不能称为只读或全局独占。

## 从界面开始

打开安装的 `EasyFly` 应用菜单项或 `easyfly-connections.desktop`。
连接配置位于主窗口左侧的“连接与仿真”，无需另外打开管理窗口。
左侧导航固定为：连接与仿真、运行总览、任务中心、相机与感知、空间视图、记录与回放、诊断与设置。
尚未连接时不创建 ROS；可编辑本地任务草稿、查看本地记录和诊断设置，实时观测页需连接后使用。
配置分为五页，所有程序/目录均有选择入口，串口设备可刷新和手动填写：

| 页 | 配置 |
| --- | --- |
| 运行环境 | 运行配置、已安装场景和静态基本几何预览；高级设置内选择兼容 PX4 输入目录、Gazebo master 端口 |
| 飞控链路 | UDP/TCP/串口、网络地址和端口、串口设备、波特率、目标 MAVLink system ID |
| DDS 连接 | ROS domain 0–232、本机/局域网、模拟/系统时间、前视/下视相机 topic |
| QGC 转发 | 是否启用、QGC 所在主机和接收端口、MAVROS 转发本地端口、QGC 程序 |
| 操作权限 | 显式启用仿真任务、文件导入、平台元数据编辑；录像与内嵌三维按安装能力提供固定入口 |

三种工作方式使用顶部卡片切换；配置只显示当前方式适用的字段。连接名称过长时省略显示，可通过提示查看完整值；场景预览随左栏等比缩放。
后台报告异常时主动显示诊断原因，不用绿色运行状态掩盖故障。

场景预览从已校验的
已安装 world 提取有限静态基本几何，不是实时仿真截图；模型、网格和 include 会省略。
后台、DDS、飞控和 QGC 分别显示状态，通信收包不会代替飞机状态或 QGC 显示确认。

三种工作方式：

- **启动本机仿真**：选择已登记且支持当前运行配置的场景，点击“启动仿真并连接 DDS”。
  未保存的新配置会先保存。程序校验当前资源、许可、PX4 兼容输入和端口，创建独立
  PX4 rootfs，再启动已有仿真入口与工作窗口。不会自动解锁或提交飞行任务。
  当前使用后台 Gazebo；相机和三维从工作窗口查看，独立 RViz 可手动打开。
  原生 Gazebo 窗口的退出残留尚未解决，界面及后端均拒绝启动该未验路径。
- **连接已运行的平台**：选择匹配的 domain、通信范围和时间源，点击“连接 DDS”。
  只打开工作窗口，不启动第二套 MAVROS/Gazebo；PX4 原生 `/fmu/*` 仍需 EasyFly 状态适配。
- **启动本机通信**：填 UDP/TCP 地址，或选择串口设备和波特率，再启动 MAVROS 与状态桥。
  默认使用系统时间。串口不存在、权限不足、端口占用和已有状态源会显示错误，不修改设备权限。

本机单机 SITL 使用 localhost DDS、模拟时间和 `127.0.0.1:14540 ↔ 127.0.0.1:14580`
的现有 PX4 offboard UDP 链路。这些飞控链路字段在仿真模式自动固定。场景/运行配置
不匹配、draft、缺缓存/插件或不合格 PX4 会在仿真子进程创建前拒绝。
安装时可记录已准备的 PX4 兼容输入；迁移安装后可在界面重新选择验证过的目录。
资源准备和依赖安装属于部署工作，缺失时不会暗中下载或使用未校验的替代资源。

## 页面分工

- **运行总览**：飞机状态、姿态、相机和局部地图；三维快捷入口跳转到空间视图。
- **任务中心**：任务目录、步骤与参数编辑、高级 YAML、文件导入、执行与查询。
- **相机与感知**：大画面原图/同帧叠加；目标和障碍明细通过标签页切换。
- **空间视图**：二维地图和内嵌 RViz；仅一个渲染器，进入三维页后手动打开。
- **记录与回放**：手动录像、校验后的本地记录索引和独立回放；列表包含缩略图、开始时间、
  已收帧时间跨度、相机来源和已有执行身份，缺失值显示未知，不补造关联。
- **诊断与设置**：有效连接配置只读、诊断与高级工具；修改连接统一回到“连接与仿真”。

连接前如有未保存的本地任务草稿，需要先保存/导出或明确处理，连接交接不会静默丢弃草稿。

## 连接 QGC

1. 在启动仿真/通信前进入“QGC 转发”，启用转发，填写 QGC 主机和接收端口。
2. 本机选择可信的 QGC AppImage/程序。仿真默认 QGC 接收 `14560`、MAVROS 转发本地端口
   `14558`，避开 PX4 自带的 localhost `14550` 直连；本机仿真拒绝把转发也接到 `14550`。
3. 启动仿真/通信，等待真实的未解锁、在地面和新鲜连接状态，再点击“打开并连接 QGC”。
   QGC 的首次设置和关闭确认直接在其窗口完成。

程序为本次 QGC 自动写入独立的 UDP 设置、配置/缓存/保存目录，关闭自动串口和其他
发现入口，不改原 QGC 的设置。支持本机 X11/XWayland 的窗口管理，当前实际联验样本
为 QGC 5.0.8；其他版本/定制组织名需独立验证。首次运行的单位设置属于 QGC 自身界面。

界面区分“进程运行”“转发端点发送/接收字节”“实际工作窗口的新鲜飞机状态”。
端点收包不冒充 QGC 仪表显示签收。远端 QGC 在目标机器打开并匹配端口；本机不冒充
启动了远端程序。修改活动转发参数需要正常停止后端、保存配置、再启动。

QGC 联网维护模式不能同时启用 EasyFly 任务执行、导入或参数修改。“诊断与设置 → 高级工具”中的
离线隔离 QGC 预览与联网维护分别操作，共用连接配置里选择的程序路径。选择程序不启用转发。关闭 QGC 不等于停止飞控。

## 停止、重开与配置

- 工作中的“结束观测”走原有草稿、录像、工具和未决确认，完成后返回
  EasyFly 的连接页，保留仿真/通信后台。取消关闭确认会留在当前工作窗口。
- 窗口关闭按钮和退出菜单结束当前 EasyFly 界面，后台继续保留。重新打开 EasyFly
  查询同一实例；点击“连接 DDS /
  重开工作窗口”重建相同配置的整个工作进程，不重复 launch 后端。
- 停止仿真/通信前，先关闭本次 QGC，再正常关闭工作窗口。后端重新检查真实的连接、
  未解锁、在地面、新鲜度、单发布源和已观测活动 action。状态未知/过期、任务未决、
  已解锁或离地会拒绝停止。启动尚未连接时可取消启动。
- 管理服务仅向本次 launch 请求正常 SIGINT，不按进程名称查杀、不自行升级信号。
  launch 内部沿用 ROS 2 的子进程超时升级策略，异常退出不能标为正常停止。
  停止超时且 launch 仍存活时显示未确认并保留所有权。QGC 通过所属会话和专属配置目录定位自己的主窗口，发送正常
  关闭请求；其确认框需要在 QGC 完成，不强杀 AppImage 包装或其他 QGC。
- 结束所有自有对象后，可从“管理服务”菜单结束空闲服务。不同管理窗口不能重复创建
  本机后端；外部仿真、MAVROS 和已有端口不会被接管。

配置文件为 `$XDG_CONFIG_HOME/easyfly/connections.json`（通常 `~/.config/easyfly`），
父目录 0700、文件 0600，最多 32 条。schema1 观测配置只读迁移为内存中的默认关闭功能，
显式保存后写 schema2。损坏、未知字段或权限错误保留原文件并报告，不静默修复。
导入/导出和保存不启动连接。

运行日志、QGC 专属设置和独立 rootfs 位于
`$XDG_DATA_HOME/easyfly-connections/runs/<session>`；任务/导入 journal 按配置 UUID
存于 `profiles/<id>`，不随会话退出删除。“打开运行日志目录”可直接访问。
管理服务崩溃后的后台认证接管尚未实现；不会凭一个 PID 文件接管或杀死旧对象。

## 构建与验证（开发/部署）

独立只读桌面的原六包闭包仍成立；连接管理是可选包。仿真/本机通信功能需要另外安装
相应 EasyFly 平台、Gazebo、已验 PX4 输入、资源与兼容包。启动器自动加载所选安装
构建时记录的 underlay，不能用一个只有六个 UI 包的安装冒充完整仿真部署。
集成连接页由管理包提供 Qt 插件，通过 ABI1 在运行时装载；未连接的主窗口不创建
ROS context。启动或变更 domain 时，以冻结环境创建整个工作进程，再正常关闭空闲
入口窗口，因此不在运行中的 ROS context 上热改 domain。此交接不另留管理窗口。

```bash
source /opt/ros/humble/setup.bash
# 全平台安装在构建管理器前作为 underlay 加载；兼容输入也可在界面选择。
source /absolute/platform/install/setup.bash
connection_build_root=/absolute/path/new-connection-build
colcon --log-base "$connection_build_root/log" build --base-paths src \
  --build-base "$connection_build_root/build" --install-base "$connection_build_root/install" \
  --packages-up-to easyfly_connection_manager --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DEASYFLY_ENABLE_MEDIA=ON -DEASYFLY_ENABLE_RVIZ_EMBED=ON
source "$connection_build_root/install/setup.bash"
ros2 run easyfly_connection_manager easyfly_connections
```

录像和内嵌三维可在编译时分别关闭；相应页面解释未安装能力。安装了能力时页面固定可见，
不需要修改旧配置中的 recording/rviz_embed 开关；打开渲染器和开始录像仍须手动操作。
旧 schema2 两个字段继续读写以兼容原配置与冻结会话。显式请求未安装的可选能力仍会拒绝。
图形 `.desktop` 文件的 Exec 含安装前缀，搬迁后须重新生成/注册。联网 QGC 的独立窗口
关闭管理需要 X11/XWayland；没有 DISPLAY 时明确拒绝，不退回信号强杀。

使用同一新前缀运行测试，避免旧 bridge ABI/旧 Python/旧 scene 资源混用：

```bash
colcon --log-base "$connection_build_root/test-log" test --base-paths src \
  --build-base "$connection_build_root/build" --install-base "$connection_build_root/install" \
  --packages-select easyfly_connection_manager easyfly_ui_bridge easyfly_desktop \
  --executor sequential --return-code-on-test-failure --ctest-args --output-on-failure
```

单元/夹具测试不启动真实飞行。`test_live_session` 使用 localhost domain97/98，固定域
测试串行。真实 SITL/QGC、人工界面验收、长稳、四构建变体和实机分别登记，不能以组件
通过替代。协议/所有权细节见 [CONTRACT.md](CONTRACT.md)。
