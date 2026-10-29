# 远端算法管理

连接页的“远端算法管理”打开独立 Qt 客户端。远端机需先安装同一版本的 EasyFly，配置并主动运行服务；DDS 观测本身不会创建进程。也提供 `easyfly_remote_client` 命令行接口。

```bash
ros2 run easyfly_connection_manager easyfly_remote_service --config /absolute/private/server.json
ros2 run easyfly_connection_manager easyfly_remote_client --settings /absolute/private/client.json status
ros2 run easyfly_connection_manager easyfly_remote_algorithms
```

服务端／客户端模板位于本包 `config/remote-*.example.json`。复制后设为当前用户拥有的 0600 文件，再填写独立平台 UUID、客户端 UUID、证书、私钥、令牌路径、ROS domain 和预设。模板中的 UUID 仅示例，部署时分别生成新的 UUID。服务端状态目录必须是 0700 私有绝对路径，证书需由客户端信任且主机名匹配；连接不支持明文或忽略证书验证。令牌为 32–128 位 URL 安全随机字符，仅通过私有文件读取和 HTTPS Authorization 传递。

远端管理员预先限定算法及参数。目录当前支持彩线、椭圆、FAST-LIO、OpenVINS、FAST-LIVO2、ORB-SLAM3、GCOPTER、SUPER、FUEL 的已安装原生入口。客户端请求只含预设 ID，不可提交可执行文件、命令行、环境变量或 PID。定位／规划启动仍受各包现有输入、地图、标定和会话条件约束；提供远端生命周期管理不等于这些后端已取得硬件准入。当前定位 launch 中的 Gazebo 固定标定须按原包的软件仿真范围使用。

每次启动创建独占 UUID 目录，冻结预设摘要与实际 argv；ORB、FUEL 的运行目录在该会话内创建。全服务共享一个 ROS domain，持有状态目录锁和本机 domain 锁。已有同预设输出发布者会拒绝启动；不会收养或停止外部进程。多机 namespace／端口隔离另由 I-10 提供。

`grounded` 策略要求单一、新鲜、源时间持续推进的 MAVROS 状态与落地状态，已连接、上锁、在地面，并且已发现的动作状态无活动／未知任务，并核对发布源身份及观察线程心跳。FUEL 尚无首条动作状态时，仅接受其唯一原生服务 `/exploration/management_idle` 的新鲜只读空闲响应；该响应同时要求没有已接纳任务、活动任务、故障锁、计算进程或未决子任务／租约。定位和规划仅允许该策略。独立彩线／椭圆还可由管理员显式选择 `isolated_observation`，只在当前 domain 未发现飞控／飞行任务时启停，用于离线图像观测验收。

协议版本 1，状态返回平台 ID、服务启动 ID、预设摘要、真实进程状态、输出收件活动和操作条件。输出活动不代表算法有效性或飞行准入。每次操作携带平台／服务／客户端／请求／会话身份；请求在 SQLite 中先登记再执行，重复相同请求返回原收据，内容冲突被拒绝。停止只针对本客户端的精确会话，发送普通 SIGINT，并等待真实进程组排空；超时保留所有权，不自动升级信号。日志滚动保留最多两份，每份 1 MiB。

客户端失联或窗口关闭不会停止远端进程；过期状态禁用客户端操作。操作回复丢失时显示结果未知，先查询当前会话，不自动重发操作。服务停止请求会等待已拥有的进程正常结束。服务非正常退出后的非终态记录标为 `unresolved_after_service_restart` 并禁止重复启动，需管理员核对原进程与日志；不会依据相同进程名或裸 PID 自动接管。请求收据上限 10000 条，达到后拒绝新操作并要求停机维护归档。
