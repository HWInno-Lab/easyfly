# EasyFly 现有代码许可迁移状态

许可修订版：**EF-LICENSE-20261103-02**；日期：2026-11-03。基线提交：`b509b0f872743c98b16e12624b687baf56f525f3`。

用户选择同时处理现有自有代码。当前工作区已完成首批65个包、731个登记文件的许可声明变更，保留原版权通知，并按作者“未向他人提供、仅本人本地保存”的陈述撤换旧 Apache 声明；不是全仓统一改为仅受新许可约束，也没有对外发布。

## 已落实的内容

- 根 [LICENSE](../LICENSE) 改为按范围区分的统一入口；[许可全文](../LICENSES/EasyFly-Restricted-1.0.txt)落实既定用途、专利、纠正恢复及法律条款。
- [scope.json](scope.json)精确登记当前文件与指纹、原始指纹、授权依据、保留项及未完成包。依据是作者自有版权与授权公司陈述及本次现有代码迁移指令，**不宣称完成独立版权法律审查**。
- 首批65包同步文件标识、package.xml 与本包 `licenses/` 安装规则，安装位置为 `share/<包名>/licenses/`。JSON和不支持相应注释的模板依靠精确范围声明，没有插入破坏格式的注释。
- 登记自有代码只列新许可，不继续并列 Apache 选项；旧自有 Apache 正文已移出当前源码头，原版权署名保留。车辆重识别模型另列其上游 MIT；Apache 全文仅用于第三方和条款参照。旧自有源码和原拟议声明仅在本地审阅备份保留，公开技术快照不包含这些源码片段。
- 模型、二进制测试载荷、外部通知及符号链接等7个首批包内条目保留；例如车辆重识别模型的 Kaiyang Zhou MIT 通知没有改为自有版权。
- 其余32个非外部包、24个外部包未转换。非外部原始盘点共2,643条，其中731条已登记、1,912条保留。迁移范围外另有7,330个已跟踪基线条目受指纹保护。

本轮新增关键事实：[作者确认](prepublication-declaration-20261103.json)此前自有代码从未提供给他人，仅本人本地保存。因此按此陈述在首次对外提供前撤换旧拟议 Apache 声明，当前登记自有代码不提供 Apache 并行选项。这不是撤销已经有效授予他人的许可；如发现相反的实际授权事实，仍须依法处理。[Apache-2.0 第2节](https://www.apache.org/licenses/LICENSE-2.0)。

## 尚未转换的32个非外部包

以下是待核事项，依赖线索并不单独证明派生关系或不兼容。各包继续依原声明及实际来源条件处理；不得把“待核”解释为无条件获得新的商业分发许可。

| 包／路径 | 继续处理的原因 |
| --- | --- |
| [easyfly_connection_manager](../src/easyfly_ui/easyfly_connection_manager/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_demos](../src/application/a3_mission/easyfly_demos/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_desktop](../src/easyfly_ui/easyfly_desktop/package.xml) | 存在其他许可或引用线索 |
| [easyfly_desktop_tests](../src/easyfly_ui/easyfly_desktop_tests/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_fast_lio_integration](../src/perception/p3_tracking/easyfly_fast_lio_integration/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_fast_livo2_integration](../src/perception/p3_tracking/easyfly_fast_livo2_integration/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_fastdds_compat](../src/simulation/easyfly_fastdds_compat/package.xml) | 发现其他版权通知 |
| [easyfly_fleet_sim](../src/simulation/easyfly_fleet_sim/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_flight_control](../src/execution/e3_control/easyfly_flight_control/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_fuel_core](../src/algorithms/planning/easyfly_fuel_core/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_fuel_exploration](../src/execution/e2_planning/easyfly_fuel_exploration/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对；存在其他许可或引用线索 |
| [easyfly_gazebo_models](../src/simulation/easyfly_gazebo_models/package.xml) | 模型／场景混合资源；发现其他版权通知；存在其他许可或引用线索 |
| [easyfly_gazebo_plugins](../src/simulation/easyfly_gazebo_plugins/package.xml) | 混合或其他许可；含嵌入外部路径；发现其他版权通知；存在其他许可或引用线索 |
| [easyfly_gazebo_worlds](../src/simulation/easyfly_gazebo_worlds/package.xml) | 模型／场景混合资源；存在其他许可或引用线索 |
| [easyfly_gcopter_core](../src/algorithms/planning/easyfly_gcopter_core/package.xml) | 混合或其他许可 |
| [easyfly_global_planner](../src/execution/e2_planning/easyfly_global_planner/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_local_planner](../src/execution/e2_planning/easyfly_local_planner/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_localization_failover](../src/perception/p3_tracking/easyfly_localization_failover/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_mavlink_bridge](../src/communication/c2_protocol/easyfly_mavlink_bridge/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_mavros_runtime](../src/communication/c2_protocol/easyfly_mavros_runtime/package.xml) | 混合或其他许可；含嵌入外部路径；发现其他版权通知；存在其他许可或引用线索 |
| [easyfly_mission_fsm](../src/execution/e1_state_machine/easyfly_mission_fsm/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_mission_srvs](../src/application/a2_services/easyfly_mission_srvs/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_object_detection](../src/perception/p2_detection/easyfly_object_detection/package.xml) | 存在其他许可或引用线索 |
| [easyfly_openvins_integration](../src/perception/p3_tracking/easyfly_openvins_integration/package.xml) | 混合或其他许可；存在其他许可或引用线索 |
| [easyfly_orbslam3_integration](../src/perception/p3_tracking/easyfly_orbslam3_integration/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_precision_landing](../src/execution/e3_control/easyfly_precision_landing/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_px4_sitl](../src/simulation/easyfly_px4_sitl/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_sim_launcher](../src/application/a3_mission/easyfly_sim_launcher/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_super_planner](../src/execution/e2_planning/easyfly_super_planner/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_swarm_planner](../src/execution/e2_planning/easyfly_swarm_planner/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |
| [easyfly_ui_bridge](../src/easyfly_ui/easyfly_ui_bridge/package.xml) | 存在其他许可或引用线索 |
| [easyfly_web](../src/easyfly_ui/easyfly_web/package.xml) | GPL声明依赖链，待实际包含／链接／运行及分发核对 |

实际包含或链接 GPL 内容的部分，应先确定能否取得相应许可或替换／隔离；仅运行期调用的独立原创包装应依据实际关系另行判断。不能把依赖路径删除或改一个包标签作为解决。混合来源需分开识别自有贡献与继承部分；模型场景需核对来源及再许可权。

## 保留原许可的24个外部包

`bspline_opt`, `easyfly_fuel_native`, `easyfly_orbslam3_core`, `ego_planner`, `ego_to_mavros`, `fast_lio`, `fast_livo`, `librealsense2`, `livox_ros_driver2`, `livox_sdk2`, `ov_core`, `ov_init`, `ov_msckf`, `path_searching`, `plan_env`, `quadrotor_msgs`, `realsense2_camera`, `realsense2_camera_msgs`, `rog_map`, `sophus`, `super_planner`, `traj_utils`, `traj_utils_so`, `vikit_common`。

EGO 原创 ROS 2 移植新增部分曾因撤换旧 Apache 安排重新待核；用户随后同意兼容上游的许可例外，现已单独登记 GPL-3.0-only 作者许可。D-04 **当前共3项开放**：ikd-Tree、FAST-LIVO2／VIKIT、LKH，见[当前复核记录](d04-current.review.json)。原记录及原授权文件未被改写；当前关闭的是 EGO 作者许可选择，不是其包内材料同步或全部组合分发检查。新受限协议不自动适用于 EGO 的继承 GPL 内容。

## 验证记录与后续

已完成以下验证：

- 当前范围材料及7,330个范围外已跟踪条目的指纹检查；严格分发检查继续报告未完成，不能宣称全平台已完成法律或分发审查。
- 首轮许可相关测试49项通过，覆盖新范围、原草案状态和 D-04；重复字段、未知标识、越界、范围扩大、文件漂移、通知丢失等负例被拦截。
- 对731个登记文件逆向移除本次精确许可编辑，全部恢复到原始 SHA-256，确认只改许可注释、包许可元数据和安装规则；184个 Python 源文件语法检查通过。
- `easyfly_utils` 在真实 ament 环境下复制安装和软链接安装均通过。复制安装搬迁前缀并移走测试源码后，全文、Apache 原文和本包范围仍完整可读；软链接安装验证以源码存在为前提。该包 C++ 几何契约测试编译、运行通过。
- 其余64个包未逐包重建，未声称验证脱离源码的软链接安装搬迁，也没有运行全平台或飞行复验。

另外发现8份原有第三方／D-04许可证据文本被全局 `*.txt` 规则忽略。本轮保留原内容及修订前指纹，仅补交付例外，使这些文本可随源码入库；没有发送授权申请，也没有因此关闭权利缺口。名单见 scope.json 的 `additional_retained_notice_deliverables`。

历史工作证据原保存在本地 `artifacts/license-transition/`，已于2026-11-04按用户要求清理；原始日志及私有备份不再存在。正式范围、许可全文、作者决定、来源选集和公开技术指纹仍位于源码材料中，不依赖已删除目录取得许可。

后续继续处理上述32个包及资产边界，再完善实际发行组合、安装交付、外部贡献接纳流程和法律审阅。未来外部贡献不能因提交行为而被默认为已授权公司商业再许可。正式发布另按明确的发布指令执行。

## 首发前撤换 Apache 声明的追加记录

许可修订版由01更新为02。依据作者未向他人提供的陈述，首批731个登记文件撤换原 Apache 许可正文，65个包不再为自有代码并列 Apache 标识；模型 MIT 和第三方通知仍保留。未登记自有内容不从旧内部标签推定取得本次 Apache 授权，正式交付前仍须完成32个包等剩余范围。

01版的49项测试和安装记录是前一轮证据；02版验证另行保存，不覆盖旧结果。未改变产品运行逻辑及 CMake 安装规则，仅调整许可注释、包许可元数据和许可材料；源代码与安装边界重新核验。

02版追加验证完成：许可相关测试56项通过；731个登记文件与修订前快照的 AST／非注释词法／文本比较确认只变更许可注释及包许可字段，184个 Python AST 与191个 C++文件的非注释词法内容保持一致。`easyfly_utils` 新版复制安装、软链接安装及复制前缀搬迁后脱离测试源码的许可材料读取均通过，安装范围明确标记自有代码不新增 Apache 授权。该包几何契约重新编译运行通过；其余64包未重建，未作全平台／飞行验证。

## 补充答复与 GPL 例外

用户已集中答复：暂无额外权属文件和上游授权；公开及仓库来源由项目继续查询；可对确需兼容上游的自有移植／修改贡献作例外；首次交付形式延至验收补充；当前暂不接收外部代码／资源贡献。暂无材料不等于确认所有私有权利问题不存在。

已按此决定为 EGO 原创 ROS 2 移植贡献登记 GPL-3.0-only 例外，见 [EXCEPTIONS.md](EXCEPTIONS.md)。这解决作者许可选择，当前 D-04 回到3项开放；原4项记录保留为中间历史阶段。例外不附加主协议的商业／教学限制，也不改变其他登记自有代码的许可。EGO 源码、旧来源补丁及旧记录保持不变；实际包内标识、安装通知和发布组合仍须在正式交付前同步核对。

首批受限许可范围仍为65包、731个文件；32个非外部待核包和24个外部包的现有分类没有因这一份例外而批量改写。当前明确例外单独识别155个上下文文件中的实际原创贡献，不把这些文件全部认定为自有。

本轮公开来源复查见[source-review-followup-20261103.json](source-review-followup-20261103.json)：精确旧 ikd-Tree 树未发现许可命名文件，当前 main 许可文本不用于倒推旧版本；FAST-LIVO2 #413 在实时 API 中仍开放且评论为0；LKH 大学页面仍未补齐捆绑版本及分发许可。三项保持待核，没有发送联系邮件或授权请求。

本轮许可相关测试64项通过，新增负例拦截“把 GPL 例外扩成全部 EGO”“继续附加用途限制”“伪称包内已同步”“用作者例外关闭其他上游缺口”等错误。EGO 155个文件按原始选集与补丁零模糊重建通过；源码、旧补丁及既有第三方通知本轮未改动。

## 2026-11-04 临时产物清理

用户要求清理 artifacts。旧构建验证、原始测试日志、截图、离线归档和私有审阅备份随之删除，历史结果仅作历史记录，不能再声称对应原始文件仍可读取。当前正式许可、例外授权、源代码及其原始选集不在本次删除范围。运行必需的两项输入迁入本机 dependencies，并经现有校验器核对；未改变许可条款或据此关闭任何来源缺口。
