# EasyFly 许可迁移审阅清单

审阅基线：`b509b0f872743c98b16e12624b687baf56f525f3`。日期：2026-11-03。

历史阶段说明：下文保存切换前的审阅清单，其中“尚未切换”“待选择”等表述对应当时状态。用户随后已选择同时处理现有自有代码；本工作区已实施首批范围，当前结果以[迁移状态](MIGRATION-STATUS.md)和[scope.json](scope.json)为准。切换前完整原稿保存在本地审阅备份；history/20261103-pre-transition/ 现只交付必要技术指纹，本清单本身不授予许可或批准发布。

## 1. 本轮可审阅的修改

| 文件 | 拟议作用 | 正式落实前的处理 |
| --- | --- | --- |
| [EasyFly-Restricted-1.0.draft.txt](EasyFly-Restricted-1.0.draft.txt) | 自有贡献的有限用途免费许可全文 | 按最终范围复核、去除起草说明，作为独立许可全文交付 |
| [ROOT-LICENSE.draft.txt](ROOT-LICENSE.draft.txt) | 根许可入口的替换提案 | 填写实际发行标识、正式范围及全文路径后审阅；当前不覆盖根 LICENSE |
| [AUTHORIZATION.draft.txt](AUTHORIZATION.draft.txt) | 商业／教学申请渠道和所需信息 | 复核后随正式文本交付；申请本身不构成授权 |
| [CONTRIBUTION-POLICY.draft.txt](CONTRIBUTION-POLICY.draft.txt) | 后续贡献的权利及再许可安排 | 明确贡献者接受方式；不追溯约束历史外部贡献 |
| [Apache-2.0 原文](../LICENSES/Apache-2.0.txt) | 保存既有 Apache 文本 | 保持原文，并保留对应历史通知及有效授权 |
| [scope.candidate.json](scope.candidate.json)／[file-scope.review.json](file-scope.review.json) | 121包、97个非外部包的逐文件技术证据 | 只作候选，不直接改名当作正式范围 |
| [transition.review.json](transition.review.json) | 审阅材料指纹、空批准清单和待决事项 | 与实际审阅差异一同更新，不用哈希替代权利证据 |

许可方为江西穹瞰无人机科技有限公司，公开作者与其本人自有贡献的版权人为贾国源，联系邮箱为15103443485@163.com。公司依据作者授权对外许可，不宣称拥有整个仓库的版权。

免费方向为个人学习、非商业实验、学术研究；商业资助的研究本身不自动失去免费资格。内部免费豁免仅限上述公司，其他企业内部商业活动需要单独商业授权。教学无论收费与否均需要单独教学授权。再分发、必要专利、30日纠正与恢复、中文为准及法律／争议处理按全文审阅稿落实。

## 2. 现有材料如何处置

| 范围 | 包数 | 当前迁移处置 |
| --- | ---: | --- |
| 外部源码／驱动包 | 24 | 保留已有许可，不纳入本轮受限许可转换候选 |
| 项目维护但权利及组合待核实 | 91 | 暂保留已有许可；名称、Git 作者及 EasyFly 头部声明不单独构成转换依据 |
| 模型及场景混合资源 | 2 | 按资源来源逐项核对；不对整个包批量改标 |
| 混合或其他许可包 | 4 | 按下表拆分；现有许可与通知继续保留 |

全量包路径、现有标识及分类以候选 JSON 为准。2,643个文件／链接条目全部处于未批准状态；没有“默认原创即可切换”的条目。

| 重点包 | 仓库内已核对的依据 | 具体处理要求 |
| --- | --- | --- |
| easyfly_gcopter_core | [CMakeLists.txt](../src/algorithms/planning/easyfly_gcopter_core/CMakeLists.txt) 使用第三方 GCOPTER 头文件，并安装其 LICENSE；包声明 MIT | 不能将导出的第三方头文件改为自有受限许可；自有封装若拟另行许可，应单独明确文件与权利 |
| easyfly_mavros_runtime | [来源记录](../src/communication/c2_protocol/easyfly_mavros_runtime/upstream/ORIGIN.json)记录 MAVROS 2.14.0 归档及文件指纹；[构建入口](../src/communication/c2_protocol/easyfly_mavros_runtime/CMakeLists.txt)编译相关源码并安装上游通知 | `owned_mavros` 等目标名称不证明版权归属；保留 BSD 等原声明，逐项识别实际自有修改 |
| easyfly_gazebo_plugins | [构建入口](../src/simulation/easyfly_gazebo_plugins/CMakeLists.txt)分别构建多个插件；包声明 MIT 和 Apache-2.0；包含继承及混合来源 | 按插件／文件核对，不把整个插件集合统一改标 |
| easyfly_openvins_integration | [包元数据](../src/perception/p3_tracking/easyfly_openvins_integration/package.xml)声明 Apache-2.0 与 GPL-3.0-or-later；[构建入口](../src/perception/p3_tracking/easyfly_openvins_integration/CMakeLists.txt)主要安装 Python、launch、config 等 | 元数据不能单独证明实际链接或派生关系；结合具体脚本、上游代码及交付方式判断，保留原声明 |
| easyfly_gazebo_models／easyfly_gazebo_worlds | 包分类为混合资产，技术清单记录模型、贴图、场景等条目 | 对无法确认再许可权利的资产保留原状或排除交付；自有代码的新许可不覆盖资产缺口 |

技术清单中22个包存在通往 GPL 标识包的声明依赖路径。这些路径包含运行期等关系，不能据此把22个包一律判为 GPL，也不能据此宣告新受限许可兼容；应优先检查实际包含、链接、复制和分发关系。

## 3. 正式范围还需要的决定

| 项目 | 已有结论 | 尚需落实 |
| --- | --- | --- |
| 授权主体 | 作者持有本人自有贡献版权并授权公司对外许可 | 对拟纳入的具体贡献核对实际权利及必要授权材料 |
| 适用起点 | 历史有效 Apache 权利不撤销 | 选择先覆盖后续新原创贡献，还是同时逐项审查现有自有贡献；当前正在征求选择 |
| 发行标识 | 基线提交已记录 | 确定实际生效发行／提交；当前没有虚构版本号或生效日期 |
| 正式范围 | 所有现有条目未批准 | 按确定策略登记精确文件／贡献、内容指纹、既有许可及权利依据；不使用未经审查的目录通配授权 |
| 最终文字与交付 | 第二轮全文及根入口提案已备齐 | 对照选定范围复核正文、README、源码标识、包元数据及实际安装产物；法律审阅尚未完成，建议另行完成 |

若选择只覆盖后续新增原创代码，不能把“修改日期较新”当成新原创证明，也不能把原 Apache／第三方文件简单改名、搬迁或删头后纳入。新旧贡献混合时，应标明可分辨的贡献及相应依据，无法分清则继续保留原状待审。

若选择同时审查现有自有代码，逐项确认后可讨论未来发行的许可安排，但不得宣称既有有效 Apache 权利失效。同一既有材料仍存在有效 Apache 授权时，新许可不能产生排除该既有权利的效果。[依据：Apache-2.0 第2、4节](https://www.apache.org/licenses/LICENSE-2.0)。受限条款使用独立名称，不能仍称为 Apache-2.0。[依据：Apache 官方许可 FAQ](https://www.apache.org/foundation/license-faq.html#mod-license)。

## 4. 实施顺序及检查边界

1. 确定适用起点，形成精确且经权利核对的正式范围。当前批准清单为空。
2. 对选定范围定稿全文及根入口；保留历史 Apache 与各项第三方通知。逐文件改标，混合包使用明确的范围说明，避免全仓替换。
3. 同步实际受影响的 README、包元数据和安装规则。验证源码交付及实际安装产物内均能找到对应全文、范围与通知。
4. 保存范围差异及检查结果。正式发布按届时明确的发布指令执行。

新增检查工具验证当前审阅材料一致性，并拦截越界路径、重复字段、源文件漂移、已盘点包树内新增未登记文件、外部包误分类、草案中夹入批准项、作者陈述范围扩大、原许可／通知缺失。它不接受把草案直接设置为“已生效”。

```bash
python3 tools/source/verify_license_transition.py
python3 tools/source/verify_license_transition.py --require-ready
```

普通检查通过不表示可以发布。当前严格检查应退出 `2`，明确报告正式范围、权利及组合判断均未完成。完整覆盖边界见[目录说明](README.md)。

## 5. D-04 保持独立开放

旧 ikd-Tree 精确来源的授权缺口、FAST-LIVO2／VIKIT 的组合问题、FUEL 捆绑 LKH 的版本与分发证据仍是3项开放问题。EGO 原创 ROS 2 移植新增部分原已确认的 Apache-2.0 授权继续保留。

依据为[D-04 记录](../src/third_party/license_reviews/d04-20261102/review.json)。本轮自有许可迁移不覆盖这些证据缺口，不改写其关闭状态，也没有向第三方发送授权申请。
