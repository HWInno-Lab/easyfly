# EasyFly 许可材料

现有自有代码已开始按用户选择实施许可修订。本工作区许可修订版为 **EF-LICENSE-20261103-02**，已落实首批65个包、731个登记文件；这不是整个 EasyFly 的产品版本号，也不表示已经对外发布。

- [根许可入口](../LICENSE)及[许可全文](../LICENSES/EasyFly-Restricted-1.0.txt)。
- [当前适用范围](scope.json)：精确路径、内容指纹、授权依据、保留范围及待核包。
- [迁移状态](MIGRATION-STATUS.md)：本轮结果、后续待处理项与实际验证边界。
- [商业与教学授权](AUTHORIZATION.md)：联系人及申请说明，本身不授予独立使用权。
- [GPL 例外](EXCEPTIONS.md)：当前仅明确授权 EGO 原创 ROS 2 移植贡献；其他自有代码不自动适用。
- [补充答复记录](followup-decisions-20261103.json)：来源由项目继续查询，暂无补充权属文件和上游授权，首发形式延至验收，暂不接收外部贡献。

当前范围基于作者的自有版权及公司授权陈述和本次现有代码迁移指令，不宣称完成独立版权法律审查。作者已确认此前自有代码未向他人提供；现撤换旧内部 Apache 声明，登记范围只按新协议提供本次免费许可。第三方材料、模型和待核组合没有统一改标；如查明实际存在合法有效的既有授权，仍须尊重。各已迁移包均自带 `licenses/`，安装时交付至 `share/<包名>/licenses/`。

## 检查当前工作区

```bash
python3 tools/source/verify_license_scope.py
python3 tools/source/verify_license_scope.py --require-ready
python3 -m pytest -q tools/source/test_license_scope.py tools/source/test_license_transition.py tools/source/test_d04_licenses.py
```

普通检查退出 `0` 表示已登记材料一致；材料缺失、漂移、越界或范围冲突退出 `1`。严格检查当前退出 `2`，表示整个工作区尚不能据此认定已完成分发许可核对。它不把哈希匹配解释为版权归属、组合兼容或发布批准。

原 `verify_license_transition.py` 在检测到当前 `scope.json` 时转入当前范围检查；没有当前范围的草案测试环境仍执行原草案检查逻辑。

检查覆盖97个非外部包的原始文件集合及新增许可包，并核对迁移范围外7,330个已跟踪基线条目。被忽略的 SDK／模型载荷、Git 子模块载荷以及已盘点包树之外的所有新包不因此成为完整离线交付或全仓 SBOM。新增文件须先登记许可和范围，不能只更新哈希或默认继承受限许可。

## 修订依据与技术追溯

[未对外提供陈述](prepublication-declaration-20261103.json)记录作者原话；依据是作者陈述，未冒充独立传播历史调查。[history/README.md](history/README.md)说明保留的技术指纹及其用途。旧自有源码片段及原拟议许可的本地审阅备份已于2026-11-04随 artifacts 清理删除；当前技术指纹、正式许可、作者决定与来源选集继续保留。

当前自有代码的包许可元数据只列新许可；车辆重识别模型另外保留其上游 MIT。保留的 Apache 全文仅供第三方条件与条款参照使用，不构成对登记自有代码的新增 Apache 许可。

[D-04 当前记录](d04-current.review.json)现以明确 GPL 例外解决 EGO 原创移植的作者许可选择，当前仍有3项外部证据／组合缺口。原材料与2026-11-02记录保持不变；EGO 包内当前元数据及交付材料同步仍待完成，不能把作者许可决定当成已批准分发。当前范围检查器会一并核对新例外和当前记录。

本目录保留的 `.draft.txt`、`scope.candidate.json`、`file-scope.review.json`、旧用途示例和起草记录是前序审阅背景，不是当前适用范围。未来贡献规则草案尚未完成贡献者接受流程，不能推定外部贡献者已授予公司商业再许可权。当前状态以 `scope.json` 和迁移状态文档为准。
