# Agent Note: Agent Notes 工作流容错并评论代码片段

Status: implemented

Decision-ID: agent-notes-advisory-comments

## Problem

扫描器直接决定 CI 退出码时, 双链欠账或工具故障会让工作流持续失败. 只在失败后向 PR 总讨论区发报告, 无法在工作流保持成功时反馈问题, 也不能把诊断放到对应代码片段

## Code

- `.agents/skills/hx-agent-notes/scripts/github/report.ts`
- `.agents/skills/hx-agent-notes/scripts/tests/test_report.ts`
- `.agents/skills/hx-agent-notes/scripts/tests/github/test_grouping.ts`

## Decision

GitHub diff, 主线全量和报告工作流的 job 与步骤容错完成. 标准库收集器通过 python3 调用 uv 中的严格扫描器, 保存原始 ok 和全部 issues, 有违规或工具错误都正常退出. 扫描异常, 缺 uv 或缺比较端点产出 scanner-error, 不把故障解释为空 diff 或有效双链

报告工作流监听两种扫描的完成事件, 使用默认分支可信 TypeScript 实现读取诊断 artifact, 脚本目录固定 CommonJS 以兼容宿主项目的 ESM 配置. 校验版本, 必填 head SHA, ok 一致性, 诊断级别和所有路径与行号. PR 当前 head 必须仍匹配扫描 SHA, fork 数据不作为可执行代码加载. 报告缺失产出 missing-report 评论, 发布失败写 warning

PR 问题优先放到对应代码行, 删除与改名前的代码使用 LEFT, 正文链接指向删除前的基线提交和旧路径. note 端问题优先映射 related 源码. push 分页读取提交文件并使用对应 patch position 评论. 每种扫描按决策 Markdown 路径聚合诊断, 正文以 note 为标题列出关联源码及准确行号, 规则和原因. 未关联决策的诊断单独列出, 不根据文件名推测关联, 重跑更新已有机器人评论, diff 与全量报告分别保留. 超出行内范围或 API 拒绝的诊断汇总精确路径和行号. 完整 JSON 保留全部问题, 已修复问题的旧评论保留为 review 记录

评论分别显示结构或工具错误与待审核项. diff 评论注明实际 base 到 head 的比较范围, 多提交 push 包含整个范围, 不声称累计诊断只属于最后一个提交. 错误优先使用 40 组的行内评论预算; 汇总保留全部数量与规则统计, 每条评论只展开 8 项诊断, 其他细节继续保留在 JSON artifact

审核评论的级别和原因, 报告日志及 Actions 摘要使用中文, 标点只用英文半角. 规则 ID, 路径, 行号, SHA 和 JSON 字段保留原值, 外部工具原始错误附在中文说明后

本地 CLI 保留 0, 1, 2 退出码. 双链机械契约由 `.agents/notes/implemented/process/2026-10-07-agent-notes-ast-redline.md` 拥有, CI 成功只证明流程完成. 安装模板和技能指引同步采用评论反馈, 不把始终成功的 CI 当作双链有效的 required check. GitLab 适配器保存同格式 artifact 并设置 allow_failure

## Alternatives considered

- 什么都不做 / 复用现有: 能直接用失败状态阻止合入且实现较短, 但违反工作流始终成功和问题定位到代码片段的要求
- 只给扫描命令追加容错: 改动小且能避免违规导致失败, 但会漏掉依赖错误, 丢失诊断 artifact, 原来的 failure 触发条件也不再发布评论
- 只发一条 PR 总评论: API 少且没有 diff 行号约束, 但无法让评审者在对应代码片段看到问题, 因此只保留为无法行内定位时的兜底
- 展开所有待审核位置: 不需要打开 artifact 就能读取完整清单, 但迁移时的大量资源提示会掩盖真实错误, 因此用数量, 规则统计和有限具体位置呈现汇总
- 在 PR 工作流直接持写权限发表评论: 链路短且无需跨工作流 artifact, 但 fork PR 无法可靠取得写权限, 加载贡献者代码还会扩大 token 暴露范围
- 让严格扫描器始终判有效: 所有入口都能保持成功, 但会隐藏断链, 错误位置和未配对改动, 因此只在 CI 外层容错

## Consequences

工作流成功与双链有效是两个独立结果, 评论和 JSON 保留实际诊断. 依赖和 API 故障不阻塞 CI, 仍可从 artifact, 日志与 warning 调查. 默认分支必须先部署可信脚本和工作流, Actions 写评论权限决定远端评论能否发布

## Testing

使用隔离 Git 仓库验证双链删除, 改名, 代表文件错误, 重复锚点和入站引用. 收集器验证违规, 缺失比较端点和依赖故障仍退出 0. 模拟 GitHub API 验证同一决策的多源码合并与稳定评论身份, PR 与 push 定位, 删除侧, 幂等更新, 无 patch, API 失败, 过时 head 和不可信 artifact

多提交 push 回归验证 50 项资源待审核后出现的真实锚点错误仍放到代码行, 汇总显示精确比较范围和待审核总数, 只展开 8 项, 原始 JSON 的 51 项及顺序保持完整
