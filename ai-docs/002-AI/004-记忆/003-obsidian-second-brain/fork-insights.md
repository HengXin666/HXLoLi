---
title: Fork 洞察
---

# Fork 洞察

对应原文: `ref/obsidian-second-brain/FORK_INSIGHTS.md`

## 文档性质

这是 roadmap 文档，不是规范。它分析了 166 个 fork 实际构建了什么，并把可回流上游的内容整理成 50 个具体 additions。优先级标记:

- `P0`: 最高 ROI 的快速收益。
- `P1`: 很强的改进。
- `P2`: 战略性或更长期。

分析日期: 2026-05-30。分析时仓库状态: 1,462 stars、166 forks。GitHub API 返回 167。

## 构建状态

截至 2026-05-31，项目已完成这个 roadmap 的大部分: 大约 50 项中的 32 项通过 PR #45-55 进入 v0.9.0 和 v0.10.0。v0.10.0 名为 “The Architect”。

所有 P0 都已落地。重点包括:

- 第一批 tests + CI。
- 反幻觉和 false-absence guard。
- 免费无 key 研究模式。
- Google Calendar commands。
- `/obsidian-recurring`。
- calendar reconciliation。
- `/vault-deep-synthesis`。
- `/idea-discovery`。
- `/obsidian-panel`。
- Codex executable runner。
- commit-decisions miner。
- substitution-character CI gate。
- sentinel-safe regeneration。
- `/obsidian-architect`，即 codebase -> vault docs。

当时命令数: 43。

## 有意不做

剩余项不是 backlog，而是评估后主动拒绝。理由是保持 skill 精简比覆盖每个 fork idea 更重要。

不做项:

- 细分职业命令，例如 `/obsidian-proposal`、`/obsidian-event`、`/obsidian-1on1`、`/obsidian-launch-block`。这些属于销售、活动、教学、课程发布等专业领域，应进 domain fork。
- TypeScript Obsidian plugins，例如 companion status、daily auto-open。当前环境无法构建测试，并且可移植性低。
- launchd review cadence。过于个人机器特定，通用需求已由 `claude -p` headless pattern 和 opt-in bg-agent 覆盖。
- `/obsidian-dashboard`。依赖 Dataview/Tasks 插件，且与已有命令重叠。
- `/obsidian-normalize`。用于迁移大型 legacy human-note corpus，但上游从一开始就是 AI-first。
- Nushell installers。无法在当前环境测试 Nushell。
- 中文 README。对 solo maintainer 来说存在持续翻译漂移维护负担。
- `/obsidian-roadmap`、repomix integration。已折叠或延后到精简版 `/obsidian-architect` 之后。

如果未来想做这些，就是一个新决策，不是疏漏。

## 方法

每个 fork 都通过 GitHub compare API 与 `main` 比较，获得精确 ahead/behind 数。对有分叉提交的 fork 深读实际文件内容。

“fork 创建后有 push”这个启发式被丢弃，因为不可靠。例如有些 fork 只是同步上游，虽然近期 push，但 ahead 为 0。

## Fork landscape

- 156 个 fork 是未改动镜像，0 commits ahead，没有信号。
- 11 个 fork 有自己的提交。
- 其中 3 个做了实质工作，8 个做了聚焦的单一用途工作。

强信号:

1. 付费 API 研究工具是采用门槛。
2. 多个 fork 想要日历集成。
3. 上游没有测试。
4. Codex/Windows 支持被多个 fork 独立需求验证。
5. 上游缺少 anti-hallucination guard。

## 50 个 additions 译注

### A. 免费 / 零 key 研究工具

这是最大采用障碍。

1. `P0` 整个 research toolkit 加 free mode: 新 `uv` 安装不需要 API key 也能跑。BYO-key 作为 opt-in。
2. `P0` 11 个无 key source clients: arXiv、Semantic Scholar、OpenAlex、CrossRef、Wikipedia、HackerNews、Reddit JSON、Lobsters、dev.to、DuckDuckGo、SearXNG fallback。
3. `P1` 把 synthesis 移入调用 Claude session: Python 只抓原始 JSON，命令体让 Claude 写 dossier，去掉付费 synthesis 依赖。
4. `P1` 并行 source aggregator 和 graceful degradation contract: ThreadPoolExecutor、30s timeout、至少 3 个来源成功算 success。
5. `P1` 24h TTL 文件缓存: polite UA + retries，缓存到 `~/.cache/obsidian-second-brain/research/`。
6. `P2` `--academic` source profile: arXiv + S2 + OpenAlex + CrossRef，相对默认 discourse profile。

### B. Calendar 和 scheduling commands

两个 fork 独立构建了这类能力。

7. `P0` 修 `/obsidian-daily` calendar bug: 命令引用的 placeholder tool name 与真实 MCP 工具名不匹配。
8. `P1` `/obsidian-agenda`: 指定范围的日历快照，检测冲突、连续会议、focus gap、外部组织事件，并把参与者链接到 person notes。
9. `P1` `/obsidian-schedule`: 写 Google Calendar，支持独立事件、从任务创建、建议时间；从 person note 的 `email:` 字段找邮箱，绝不猜。
10. `P1` `/obsidian-meeting`: 从日历事件生成会议笔记，Notes/Decisions/Action-items 为空，拒绝编造会议内容。
11. `P1` `/obsidian-calendar` reconciliation: 标记 vault 暗示但日历上没有的承诺，只标记，不写事件。

### C. 新 thinking / workflow commands

12. `P1` `/obsidian-panel`: 让 `Advisors/` persona notes 对一个决策各自给独立意见，再综合。
13. `P1` `/idea-discovery`: 扫描 Ideas、项目开放问题、孤立 Research notes，列出 3-5 个下一方向。
14. `P1` `/vault-deep-synthesis`: 纯 vault、无网络，围绕主题找一致点、矛盾、陈旧主张、覆盖缺口。
15. `P1` `/obsidian-recurring`: 跟踪周期性义务，维护 `next-due`。
16. `P2` `/obsidian-event`: 事件运营笔记，含 pre/day-of/post checklist。
17. `P2` `/obsidian-launch-block`: 一个 mother task 内部持有子任务 checklist。
18. `P2` `/obsidian-proposal`: B2B proposal generator，同时同步 `type: opportunity` deal tracker。
19. `P2` `/obsidian-1on1`: 结构化 1:1 捕获，生成连续 learning IDs。

### D. Codebase -> vault

20. `P1` `/obsidian-architect`: 把代码库扫描成可维护、可 diff 的架构笔记集，包含 overview、模块笔记、决策、persona、Mermaid。
21. `P0` Sentinel-based safe regeneration: `@generated:start/end` 会被覆盖，`@user:start/end` 永不触碰。它是防止覆盖用户编辑的通用写入原语。
22. `P1` commit-decisions mining: 扫最近约 200 个 commit，找决策形状的消息，作为 ADR 候选。
23. `P2` `/obsidian-roadmap`: 综合架构信号和研究，生成优先 backlog 和 board cards，依赖 architect。
24. `P2` repomix integration: 打包代码库供 LLM 综合，并有纯 Python fallback。
25. `P2` Personas inference: 从 README 显式 `## Personas` 提取，或让模型推断 2-5 个 persona 并标注置信度。

### E. 反幻觉和 vault integrity

26. `P0` False-absence guard: 错误说“没有某笔记”是最常见失败模式，必须通过列目录和 grep 验证。
27. `P0` Search-completeness rule: 穷举，不采样。列出每个匹配文件，而不是代表性几个。
28. `P0` Per-command anti-fabrication footer: 每个命令都加标准 hard rule block。
29. `P1` Read-before-edit guard 和 bounded-section discipline。
30. `P1` `/obsidian-dashboard` can't-fabricate pattern: 渲染 live query block，而不是可过期的静态列表。
31. `P2` `/obsidian-normalize`: interruptible、approval-gated、per-note diff 的 legacy note schema 校正。

### F. Scheduled-agent / automation infrastructure

32. `P0` PostCompact opt-in double-flag gating: 同时要求 `OBSIDIAN_VAULT_PATH` 和 `OBSIDIAN_BG_AGENT_ENABLED=1`。纯安全收益。
33. `P0` ship bg-agent inert: 默认 disabled，并提供粘贴模板。
34. `P0` 记录 `claude -p` slash-command 不展开的 workaround。
35. `P1` launchd review cadence。
36. `P1` idempotent per-period run-stamp idiom: 成功才写 stamp，失败会 retry，避免重复触发。
37. `P1` 无人值守运行只报告，不自动修复。
38. `P2` 后台 agent 只 add/update，绝不 delete/move/archive。

### G. Obsidian plugins

39. `P2` Companion status plugin: 状态栏显示今天/本周 agent 是否运行。
40. `P2` Daily auto-open plugin: 启动时打开今天 daily note。
41. `P2` stamp-file convention: 让插件/UI 读取 agent 运行状态。

### H. Platform 和 install

42. `P1` Codex executable invocation path: `run-command.sh` 和 PATH shims，把每个命令包装成真正的 `codex exec` 调用。
43. `P1` 面向 vault 的 AGENTS.md 模板，带 `## For future Codex` 前言。
44. `P1` Windows `setup.sh` rename fix: 用 `cat tmp > dest && rm tmp` 替代 `mv tmp dest`，解决 WSL mounts 和 OneDrive 同步目录上的 atomic rename 问题。
45. `P2` Nushell installers。
46. `P2` minimal/no-background-agent install profile。

### I. Quality, testing, CI

47. `P0` `tests/test_smoke.py`: 两个 stdlib tests，检查 codex-cli 构建产物和 `vault_health.py --json`。
48. `P0` CI workflow + pytest dev dependency。
49. `P1` source-level em-dash linter + pre-commit hook，扩展到完整 banned-char set 并加 CI。

### J. Docs, ecosystem, i18n, fork management

50. `P1` `DELTAS.md` fork customization template: 本地差异集中记录，便于 merge upstream。

Bonus:

- `ECOSYSTEM.md`: 定义 upstream-core/domain-fork contract。
- `README_zh.md` 和 language switcher。
- `--project` subtree routing 和 `/obsidian-board --refresh` from git-history mode。

## 不要复制什么

- 不要复制 pillar-vault fork 的哲学反转。它删除 `## For future Claude` 和 `ai-first: true`，这违背上游 AI-first 规则。可以拿 guard，不拿模型。
- 不要照搬 `/obsidian-notion-sync` 和 cron scripts。它们硬编码到个人 Notion、Discord、zh-TW。
- 不要合并坏掉的 AGENTS.md。全局 Claude->Codex find/replace 会破坏上下文。

## 第一波 8 个 P0

最值得先落的 8 件事:

1. 修 `/obsidian-daily` calendar tool-name bug。
2. 加 `tests/test_smoke.py` 和 CI。
3. 加 anti-fabrication、false-absence、search-completeness guards。
4. 用双 flag gate PostCompact bg-agent，并默认 inert。
5. research toolkit 加 free mode。
6. 采用 sentinel safe regeneration。
7. 文档化 `claude -p` slash-command expansion workaround。

## 对 HXLoLi 的启发

从这个 fork 分析里，`HXLoLi` 最应该优先学的不是日历或 Obsidian 插件，而是:

- P0 级健康检查和 CI。
- 反幻觉和搜索完备性规则。
- sentinel 安全再生成。
- codebase -> docs 的 `/obsidian-architect` 思路。
- free mode / graceful degradation: 自动化脚本没有外部 key 时也应该能退化运行。
- `DELTAS.md` 风格的本地差异记录: 对个人博客长期迭代很有用。
