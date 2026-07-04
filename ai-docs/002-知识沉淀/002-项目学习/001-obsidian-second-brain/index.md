---
title: obsidian-second-brain 中文译注
---

# obsidian-second-brain 中文译注

来源项目: `ref/obsidian-second-brain`  
来源版本: `bf63932`  
译注目的: 为后续评估 `HXLoLi` 可以学习哪些知识库/AI 工作流设计做准备。

> [!TIP]
> 这里不是覆盖原项目的“本地化发行版”，而是给 `HXLoLi` 知识库使用的中文译注。译注保留命令名、路径、frontmatter 字段、脚本名、代码块和关键英文术语，避免破坏可回溯性。

## 翻译策略

“无损耗”在这里按工程文档处理:

- 不覆盖原文: 原始项目仍在 `ref/obsidian-second-brain`。
- 不翻译机器接口: 例如 `/obsidian-save`、`ai-first: true`、`type: project`、`[[wikilinks]]`、`scripts/build.sh`。
- 不改写结构含义: 中文表达尽量贴近原文，不把规则压缩成普通摘要。
- 术语保留原形: Claude、Obsidian、vault、frontmatter、MCP、Grok、Perplexity、Gemini、NotebookLM 等保留英文或中英并列。
- 可继续对照: 每个译注文档都标明对应的原文文件。

## 项目一句话翻译

`obsidian-second-brain` 是一个跨 CLI 的 Obsidian AI 工作流工具包。它把任意 Obsidian vault 变成面向未来 AI 读取和维护的“第二大脑”: 新资料不会只是追加成孤立笔记，而是会重写相关旧页面、传播到项目/人物/任务/日志、处理矛盾、形成综合页，并通过命令、脚本和可选 hook 让知识库持续变聪明。

## 原项目核心主张

原项目的 README 把它定位为 Karpathy 的 LLM Wiki 模式的演进版:

- Karpathy 模式: 丢入资料, LLM 生成 wiki 页面, 再向 wiki 提问。
- 这个项目的增强: 新资料会更新已有页面，而不是只追加新页面。
- 这个项目的增强: 矛盾会被自动发现、解决或记录。
- 这个项目的增强: 模式和洞见会被主动综合成新页面。
- 这个项目的增强: 定时 agent 可以维护 vault，例如日终总结、周回顾、矛盾扫描、健康检查。
- 这个项目的增强: 笔记格式优先服务未来的 Claude 检索和推理，而不是优先服务人工阅读。

## 与普通博客/笔记系统的差异

| 维度 | 普通博客或静态笔记 | obsidian-second-brain |
|---|---|---|
| 写入方式 | 人手写文章或笔记 | AI 根据对话、资料、命令写入 |
| 知识更新 | 新文章通常追加 | 新信息会改写相关旧笔记 |
| 关联方式 | 标签、目录、手动链接 | 强制 `[[wikilinks]]`、索引、日志、传播规则 |
| 面向读者 | 人 | 未来的 AI 代理和人 |
| 自动化程度 | 构建、发布、搜索 | 保存、提取、综合、矛盾解决、健康检查 |
| 错误控制 | 靠作者检查 | 反幻觉规则、穷尽搜索、来源和置信度 |
| 历史记录 | Git/文章时间 | 双时间事实、操作日志、raw 原文不可变 |

## 分层架构翻译

对应原文: `architecture.md`

### 系统概览

`obsidian-second-brain` 是一个跨 CLI 的 skill，不是 Obsidian 插件，也不是托管服务。它把任意 Obsidian vault 转成 AI-first 的第二大脑。一个平台中立的命令源会在构建时编译到多个 AI CLI:

- Claude Code
- Codex CLI
- Gemini CLI
- OpenCode
- Hermes
- Pi

运行时，slash command 把用户 vault 当作普通 Markdown 文件读写。凡是确定性的工作，例如健康检查、研究抓取、代码库扫描，则交给 Python/Shell 脚本处理。

项目要点:

- 44 个命令，按 frontmatter 的 `category:` 分为 `vault`、`thinking`、`research`、`meta`。
- 43 个命令跨平台。`/obsidian-calendar` 依赖 Google Calendar MCP，所以只给 Claude Code 和 Pi。
- 研究工具默认可以无 key 使用免费公开来源；有 key 时可使用 Grok、Perplexity、Gemini。
- 可选后台 agent 和用户自定义定时 agent。
- MIT 许可证。

### 适配器模式

核心思想: `commands/` 是唯一真相源，构建系统把它编译到不同平台，而不是维护多套命令。

- `commands/<name>.md` 使用 Claude Code slash command 的形状，并声明 `description:`、`category:`、`triggers_en:`、可选 `exclude:`。
- `scripts/build.sh` 编排 `adapters/` 层。`bash scripts/build.sh` 构建所有平台，`--platform <name>` 只构建一个平台。
- Claude Code 适配器基本是原样复制。
- 其他适配器会生成一个 dispatcher 文件，例如 `AGENTS.md`、`GEMINI.md` 或平台等价文件，并根据每个命令的描述、分类和语言生成路由表。
- 针对非 Claude CLI，会把 Claude 特定说法中和成平台无关表达，例如 `Read tool` 改成 `read files`。
- 输出在 `dist/<platform>/`，该目录被 gitignore，任何时候都应该重新生成而不是手写修改。

对贡献者的结论: 增加或修改命令时，只编辑 `commands/<name>.md`，下一次构建由适配器自动拾取。

### 仓库布局

| 路径 | 作用 |
|---|---|
| `commands/` | 44 个 slash command 定义，是平台中立的产品表面 |
| `references/` | 共享规范，尤其是不可绕过的 `ai-first-rules.md` |
| `scripts/` | 构建、vault 工具、研究工具、代码库扫描器 |
| `adapters/` | 平台翻译层，每个平台一个 adapter |
| `hooks/` | Claude Code hook: AI-first 校验、上下文注入、后台 agent |
| `dist/` | 各平台构建产物，gitignore |
| `tests/` | smoke test 和 CI fixture |
| `examples/sample-vault/` | 示例 AI-first 笔记 |
| `SKILL.md` | skill 激活后的完整操作手册 |
| `architecture.md` | 架构说明 |
| `README.md` | 面向 GitHub 的公开文档 |
| `pyproject.toml` | Python 依赖，使用 `uv` 管理 |

### 运行时数据流

```text
用户运行 /obsidian-save
        |
        v
命令正文: 来自 dist/<platform>/, 由 commands/ 编译
        |
        |-- 引用 references/ai-first-rules.md
        |-- 调用 scripts/* 处理确定性工作
        v
把 AI-first Markdown 写入 vault
        |
        |-- Claude Code 下由 validate-ai-first.sh 校验
        |-- 传播到 index.md、操作日志、链接笔记、daily note
        v
vault 更新完成; 后续 /obsidian-world 和 load_vault_context.py 再读回状态
```

## 设计原则翻译

对应原文: `architecture.md` 的 Key design principles。

1. 一个源，多平台。`commands/` 是规范来源，adapter 只负责编译，不分叉维护。
2. 不符合 AI-first 就不发布。`ai-first-rules.md` 是不可协商的写入规范，并在写入时尽量机械校验。
3. 确定性工作交给脚本。命令描述意图，Python 负责解析、抓取、扫描，再把结构化结果交给 AI 综合。
4. 创建前先搜索。避免重复笔记，也不能凭记忆宣称某内容不存在。
5. 一切都要传播。每次写入都更新相关笔记、索引和操作日志。
6. 默认安全。后台 agent 可选且只增不删；`dist/` 重新生成；可刷新文档用 sentinel 标记保护人工编辑。
7. vault 会复利。写入越多，上下文越多，AI 后续越能成为有效搭档。

## README 核心场景翻译

对应原文: `README.md`

### 安装后会发生什么

- 会议后运行 `/obsidian-save`: Claude 提取决策、人物、任务、想法，并保存到正确笔记。
- 录音备忘运行 `/obsidian-ingest meeting.m4a`: Claude 转录、识别说话人、提取承诺和行动项，再分发到实体页、任务板、daily note。
- 白板照片运行 `/obsidian-ingest photo.png`: Claude 读图，提取文字和结构，创建概念笔记并链接项目。
- 好视频运行 `/obsidian-ingest https://youtube.com/...`: Claude 不只是总结成一页，而是重写已有页面，更新人物，解决矛盾，触发综合页。
- 重大决策前运行 `/obsidian-challenge`: Claude 在你的 vault 里找过往失败、反转决策和类似主题，用你的历史反驳你。
- 看全局运行 `/obsidian-visualize`: 生成 vault 的视觉 canvas，中心节点、类型颜色、孤岛笔记都会展示。
- 睡觉时: 夜间 agent 关闭当天、调和矛盾、综合跨来源模式、修复孤立笔记、重建索引。
- 新的一天运行 `/obsidian-daily`: 拉取日历事件、逾期任务和夜间变化，写入今日笔记。
- X 帖子运行 `/x-read`: 读取原帖、线程、回复、摘要、关键主张、回复情绪和值得关注的人。
- X 趋势运行 `/x-pulse`: 扫描一个主题当前正在流行什么，输出主题、空白点、hook 格式和今日可写的帖子。
- Web 研究运行 `/research`: 生成有引用的 dossier，包括摘要、关键事实、时间线、关键玩家、反方观点、开放问题。
- vault 优先深度研究运行 `/research-deep`: 先扫描 vault 已知内容，只补齐缺口，然后输出 delta 报告并传播更新。
- YouTube 运行 `/youtube`: 提取字幕和元数据，生成摘要、要点、引语、主题、评论情绪，并保存成 AI-first note。

### Before / After 翻译

| 没有它 | 有它 |
|---|---|
| 决策靠复制粘贴，常常丢失 | 自动保存到正确项目笔记 |
| daily note 靠自己写 | 自动创建和预填 |
| 找模式要重读很多笔记 | `/obsidian-emerge` 自动找 |
| 没人反驳自己 | `/obsidian-challenge` 用你的历史反驳你 |
| 每个会话都要重新解释背景 | `/obsidian-world` 载入完整上下文 |
| 读完资料就忘 | `/obsidian-ingest` 用一个来源重写多个 vault 页面 |
| 矛盾不知道存在 | `/obsidian-reconcile` 自动处理 |
| 综合靠手工 | `/obsidian-synthesize` 自动发现跨来源模式 |
| vault 只有 Claude 能读 | `/obsidian-export` 导出干净快照给任意 AI 工具 |
| 事实变化会覆盖旧信息 | 双时间事实记录“何时为真”和“何时学到” |
| X 线程靠截图 | `/x-read [url]` 返回帖文、线程、情绪和人物 |
| Web 研究开十几个 tab | `/research [topic]` 生成带来源的 dossier |
| 已知内容也从头研究 | `/research-deep` 先读 vault，只补缺口 |
| 视频看完就忘 | `/youtube [url]` 字幕、摘要和引用保存到 vault |
| 人类阅读优先 | AI-first 规则优先未来 AI 检索 |

## 已完成的中文译注文档

- [AI-first 笔记规则](./ai-first-rules.md): 对应 `references/ai-first-rules.md`。
- [写入规则](./write-rules.md): 对应 `references/write-rules.md`。
- [Vault 结构规范](./vault-schema.md): 对应 `references/vault-schema.md`。
- [命令索引](./commands-index.md): 对应 `commands/*.md` 的 frontmatter 和主要用途。
- [README 公开说明](./readme-public.md): 对应 `README.md` 的对外文档译注。
- [Skill 操作手册](./skill-manual.md): 对应 `SKILL.md`。
- [Fork 洞察](./fork-insights.md): 对应 `FORK_INSIGHTS.md`。
- [生态边界](./ecosystem.md): 对应 `ECOSYSTEM.md`。
- [`_CLAUDE.md` 模板](./claude-md-template.md): 对应 `references/claude-md-template.md`。
- [HXLoLi 可借鉴点](./hx-loli-borrow-list.md): 对当前博客项目的初步迁移建议。
- [HXLoLi 采纳方案](./hx-loli-adoption-plan.md): 基于当前代码结构的落地路线。

## 对 HXLoLi 的初步判断

`HXLoLi` 已经有三块很适合承接这套思想:

- `ai-docs/`: 天然可以作为“AI 生成但需人工 review 的知识沉淀区”。
- `static/docusaurus-graph.json` 和文档图谱: 已具备知识关联可视化基础。
- `scripts/`: 已有生成 sidebar、RSS、搜索索引、加密私有文章、文档关系图等自动化脚本，适合继续承接健康检查、索引和 AI 写入校验。

真正值得学习的不是“换成 Obsidian”，而是把博客仓库变成一个 AI-first 的、可审计的、可传播更新的知识库。
