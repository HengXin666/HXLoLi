---
title: 命令索引
---

# 命令索引

对应原文: `ref/obsidian-second-brain/commands/*.md`

> [!TIP]
> 命令名不翻译，`category` 和 `triggers_en` 保留原样。中文说明是语义译注，便于后续迁移到 `HXLoLi` 的 AI 工作流。

## 分类总览

| 分类 | 含义 | 命令数 |
|---|---|---:|
| `vault` | 日常写入、捕获、查找、任务板、项目状态等 vault 管理 | 16 |
| `thinking` | 从历史笔记中生成洞见、反驳、综合、决策、学习 | 13 |
| `research` | 外部资料研究和摄入，结果按 AI-first 规则保存 | 8 |
| `meta` | 初始化、健康检查、导出、可视化、架构文档、扩展系统 | 7 |

## Vault 命令

| 命令 | 中文语义 | 触发语示例 |
|---|---|---|
| `/obsidian-save` | 把当前对话中值得保留的内容保存到 vault，包括决策、任务、人物、项目、想法、经验等 | `save this`, `save the conversation` |
| `/obsidian-daily` | 创建或更新今天的 daily note，拉取日历、逾期任务和对话上下文 | `todays note`, `open daily` |
| `/obsidian-log` | 把当前工作或开发会话写入 vault，并从上下文推断项目 | `log this work`, `log this session` |
| `/obsidian-task` | 把任务加入合适的 kanban board，推断优先级和截止日期 | `add task`, `new todo` |
| `/obsidian-person` | 从对话上下文创建或更新人物笔记 | `save this person`, `add person` |
| `/obsidian-capture` | 零阻力快速捕获想法，保存到 ideas 文件夹并写入 daily note | `capture this idea`, `quick note` |
| `/obsidian-catchup` | 处理 Telegram journal bot 在移动端捕获的语音、文本、图片、PDF、链接队列 | `catch up`, `process my captures` |
| `/obsidian-find` | 智能搜索 vault，返回带上下文的结果，而不只是文件名 | `find in vault`, `search my notes` |
| `/obsidian-recap` | 按今天、本周、本月从 vault 总结一段时间 | `recap today`, `summarize the week` |
| `/obsidian-board` | 显示或更新 kanban board，标出逾期项并按对话更新 | `show board`, `kanban` |
| `/obsidian-board-hygiene` | 批量清理 kanban board，找出陈旧事项并归档、改期或标记完成 | `clean up my board`, `board hygiene` |
| `/obsidian-project` | 创建或更新项目笔记，并自动加入 board 和 daily note | `new project`, `project setup` |
| `/obsidian-projects` | 从 git 和本地文档实时汇总项目状态，不需要额外配置 | `projects overview`, `what am I working on` |
| `/obsidian-recurring` | 跟踪周期性义务，例如付款、申报、运维，计算下一次 due date | `recurring task`, `monthly obligation` |
| `/obsidian-world` | 一次性加载身份、价值观、优先级和当前状态，用分级上下文节省 tokens | `load context`, `where am I` |
| `/obsidian-calendar` | 一个日历命令含四种模式: agenda、reconcile、meeting、schedule | `review my agenda`, `schedule this task` |

## Thinking 命令

| 命令 | 中文语义 | 触发语示例 |
|---|---|---|
| `/obsidian-challenge` | 用自己的 vault 历史红队当前想法，找矛盾、旧失败和有问题的假设 | `challenge this`, `red team my idea` |
| `/obsidian-emerge` | 从最近笔记中浮现未命名模式、重复主题、隐藏连接和未明说结论 | `find patterns`, `what is emerging` |
| `/obsidian-connect` | 用 vault link graph 连接两个不相关领域，制造创造性摩擦 | `connect domains`, `bridge ideas` |
| `/obsidian-graduate` | 把想法片段升级成完整项目规格、任务、board 项和结构 | `graduate this to project` |
| `/obsidian-decide` | 记录决策。默认轻量写进项目笔记，也可用 `--formal` 写完整 ADR | `log this decision`, `ADR` |
| `/obsidian-distill` | 把长笔记或来源压缩成关键主张，每条主张都带回原始来源块的 provenance | `distill this`, `summarize with sources` |
| `/obsidian-reconcile` | 查找和解决 vault 中的矛盾，让 vault 维护自己的事实一致性 | `find contradictions`, `reconcile vault` |
| `/obsidian-review` | 从 vault 历史生成结构化周/月回顾笔记 | `weekly review`, `monthly review` |
| `/obsidian-synthesize` | 自动扫描 vault 中未命名模式并写综合页，不需要用户明确指定主题 | `synthesize`, `auto-synthesis` |
| `/obsidian-learn` | 回顾 vault 中的 learnings，清理陈旧项，浮现仍活跃的模式 | `review learnings`, `what have I learned` |
| `/obsidian-panel` | 召集多个不同视角对决策给出独立判断，再综合 | `advisor panel`, `panel review` |
| `/idea-discovery` | 读取未升级想法、项目开放问题、孤立研究笔记，提出 3 到 5 个下一方向候选 | `what should I work on next` |
| `/vault-deep-synthesis` | 围绕一个主题深度交叉引用 vault 已知内容，列出一致点、矛盾、陈旧主张和缺口 | `what does my vault say about` |

## Research 命令

| 命令 | 中文语义 | 触发语示例 |
|---|---|---|
| `/research` | 带引用的 Web 研究。有 Perplexity key 时用 Sonar，否则用 Wikipedia、HN、arXiv、Reddit 等免费来源 | `research this`, `look up` |
| `/research-deep` | vault-first 深度研究。先扫描 vault，再补缺口，合成 delta，并通过 `/obsidian-save` 传播更新 | `deep research`, `research gaps` |
| `/notebooklm` | 基于 Gemini File Search 的 vault-first source-grounded research，是 `/research-deep` 的 grounded 并行方案 | `ask my notebook`, `source-grounded research` |
| `/x-read` | 用 Grok + Live Search 深读 X/Twitter 帖子，返回原文、线程、摘要、主张、回复情绪、值得关注的人 | `read this x post`, `analyze this tweet` |
| `/x-pulse` | 扫描 X 上某主题的趋势，输出主题、声音、hook 和今日发帖想法 | `x pulse`, `scan x for` |
| `/youtube` | 提取 YouTube 字幕、元数据和热门评论，经 Grok 总结并保存到 vault | `summarize youtube`, `youtube transcript` |
| `/podcast` | 提取播客单集元数据、转录和摘要，保存为 AI-first note | `summarize this podcast` |
| `/obsidian-ingest` | 摄入任意来源，让 vault 围绕新知识自我重写: 更新实体、替换陈旧主张、综合新概念、解决矛盾 | `ingest this source`, `absorb this` |

## Meta 命令

| 命令 | 中文语义 | 触发语示例 |
|---|---|---|
| `/obsidian-init` | 扫描 vault 并生成 `_CLAUDE.md` 操作手册、`index.md` 目录和 `log.md` 指针 | `init vault`, `bootstrap vault` |
| `/obsidian-health` | 运行 vault 健康检查，按严重程度分组，检测矛盾、概念缺口、陈旧主张、结构问题 | `vault health`, `audit vault` |
| `/obsidian-export` | 导出 agent 或工具可消费的干净结构化 vault 快照，例如 JSON、Markdown index、OKF bundle | `export vault`, `snapshot vault` |
| `/obsidian-visualize` | 生成 vault 的视觉 canvas map，看第二大脑的形状和知识连接 | `visualize vault`, `vault map` |
| `/obsidian-retrieval-eval` | 评估 vault 搜索对自然语言问题是否能找对笔记，输出 recall@k、MRR 和失败案例 | `evaluate retrieval` |
| `/obsidian-architect` | 扫描代码库并写入可维护架构笔记: overview、模块笔记、关键决策；重跑时只刷新生成区 | `document this codebase` |
| `/create-command` | 通过访谈创建新的 obsidian-second-brain 命令，用户无需手写 markdown 或 frontmatter | `create command`, `new command` |

## 关键命令工作流翻译

### `/obsidian-save`

主保存命令。读取整个对话并抽取值得保存的内容。

流程:

1. 扫描对话，识别所有值得进 vault 的项目: 决策、任务、人物、项目、想法、learnings、交易、提及。
2. 按类型分组: people、projects、tasks、decisions、ideas、deals。
3. 为每组并行处理，分别搜索、创建或更新相关笔记。
4. 全部完成后，更新今天的 daily note，并链接所有保存内容。
5. 回报清晰列表: 保存了什么、保存到哪里。

规则: 不要要求用户指导保存到哪里，要推断。只有真正歧义时才问。

### `/obsidian-ingest`

摄入来源，不是把来源总结成单独一页，而是让 vault 围绕新知识重写自己。

核心动作:

- 保存原始来源到 `raw/`。
- 找到相关人物、项目、概念、任务、决策页。
- 更新已有页面，而不是只追加新页面。
- 替换陈旧主张，同时保留历史和来源。
- 发现矛盾时解决或显式记录。
- 发现跨来源模式时创建 synthesis 页面。

评价标准: 摄入后 vault 应该变得不同，而不只是变大。旧页面如果没有更聪明、更互联、更当前，摄入就不够深。

### `/obsidian-architect`

把软件项目转换成可维护的架构笔记，方便未来的自己和未来的 Claude 回答“这个项目如何工作以及为什么这样设计”。

流程:

1. 解析代码库路径。
2. 运行 `python scripts/architect_scan.py --path <codebase>`，获得语言、模块、依赖、入口点、CI/Docker 信号、git commit 等 JSON。
3. 可选运行 `python scripts/mine_commit_decisions.py --repo <codebase> --json` 提取决策历史。
4. 写入项目 hub 下的 `Architecture/`。
5. 生成:
   - `Architecture - Overview.md`: 项目是什么、技术栈、模块关系、Mermaid 图、persona。
   - 每个核心模块一条 `Architecture - <Module>.md`。
   - `Architecture - Key decisions.md`。
6. 用 sentinel 标记保护人工编辑，只替换 `@generated` 区域。
7. 重跑时更新改变的生成块，并记录 `scanned-commit`。

### `/obsidian-health`

健康检查命令。运行脚本后，把问题按严重程度分组。

检查维度:

- 断链。
- 重复笔记。
- 缺失 frontmatter。
- 逾期任务。
- 未填模板语法。
- 孤立笔记和空文件夹。
- 决策或知识中的矛盾。
- 被提到 3 次以上但没有专页的概念缺口。
- 快速变化主题上的陈旧主张。

安全修复可以自动提供；有破坏性的修复，例如归档、合并、解决矛盾，需要显式确认。

### `/obsidian-retrieval-eval`

评估 vault 搜索质量。它复用真实搜索引擎，而不是模拟。流程是从 vault 采样笔记，让 LLM 为每条笔记写一个不使用标题词的问题，再看搜索能否找回正确笔记。

指标:

- recall@1 / recall@3 / recall@5 / recall@10
- MRR
- 具体失败案例: 错过了什么、哪个错误笔记排第一

价值: 搜索优化要可测量，而不是凭感觉。

## 可迁移到 HXLoLi 的命令优先级

如果先做最小迁移，不需要一次性复制 44 个命令。对 `HXLoLi` 最有价值的优先级是:

1. `hx-save`: 保存一次 AI 对话/调试/设计讨论到 `ai-docs/`。
2. `hx-architect`: 扫描 `HXLoLi` 代码库，写入可刷新的架构笔记。
3. `hx-health`: 检查 docs/blog/ai-docs 的断链、孤儿页、frontmatter 缺失、标题重复。
4. `hx-ingest`: 摄入一篇资料或博客草稿，并更新已有知识页。
5. `hx-retrieval-eval`: 衡量本地搜索和知识图谱对自然语言问题的召回质量。
