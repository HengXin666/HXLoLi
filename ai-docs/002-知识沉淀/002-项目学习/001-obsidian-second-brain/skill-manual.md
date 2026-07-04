---
title: Skill 操作手册
---

# Skill 操作手册

对应原文: `ref/obsidian-second-brain/SKILL.md`

## Skill 描述译文

`obsidian-second-brain` 用来把任意 Obsidian vault 当作一个活的、会自我重写的第二大脑来操作。它是 Karpathy 的 LLM Wiki 模式的演进版: 新来源会重写已有页面，矛盾会自动调和，定时 agents 会在你睡觉时维护 vault。

当用户要求 Claude 读取、写入、更新、搜索或管理 Obsidian vault 时，都应该使用这个 skill。包括:

- 从对话中保存笔记。
- 创建 daily entries。
- 更新 kanban boards。
- 记录开发工作。
- 管理人物笔记。
- 捕获决策。
- 跟踪交易。
- 维护任意 vault 结构。

当用户想从零启动新 vault、运行健康检查，或把 `_CLAUDE.md` 放进 vault 以便所有 Claude surface 使用同一套规则时，也会触发它。

它还包含研究工具包: `/x-read`、`/x-pulse`、`/research`、`/research-deep`、`/notebooklm`、`/youtube`、`/podcast`。这些命令通过 Grok、Perplexity、NotebookLM/Gemini、YouTube、podcast feeds 做 AI 研究，并按 AI-first vault rule 自动保存结果。

只要对话产生值得保留的信息，例如决策、遇到的人、开始的项目、完成的任务、学到的经验、研究发现，就应该主动使用。

## 总宣言

Claude 把你的 Obsidian vault 当作自我重写的知识库来操作。它继承 Karpathy 的 LLM Wiki 模式，但进一步要求: 来源不会只是追加成新页，而是会更新已有页面；矛盾会自动调和；定时 agents 会在你睡觉时维护 vault。

所有值得记住的都会被保存。每次更新都会传播到它该出现的地方。

## Quick Start

### 0. 选择 vault 访问方式

按优先级尝试以下方式，使用第一个可用方式。

#### Method 0 - SessionStart hook

如果 `hooks/load_vault_context.py` 已经作为 SessionStart hook 接入 `~/.claude/settings.json`，`_CLAUDE.md` 会在会话开始时自动注入上下文。此时跳过后面的 “第一次读 `_CLAUDE.md`”。

接入方式:

```bash
bash scripts/setup.sh "/path/to/vault"
```

或者运行 `/obsidian-setup`。

#### Method A - MCP server (`mcp-obsidian`)

如果 MCP tools 可用，例如:

- `get_file_contents`
- `list_files_in_vault`
- `search`
- `append_content`
- `write_file`

就使用它们。

#### Method B - 直接文件系统

这是 fallback，而且总是可用。对 vault 路径使用普通文件工具。vault 本质上是普通 Markdown，所以没有 MCP 也能工作，只是更啰嗦。

如果 MCP 没装，静默使用文件系统访问。第一次可以提示用户:

```text
For faster vault access on large vaults, consider installing mcp-obsidian:
claude mcp add obsidian-vault -s user -- npx -y mcp-obsidian "/path/to/your/vault".
Everything works without it.
```

### 1. 第一次进入 vault: 读取 `_CLAUDE.md`

做任何事前，检查 vault 根目录是否存在 `_CLAUDE.md`:

```text
get_file_contents("_CLAUDE.md")
```

如果存在，就严格遵守里面的规则。它覆盖 skill 默认规则。`_CLAUDE.md` 没说的地方才回退到 skill 默认值。

如果不存在，使用 skill 默认规则，并主动询问是否创建一个。

如果 SessionStart hook 已激活，`_CLAUDE.md` 已在上下文中，可以跳过这步。

### 2. 第一次面对新用户: 运行 discovery

```text
list_files_in_vault()
```

扫描结构以理解:

- 文件夹命名。
- 模板位置。
- 命名约定。
- frontmatter 模式。

然后读取 2 到 3 条已有笔记来校准写作风格，再创建新内容。

### 3. Bootstrap 新 vault

如果用户没有 vault:

```bash
curl -sL https://raw.githubusercontent.com/eugeniughelbur/obsidian-second-brain/main/scripts/quick-install.sh | bash
```

手动方式:

```bash
python scripts/bootstrap_vault.py --path ~/path/to/vault --name "Your Name"
```

带 preset:

```bash
python scripts/bootstrap_vault.py --path ~/my-vault --name "Your Name" --preset executive
python scripts/bootstrap_vault.py --path ~/my-vault --name "Your Name" --preset builder
python scripts/bootstrap_vault.py --path ~/my-vault --name "Your Name" --preset creator
python scripts/bootstrap_vault.py --path ~/my-vault --name "Your Name" --preset researcher
```

assistant mode:

```bash
python scripts/bootstrap_vault.py --path ~/my-vault --name "Your Name" --mode assistant --subject "Boss Name"
```

Preset 含义:

- `executive`: 决策、人物、会议、战略规划。Kanban: OKRs、Quarterly、Weekly。
- `builder`: 项目、dev logs、架构决策、调试。Kanban: Backlog、Sprint、Done。
- `creator`: 内容日历、想法流水线、受众笔记、发布。Kanban: Ideas、Drafts、Published。
- `researcher`: 来源、文献笔记、假设、方法论。Kanban: Reading、Processing、Synthesized。

不指定 preset 时，使用通用 vault。所有 preset 默认使用 wiki-style。

assistant mode 会创建一份为“代别人维护 vault”而配置的 `_CLAUDE.md`。

## 核心操作原则

### AI-first vault rule

vault 是给 future-Claude 阅读和推理的，不是给人类 review 的。Claude 写的每条笔记都必须遵守 `references/ai-first-rules.md`:

1. 自包含上下文。
2. `For future Claude` 前言。
3. 丰富且一致的 frontmatter。
4. 每条主张带时效标记。
5. 原样保留来源。
6. 强制交叉链接。
7. 标注置信度。

### 永远不要孤立创建

每次写入都要问: “这还属于哪里?”

| 创建/更新 | 也要更新 |
|---|---|
| 新项目笔记 | Kanban board、今天的 daily note |
| 已完成任务 | Kanban Done、项目笔记、daily note |
| 人物笔记 | daily note、People index |
| dev log | daily note、项目笔记 Recent Activity |
| deal update | Deals kanban、Dashboard totals |
| 决策 | 项目笔记 Key Decisions、daily note |
| mention/shoutout | Mentions Log、人物笔记、daily note |
| hook、反方角度、内容想法 | `social-media/ideas.md` |
| 可复用数字或统计 | `social-media/data-points.md` |
| 表现好的外部帖子 | `social-media/swipe-file.md` |
| 值得保留的研究发现 | `social-media/research/YYYY-MM-DD — topic.md` |
| 任意 vault 写入 | 操作日志、`index.md` |

始终传播。不要创建孤儿笔记。

### 双时间事实

当事实变化，例如角色、公司、状态、位置、工具，不要删除旧值。给 frontmatter 的 `timeline:` 追加新条目，同时记录 event time 和 transaction time。

```yaml
timeline:
  - fact: "CTO at Single Grain"
    from: 2024-01-01
    until: 2026-04-07
    learned: 2026-02-23
    source: "[[2026-02-23]]"
  - fact: "Architect at Single Grain"
    from: 2026-04-07
    until: present
    learned: 2026-04-07
    source: "[[2026-04-07]]"
```

顶层字段 `role:`、`status:`、`company:` 反映当前状态。`timeline:` 保留完整历史和来源。

### `CRITICAL_FACTS.md`

一个约 120 tokens 的小文件，与 `SOUL.md` 一起在每次会话 L0 加载。只包含每次对话都需要知道的当前事实:

- 时区。
- 当前 manager。
- 当前地点。
- 当前公司和角色。
- 其他现在为真且每次交互都相关的事实。

关键事实变化时更新它，并保持在 150 tokens 以下。

### Raw 不可变

wiki-style vault 中，`raw/` 存原始来源，例如文章、转录、PDF。Claude 读取但不修改。它是真相源。如果 wiki 页面损坏，从 raw 重新推导。摄入时，原文进 `raw/`，派生页进 `wiki/`。

### 维护 `index.md` 和 `log.md`

两个结构文件让 vault 可导航、可审计。

- `index.md`: 全 vault 页面目录，按类别组织。Claude 导航时先读它，而不是直接搜索。新建或删除笔记时更新。
- `log.md`: 追加式操作日志。每次保存、摄入、健康检查、结构变化都追加时间戳条目。不能删除或重写旧条目。

现代 vault 可以使用分日日志:

- `Logs/YYYY-MM-DD.md`: 每天一个 append-only 文件。
- 根目录 `log.md`: 只作为指针文件，不写操作条目。

### vault 是活系统

vault 不是文件柜，而是随输入自我重写的活知识库。新信息进入时:

- 更新已有页面，不只是追加。
- 解决或记录新旧主张矛盾。
- 多来源模式会触发 synthesis 页面。
- 陈旧主张会被当前信息替换，但历史保留。

摄入后 vault 应该不同，而不只是更大。

### Two-Output Rule

每次产生洞见的交互都必须生成两个输出:

1. 用户在对话中看到的答案。
2. 写回相关笔记的 vault update。

所有 thinking tools，以及 Claude 从 vault 中综合信息回答的查询，都遵守这个规则。

### Synthesis Hook

Claude 在任何操作中注意到模式时，应自动在 `wiki/concepts/` 创建 synthesis 页面。模式包括:

- 同一概念出现在 3 个以上无关来源。
- 某主张被多个独立来源增强。
- 时间序列笔记中出现趋势。
- 两个实体共享意外连接。

synthesis 页面是 vault 自己在思考。

### Reconciliation

vault 不应有两个互相矛盾但彼此不知道的页面。发现矛盾时:

- 能解决: 重写过时页面并保留历史。
- 不能解决: 创建显式 conflict page，标为 open question。

vault 级事实维护使用 `/obsidian-reconcile`。

### 主动保存提醒

未保存的对话是丢失知识。Claude 应主动提醒:

- 10 轮以上对话后。
- 用户表示结束时，例如 ok、thanks、done、bye。
- 一个逻辑工作块完成时。

Claude Desktop 没有后台 agent 时尤其重要。

### 创建前搜索

新建任何笔记前:

```text
search(query="keyword from title")
```

重复笔记是 vault rot。找到已有笔记就合并或更新。

### 不要宣称缺失，不要编造

两类失败会静默腐蚀 vault:

- False absence: 不经穷尽搜索就说某笔记不存在。
- Fabrication: 发明未被陈述的事实、实体、比例、日期或关系。

未知标 `TBD`。外部主张带 URL 和时效标记。推断带置信度。

### 匹配 vault 声音

写新笔记前读同文件夹的已有笔记。匹配 frontmatter、标题风格、列表格式、语气、emoji 使用。不要引入新惯例，只延展已有惯例。

### Frontmatter 必须有

每条笔记至少:

```yaml
---
date: 2026-03-24
tags:
  - <note-type>
---
```

完整 schema 见 `references/vault-schema.md`。

## `_CLAUDE.md`

这是整个 skill 最重要的概念。

`_CLAUDE.md` 位于 vault 根目录，持久化 Claude 的操作规则，覆盖 Desktop、Code、VS Code、terminal 等所有 Claude surface。没有它，Claude 每次对话都要重新学习 vault 惯例。

优先级规则: `_CLAUDE.md` 在所有 vault 特定规则上胜出。skill 默认值只在 `_CLAUDE.md` 沉默时适用。不要让 skill 默认规则覆盖显式 `_CLAUDE.md` 规则。

它包含:

- vault 文件夹地图。
- 特定笔记类型的 frontmatter schema。
- 命名约定。
- 哪些内容自动保存，哪些先询问。
- 需要特殊处理的人和项目。
- 关键文件链接，例如 boards、dashboard、templates。

## 常见操作

### 从对话保存信息

1. 识别笔记类型。
2. 检查相关笔记是否已存在。
3. 写入或更新，frontmatter-first。
4. 传播到 boards、daily note、linked notes。

### 创建今天的 daily note

```text
date = today in YYYY-MM-DD format
path = Daily/{date}.md
```

读取 `Templates/Daily Note.md`，填日期字段，创建文件。然后扫描最近对话，把值得记录的东西写进今日章节。

### 记录 dev session

读取 `Templates/Dev Log.md`，填入日期、项目、做了什么、解决什么问题、做出什么决策、下一步。保存到 `Dev Logs/YYYY-MM-DD — Project Name.md`，再链接到项目笔记和今日 daily note。

### 更新 kanban board

board 使用 `kanban-plugin: board` frontmatter。列是 `## Column Name`。事项格式:

```markdown
- [ ] **Title** · @{due-date}
	Description [[Links]]
```

完成项移动到 `## ✅ Done`，并加删除线:

```markdown
- [x] ~~**Title**~~ ✅ Date
```

### vault health check

```bash
python scripts/vault_health.py --path ~/path/to/vault
```

报告重复笔记、孤儿文件、过期任务、空文件夹、断链、缺少 frontmatter 的笔记。

## 命令执行要点

所有 slash commands 都可以在 Claude surface 中使用。每个命令都应读上下文、创建前搜索，并把变化传播到相关位置。

名字匹配规则: 如果名称参数有错别字或近似，搜索 vault 中最接近匹配，展示找到的内容并让用户确认。不要静默创建带错拼名字的新笔记。

详细命令语义见 [命令索引](./commands-index.md)。

## Context Engine: `/obsidian-world`

`/obsidian-world` 一次加载身份、价值观、优先级和当前状态，但按上下文层级控制 token。

- L0 Identity: 约 200 tokens，读取 `SOUL.md` / `About Me.md` 和 values。
- L1 Navigation: 约 1-2K tokens，读取 `index.md` 和最近 `log.md`。
- L2 Current State: 约 2-5K tokens，读取 dashboard、今天和最近 3 天 daily note、活跃 boards、上一会话摘要。
- L3 Deep Context: 只在需要时加载，可能 5-20K tokens。

输出应简洁，只是 boot-up sequence，不是完整报告。

## Research Commands

研究命令把外部知识引入 vault，包括 X 帖子、X 舆论、Web 研究、vault-grounded synthesis、YouTube、podcast。所有输出都按 AI-first note 写入。

配置:

- API keys 放在 `~/.config/obsidian-second-brain/.env`。
- Python 3.10+ 和 `uv`。
- `uv sync` 安装依赖。

成本追踪:

- `/x-read`、`/x-pulse`、`/youtube`、`/podcast` 的 Grok 调用写入 `~/.research-toolkit/usage.log`。
- Perplexity 不做使用追踪，这是有意设计。
- 没有硬性 cap，不做每次确认，信任用户自己监控。

## Scheduled Agents

原项目定义 4 个定时 agent 模式。它们保守默认，不会自主删除或归档，也不会在运行中问用户问题。

### `obsidian-morning`

每天早上创建当天 daily note，拉入今天到期或逾期任务，并列出 7 天无活动的 active projects。

### `obsidian-nightly`

每天晚上做睡眠期 consolidation:

1. 关闭当天: daily note 写 End of Day，总结 3-5 条，移动完成任务。
2. 调和: 扫描实体和概念中与新笔记冲突的旧信息。
3. 综合: 扫描今天和昨天摄入的来源，找跨来源概念。
4. 修复: 找今天创建但无 incoming link 的笔记，补链接；重建 `index.md`。
5. 记录: 写 `log.md`。

只添加、更新、链接，不做破坏性操作。

### `obsidian-weekly`

每周五生成 weekly review note，保存到 `Reviews/YYYY-MM-DD — Weekly Review.md`，并从本周最后一个 daily note 链接。

### `obsidian-health-check`

每周日运行 vault health check，写报告，不自动修复。

## Headless gotcha

非交互模式 `claude -p` 不会展开自定义 slash commands。cron 或 launchd 里运行:

```bash
claude -p "/obsidian-daily"
```

会把 `/obsidian-daily` 当普通提示词发送，命令文件不会加载。

可靠模式是让 Claude 读取命令文件并执行:

```bash
cd "$VAULT" && claude --dangerously-skip-permissions \
  -p "Read ~/.claude/commands/obsidian-daily.md and carry out its instructions exactly."
```

launchd 中要显式设置 `PATH`。

## Background Agent

PostCompact hook 会在 Claude 压缩上下文后自动触发后台 agent。它读取 session summary，把值得保存的内容传播到 vault。

流程:

1. Claude Code 上下文压缩后触发 `PostCompact` hook。
2. hook 从 stdin 读取 JSON summary。
3. 在 vault 目录启动 headless `claude --dangerously-skip-permissions -p`。
4. agent 静默传播更新并退出。

安全性: 不删除、不归档、不合并。只添加或更新。summary 没有值得保存的内容时，不动 vault。

## Write-Time AI-First Validator

PostToolUse hook 在每次对配置 vault 内 Markdown 的 `Write` 或 `Edit` 后触发，非阻塞地检查 AI-first 规则。

检查:

1. 是否有 frontmatter delimiter。
2. frontmatter 中是否没有 tab。
3. 是否有 `date:`、`type:`、`tags:`、`ai-first: true`。
4. 正文是否有 `## For future Claude` 前言。

跳过:

- `OBSIDIAN_VAULT_PATH` 外文件。
- `raw/`、`templates/`、`_export/`、`.obsidian/`、`.git/`、`.trash/`。

失败时，Claude 会在 stderr 看到 warning，并可在同一轮修复。原写入不会回滚。

## Per-Project Vaults

默认安装把 `OBSIDIAN_VAULT_PATH` 写到全局 `~/.claude/settings.json`，假定一台机器一个 vault。

多 repo 工作流可以在项目目录下创建 `.claude/settings.json` 覆盖:

```json
{
  "env": {
    "OBSIDIAN_VAULT_PATH": "/Users/you/vaults/repo-a-vault"
  }
}
```

Claude Code 会把项目级设置合并在全局设置之上。因此从不同目录启动会话时，hooks 和命令会操作不同 vault。

注意: 这不提供单个 vault 内的 scope 隔离。`/obsidian-find`、`/obsidian-recap`、`/obsidian-emerge` 会扫描整个配置 vault。

## 参考文件和脚本

参考文件:

- `references/vault-schema.md`: 完整文件夹结构和 frontmatter specs。
- `references/write-rules.md`: 写作、链接、格式化规则。
- `references/claude-md-template.md`: 生成 `_CLAUDE.md` 的模板。

脚本:

- `scripts/setup.sh`: 一键安装器。
- `scripts/bootstrap_vault.py`: 从零 bootstrap vault。
- `scripts/vault_health.py`: 审计 vault 结构问题。
