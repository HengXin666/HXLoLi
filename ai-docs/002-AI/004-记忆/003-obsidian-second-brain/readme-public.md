---
title: README 公开说明
---

# README 公开说明

对应原文: `ref/obsidian-second-brain/README.md`

## 对外定位

`obsidian-second-brain` 的对外标语可以译为:

> 一个代码库，六个 CLI，同一个大脑。

它把自己描述为 Karpathy 的 LLM Wiki 模式的演进:

- 每个来源会更新已有页面，而不是只追加新页面。
- 矛盾会自动调和。
- vault 会在你睡觉时复利。
- 44 个命令、自动综合、会和你争论的 thinking tools。
- 从 X、Web、YouTube 拉取实时研究信号。
- 4 个 scheduled agents。
- 4 个角色 preset。
- 写入时 AI-first validator。
- `/create-command` 访谈式创建新命令。
- 多语言 trigger schema。

v0.10 的主题是 “The Architect”: 新命令 `/obsidian-architect` 扫描代码库，并把可维护架构笔记写入 vault。重跑可刷新，且不会覆盖人工编辑。

## Problem

用户每天使用 Claude，但每个会话都从零开始。你需要重复解释所有背景。对话结束后，所有东西消失。

用户也在 Obsidian 里做笔记。几百个文件放在那里。因为忘记六个月前做过的决策，可能重复决策。想法烂在 daily notes 里。没有人连接线索。

两个强大的工具完全断开。

## 如何扩展 Karpathy 的 LLM Wiki

| 维度 | Karpathy 的 LLM Wiki | obsidian-second-brain |
|---|---|---|
| 新来源 | 追加新页面并交叉引用 | 重写已有页面。人物更新、主张修订、陈旧事实替换 |
| 矛盾 | 标出后人工解决 | `/obsidian-reconcile` 自动解决 |
| 模式 | 用户询问时浮现 | `/obsidian-synthesize` 自己找未命名模式并写 synthesis pages |
| 运行时机 | 按需运行 | 4 个 scheduled agents: nightly close、weekly review、contradiction sweep、vault health |
| 笔记格式 | 人类可读 wiki pages | AI-first: `## For future Claude` 前言 + frontmatter，面向 LLM retrieval |

如果 Karpathy 的 wiki 是你用 LLM 维护的知识库，那么这个项目想做的是会维护自己的知识库。

## 安装后场景

### 会议后: `/obsidian-save`

Claude 提取每个决策、人物、任务、想法，并把它们保存到正确笔记。用户不用手动整理。

### 录音备忘: `/obsidian-ingest meeting.m4a`

Claude 用 Whisper 转录，识别说话人，抽取每个承诺和行动项，并分发到实体页、任务板和 daily note。

### 白板截图: `/obsidian-ingest photo.png`

Claude 读图，抽取文字和结构，创建概念笔记，链接到相关项目。照片变成知识。

### 好视频: `/obsidian-ingest https://youtube.com/...`

Claude 不会只总结成一条笔记，而是重写已有页面。人物更新，矛盾解决，模式触发 synthesis pages。一个 URL 进入，vault 更聪明。

### 重大决策前: `/obsidian-challenge`

Claude 搜索 vault 中同主题的过往失败和反转决策，用用户自己的话反驳用户，让 vault 负责问责。

### 看大图: `/obsidian-visualize`

生成整个 vault 的 visual canvas。hub nodes 居中，按类型着色，orphan 高亮。用 Obsidian 打开后可以看到知识形状。

### 睡觉时

nightly agent 跑 5 个 phase:

1. 关闭当天。
2. 调和矛盾。
3. 综合跨来源模式。
4. 修复孤立笔记。
5. 重建索引。

醒来时 vault 更聪明。

### 新一天: `/obsidian-daily`

Claude 把日历事件、逾期任务、夜间变化拉进今天笔记。

### X 帖子: `/x-read`

Grok 拉取帖子、线程、回复，返回原文、摘要、关键主张、回复情绪和值得关注的人。不再需要截图。

### 今日内容趋势: `/x-pulse`

Grok 扫描某主题在 X 上的实时趋势，返回 3 到 5 个新兴主题、代表帖子、关键声音、没人填的空白、有效 hook 格式，以及今天可写的 3 个具体帖子想法。

### Web 研究: `/research`

Perplexity Sonar Pro 生成带 citations 的深度 dossier: 摘要、关键事实、时间线、关键参与者、反方观点、推荐阅读、开放问题。保存到 vault 并自动在 Obsidian 打开。

无 key 时会使用免费来源，由 Claude 综合。

### vault-first 深度研究: `/research-deep`

先扫描 vault 已知内容，识别缺口。然后通过 Perplexity 和 Grok 发起 3 到 5 个目标搜索，合成 delta report:

- 什么是新的。
- 什么被确认。
- 有哪些矛盾要解决。
- 建议更新哪些 vault 页面。

不会重新研究 vault 已有基线，只补缺口。

### YouTube: `/youtube`

通过 `youtube-transcript-api` 获取免费字幕。可选用 YouTube Data API v3 获取元数据和热门评论。Grok 总结成 TL;DR、Key Points、Notable Quotes、Themes、Comment Sentiment、Worth Following Up On，保存为 AI-first note。

### 用户甚至不用打开 Obsidian

一切通过 Claude 完成。

## 分层系统

```text
LAYER 1: Operations
Claude remembers everything

LAYER 2: Thinking Tools
Claude thinks with you

LAYER 3: Context Engine
Claude knows who you are

LAYER 4: Research Toolkit
Claude pulls knowledge in

ALWAYS ON
Background agent + scheduled agents
Auto-synthesis + save reminders
```

总计 44 个命令。`/obsidian-calendar` 依赖 Google Calendar MCP，只在 Claude Code 中完整可用；Codex、Gemini、OpenCode、Hermes 构建提供 43 个跨平台命令。

各层含义:

- Layer 1: 保存、组织、摄入、调和、导出、日历调度、vault 维护。
- Layer 2: 反驳想法、浮现隐藏模式、桥接不相关领域、把想法升级成项目。
- Layer 3: 加载身份和当前状态，让每次会话从上一次结束处继续。
- Layer 4: 把实时外部知识引入 vault，包括 X、Web、YouTube、podcast。vault-first synthesis 知道你已经知道什么。
- Always On: 无需手动操作，让 vault 保持活跃。

## 44 个命令

### Operations

| Command | 作用 |
|---|---|
| `/obsidian-save` | 保存对话中的决策、任务、人物、想法 |
| `/obsidian-ingest` | 丢入 URL、PDF、音频、截图，vault 自我重写，每个来源触达 5-15 页 |
| `/obsidian-synthesize` | 自动发现跨来源模式并写 synthesis pages |
| `/obsidian-reconcile` | 找矛盾并解决，让 vault 维护自己的事实 |
| `/obsidian-export` | 导出任意 AI 工具可读的 JSON/Markdown snapshot |
| `/obsidian-daily` | 创建或更新今天的 daily note |
| `/obsidian-calendar <mode>` | agenda、reconcile、meeting、schedule 四模式日历命令 |
| `/obsidian-recurring` | 跟踪周期性义务和下一次 due date |
| `/obsidian-log` | 记录工作会话并链接到相关位置 |
| `/obsidian-task` | 加任务到正确 board，带优先级和 due date |
| `/obsidian-person` | 创建或更新人物笔记 |
| `/obsidian-capture` | 零阻力捕获想法 |
| `/obsidian-catchup` | 处理 Telegram bot 捕获的移动端内容 |
| `/obsidian-find` | 带上下文的智能搜索 |
| `/obsidian-recap` | 汇总一天、一周或一月 |
| `/obsidian-review` | 结构化周/月 review |
| `/obsidian-board` | Kanban board 查看和更新 |
| `/obsidian-board-hygiene` | 批量整理陈旧或逾期 board item |
| `/obsidian-project` | 项目笔记，并链接 board 和 daily |
| `/obsidian-projects` | 从 git 和本地文档实时汇总项目状态 |
| `/obsidian-health` | vault audit: 矛盾、缺口、陈旧主张、孤儿页 |
| `/obsidian-retrieval-eval` | 衡量 vault 搜索质量: recall@k、MRR、失败案例 |
| `/obsidian-decide [--formal]` | 记录决策；`--formal` 写完整 ADR |
| `/obsidian-visualize` | 生成第二大脑 visual canvas |
| `/obsidian-learn` | 回顾 learnings、清理陈旧项、把模式提升成规则 |
| `/obsidian-init` | 生成 `_CLAUDE.md`、`index.md`、`log.md` |
| `/obsidian-architect` | 扫描代码库并写架构笔记，可重跑刷新 |
| `/create-command` | 访谈式创建新命令，无需手写 markdown |

### Thinking

| Command | 作用 |
|---|---|
| `/obsidian-challenge` | vault 用你的历史反驳你的想法 |
| `/obsidian-panel` | 对决策召集多个视角，每个独立 verdict，再综合 |
| `/obsidian-emerge` | 从 30 天笔记里浮现你没命名的模式 |
| `/obsidian-connect [A] [B]` | 桥接两个不相关领域，激发新想法 |
| `/vault-deep-synthesis [topic]` | 交叉引用某主题的所有笔记: 一致点、矛盾、陈旧主张、缺口 |
| `/obsidian-distill [note or source]` | 把长来源压缩成关键主张，每条带 provenance |
| `/idea-discovery` | 从想法、开放问题、孤立研究中排名 3-5 个下一方向 |
| `/obsidian-graduate` | 把想法片段变成完整项目和任务 |

### Context

| Command | 作用 |
|---|---|
| `/obsidian-world` | 用 L0-L3 渐进 token budget 加载身份和状态 |

### Research

| Command | 作用 |
|---|---|
| `/x-read [url]` | 深读 X 帖子: 原文、线程、摘要、主张、回复情绪、声音 |
| `/x-pulse [topic]` | 扫描 X 趋势: 主题、声音、hook、post ideas |
| `/research [topic]` | Web research dossier；有 key 用 Perplexity，无 key 用免费来源 |
| `/research-deep [topic]` | vault-first open-web synthesis，并传播更新 |
| `/notebooklm [topic]` | Gemini File Search vault-grounded synthesis |
| `/youtube [url]` | 提取字幕、元数据、热门评论，生成 AI-first summary |
| `/podcast [url]` | Apple Podcasts 或 RSS 到转录和 AI-first summary |

## 研究工具设置

配置文件:

```text
~/.config/obsidian-second-brain/.env
```

把 `.env.example` 复制过去，添加 keys。也可以运行 `install.sh` 并在 research prompt 里回答 `y`。

没有 keys 时，`/research` 和 `/research-deep` 仍然可用。它们会自动退回免费无 key 来源:

- Wikipedia
- HackerNews
- arXiv
- Reddit
- Lobsters
- dev.to
- OpenAlex
- Semantic Scholar
- CrossRef
- DuckDuckGo

可用 `--free` 强制 free mode，用 `--academic` 限制到 scholarly sources。

其他研究命令仍需要对应 key。

## Thinking tools 示例

### `/obsidian-challenge`

用户说: “我想用 Rust 重写 API。”

Claude 找到 2025 年 Rust rewrite 失败的 post-mortem，又找到承诺未来 2 年使用 TypeScript 的 decision log，然后指出: “你的笔记说这失败过。还要继续吗?”

### `/obsidian-emerge`

Claude 扫描 30 条 daily notes。用户在 4 个无关项目里提到 onboarding friction。

结论: onboarding 是跨项目瓶颈，但用户从未命名它。

### `/obsidian-connect "distributed systems" "cooking"`

Claude 沿 link graph 追踪两个 cluster，找到共同概念: preparation 和 load distribution，然后生成 3 个可行动的交叉想法。

### `/obsidian-graduate`

Claude 读取三周前的一个 idea，找到相关项目和人物，生成完整 spec、goals、phases、tasks、board entries，并把原 idea 标记为 `graduated`。

## Research toolkit 示例

### `/x-read`

Grok 通过 live X access 获取帖子和回复。输出:

- 原文。
- TL;DR。
- key claims。
- reply sentiment。
- notable counter-arguments。
- voices to watch。

典型成本约 USD 0.05/call。

### `/x-pulse "AI automation"`

输出结构:

- WHAT'S HOT。
- WHAT'S UNDEREXPLORED。
- HOOKS THAT ARE WORKING。
- POST IDEAS FOR YOU TODAY。

把原本可能需要滚动 X 两小时的信息，在约 30 秒内返回。示例成本约 USD 0.13。

### `/research "AI memory tools"`

返回结构化 dossier:

- Summary。
- Key Facts，每条带 `(as of YYYY-MM, source.com)`。
- Timeline。
- Key Players。
- Contrarian Views。
- Recommended Further Reading。
- Open Questions。
- full citations。

保存到 `Research/Web/`。

### `/research-deep "AI memory tools"`

4 个 phase:

1. Vault scan: 找相关 notes，建立基线。
2. Gap analysis: 识别 vault 沉默或陈旧之处。
3. Targeted research: 针对缺口搜索 Web/X。
4. Synthesis: 输出 delta report。

vault-first 的关键是不会浪费 tokens 重新研究你已经知道的内容。

### `/notebooklm "AI-first vault rule"`

扫描 vault，上传最相关的 12 条笔记到 Gemini File Search store，让 Gemini 只基于这些来源综合并保存结果。之后删除 store。

适合与 `/research-deep` 配对: open-web view 和 vault-grounded view 不一致的地方，往往是值得写作的观点。

### `/youtube`

通过 `youtube-transcript-api` 免费取字幕，可选 YouTube Data API v3 取元数据和评论。Grok 总结。frontmatter 包含 view count、channel、published date、like count。

### `/podcast`

Apple Podcasts URL 或 RSS feed。转录来源优先级:

1. RSS 的 `<podcast:transcript>` tag。
2. 有 `OPENAI_API_KEY` 时用 Whisper API。
3. show-notes fallback。

Spotify 不支持，因为 DRM 阻止音频和转录访问。

## The Vault is Alive

传统 vault 是文件柜。你把东西放进去，然后它们待在那里。

这个 vault 会随每个输入自我重写:

- 摄入来源: 旧页面重写、矛盾解决、模式综合。
- 保存对话: 实体、概念、决策分布到整个 vault。
- 提问: Two-Output Rule 让每个答案也更新页面。
- 事实变化: 双时间事实记录何时为真、vault 何时学到。
- 什么都不做: background agent 和 scheduled agents 睡觉时维护。
- 等一周: auto-synthesis 找跨来源模式并写连接页。

一周后的 vault 和初始 vault 本质上不同。

## Presets

| Preset | 适用人群 | Kanban 风格 |
|---|---|---|
| `executive` | founders、operators、managers | OKRs / Quarterly / Weekly |
| `builder` | developers、engineers、architects | Backlog / Sprint / Done |
| `creator` | writers、YouTubers、marketers | Ideas / Drafts / Published |
| `researcher` | academics、analysts、deep-divers | Reading / Processing / Synthesized |

示例:

```bash
python bootstrap_vault.py --path ~/my-vault --name "Your Name" --preset builder
```

不选 preset 时得到通用 vault。

## Background 和 scheduled agents

Background agent 在每次 context compaction 后触发:

```text
PostCompact -> obsidian-bg-agent.sh -> claude -p (headless) -> vault updated
```

Scheduled agents:

| Agent | 时间 | 工作 |
|---|---|---|
| `morning` | 8 AM | daily note + overdue tasks |
| `nightly` | 10 PM | 睡眠期 consolidation: close day + reconcile + synthesize + heal orphans |
| `weekly` | Fridays 6 PM | weekly review |
| `health` | Sundays 9 PM | vault health audit |

Claude 会在 10 轮以上对话后，或用户说 done/thanks 时提醒 `/obsidian-save`，避免对话丢失。

## Vault architecture

默认 wiki-style / LLM-first:

```text
vault/
+-- _CLAUDE.md          # Operating manual
+-- index.md            # Page catalog, Claude reads first
+-- log.md              # Activity timeline
+-- SOUL.md             # Identity
+-- CRITICAL_FACTS.md   # about 120 tokens, always loaded
+-- raw/                # immutable source material
+-- wiki/               # Claude workspace
|   +-- entities/
|   +-- concepts/
|   +-- projects/
|   +-- daily/
|   +-- logs/
|   +-- reviews/
|   +-- tasks/
|   +-- decisions/
+-- boards/
+-- templates/
```

## Install

### Claude Code

一行安装:

```bash
curl -fsSL https://raw.githubusercontent.com/eugeniughelbur/obsidian-second-brain/main/scripts/quick-install.sh | bash
```

或者两条命令:

```bash
git clone https://github.com/eugeniughelbur/obsidian-second-brain ~/.claude/skills/obsidian-second-brain
bash ~/.claude/skills/obsidian-second-brain/scripts/setup.sh "/path/to/your/vault"
```

然后运行:

```text
/obsidian-init
```

### Codex CLI / Gemini CLI / OpenCode

```bash
git clone https://github.com/eugeniughelbur/obsidian-second-brain
cd obsidian-second-brain
bash scripts/build.sh --platform codex-cli
cp -R dist/codex-cli/. /path/to/your/vault/
```

然后从 vault 根目录启动对应 CLI。

Codex build 会生成 native Codex Agent Skills: 每个命令一个 skill，位于 `.agents/skills/<name>/SKILL.md`。Codex 通过 progressive disclosure 自动发现它们。`AGENTS.md` 只保留薄的 always-on 手册。

Gemini / OpenCode 仍生成 `GEMINI.md` / `AGENTS.md` dispatcher，并带自动路由表。

### Pi Coding Agent

```bash
git clone https://github.com/eugeniughelbur/obsidian-second-brain
cd obsidian-second-brain
bash scripts/build.sh --platform pi
pi install ./dist/pi
```

Pi build 生成 native Pi package: prompt templates 在 `.pi/prompts/`，discovery skill 在 `.pi/skills/obsidian-second-brain/`。Pi 没有 background-agent 等价物，可手动或 cron 运行 nightly。

### Hermes / open models

skill 是 model-agnostic。OpenCode、Codex、Gemini 构建都是普通 instruction files，所以可以跑在 host CLI 指向的任意模型上，包括 Hermes。

也有专门 Hermes Agent build:

```bash
bash scripts/build.sh --platform hermes
```

Open models 对 instruction-following 不如 Claude 稳定。核心命令如 `/obsidian-save`、`/obsidian-daily`、`/obsidian-capture`、`/obsidian-find`、`/obsidian-task` 和 free mode `/research` 通常可用。sub-agent-heavy 和 deep synthesis 命令更依赖强指令遵循，建议使用强模型。

## 可选 semantic search

默认搜索是快速关键词搜索，不需要模型或安装。可选开启 meaning-based layer，让没有共享词的查询也能找到笔记。开启后与关键词搜索融合。

推荐本地私有方案:

```bash
ollama pull mxbai-embed-large
uv run python scripts/eval/semantic_search.py --path "<vault>" --build
```

如果模型不可达，搜索静默回退到 keyword，不会坏掉或挂住。

## FAQ 译注

### Claude Code skill 是什么?

它是 Anthropic Claude Code CLI 的可复用行为包，包含 slash commands、scripts、references 和操作说明，让 Claude 自动加载领域能力。

### 这是 Obsidian 插件吗?

不是。Obsidian plugin 运行在 Obsidian 里并提供 UI 能力。Claude Code skill 运行在 Claude Code 中，让 Claude 从 Obsidian 外部读写和推理你的 vault。

### 和普通 Obsidian plugin 的差异?

Obsidian plugin 受 Obsidian API 限制。Claude Code skill 受 Claude 在 shell 中能做什么限制，所以可以做 live web research、定时 agents、跨多年笔记综合等插件难做的事情。

### 支持 Codex/Gemini/OpenCode 吗?

支持。构建脚本把平台中立来源编译成多个平台输出。vault 规则在所有平台一致。

### 支持 Hermes 或 open models 吗?

支持。核心命令在 open models 上可用，复杂综合命令建议用更强 instruction-follower。

### 支持 Obsidian Sync 吗?

支持。skill 写入的是标准 Markdown 文件。Obsidian Sync、iCloud、Syncthing、Git-based sync 都无需修改。

### 需要 API keys 吗?

大部分不需要。vault commands 不需要 key。`/research` 和 `/research-deep` 无 Perplexity key 时使用免费来源。其他研究命令需要对应 key。日历命令需要 Google Calendar MCP connector。

### 和 Notion AI / Mem 有何不同?

Notion AI 和 Mem 是闭源 SaaS，数据在厂商那里。这个 skill 把所有内容存为本地 Obsidian vault 的 Markdown，没有 vendor lock-in。

### 能文档化代码库吗?

能。`/obsidian-architect` 扫描软件项目，写 overview、模块笔记、key decisions。重跑只刷新 generated content，不覆盖人工编辑。

### AI-first vault rule 是什么?

笔记面向 future-Claude 检索和推理，而不是优先给人类阅读。它要求机器可读结构、每条主张带时效标记、强制 wikilinks、保留 source URLs、标注 confidence。

### 对现有 vault 安全吗?

安全。不会未经明确确认做破坏性删除或修改。已有笔记保持原状。新笔记遵守 AI-first。`/obsidian-health` 标记旧笔记的问题。

### `/research-deep` 比 `/research` 多什么?

`/research` 是单次 web research dossier。`/research-deep` 先扫描已有 notes，识别已知内容和缺口，只对缺口做 3-5 个 targeted searches，再输出 delta report 并建议 vault 更新。

### 研究命令成本

原 README 给出的 2026-04 近似成本:

- `/x-read`: 约 USD 0.05。
- `/x-pulse`: 约 USD 0.13。
- `/research`: 约 USD 0.04。
- `/research-deep`: 约 USD 0.40-0.80。
- `/youtube`: 约 USD 0.04。
- `/podcast`: 约 USD 0.04 Grok 调用，Whisper 另计约 USD 0.006/min。

Grok 调用记录到 `~/.research-toolkit/usage.log`。没有硬 cap，用户自行监控。

### Windows/Linux?

核心命令在 Claude Code 可运行处都可用。`install.sh` 支持 Linux、macOS、Windows MSYS2/Git Bash。Linux/macOS 使用 symlink，Windows 使用 copy 和 `update.sh` 刷新。

### 每个项目一个 vault?

支持。全局 `OBSIDIAN_VAULT_PATH` 可被项目目录 `.claude/settings.json` 覆盖。命令和 hooks 全局安装，只有 vault path 变化。不提供单 vault 内 `--scope` 隔离。

### 如何更新?

```bash
cd ~/.claude/skills/obsidian-second-brain && git pull
```

Linux/macOS 因 slash commands 是 symlink，不需要额外步骤。Windows 还要运行:

```bash
bash update.sh
```

然后重启 Claude Code。

## Philosophy

多数 second brain 工具让用户当清洁工。

这个 skill 反过来: 用户思考、工作、谈话，Claude 负责记忆。然后 Claude 使用这些记忆，让用户思考得更好: 浮现你错过的东西、挑战你的假设、连接你不会连接的内容、综合你没要求的模式。

vault 不是增长，而是进化。

> Your notes are the moat.

灵感来自 Andrey Karpathy 的 LLM-Wiki。

## Contributing

欢迎 PR:

- 新 thinking tools。
- 笔记类型 schema，例如 habits、books、investments。
- MCP integrations，例如 Calendar、Linear、Slack。
- 替代 vault structures。
- VS Code / Cursor setup guides。

领域特定 fork 见 `ECOSYSTEM.md`。upstream 提供 primitives，fork 拥有 domain knowledge。

自定义 fork 时，复制 `references/DELTAS.template.md` 到 fork 根目录的 `DELTAS.md`，记录本地差异。upstream 不碰这个文件，方便持续 merge `upstream/main`。

## License

MIT
