---
title: AI-first 笔记规则
---

# AI-first 笔记规则

对应原文: `ref/obsidian-second-brain/references/ai-first-rules.md`

## 总原则

vault 是为未来的 Claude 读取和推理而设计，不是优先给人类逐页阅读。用户很少直接打开笔记，而是调用 Claude 在多年积累的知识中检索、综合和连接线索。因此，所有会写入 vault 的命令都必须生成符合这些规则的笔记。

本文是 canonical specification。原项目中它位于 `references/ai-first-rules.md`，并被 `_CLAUDE.md` 第 0 节、每个 slash command、`references/write-rules.md` 引用。

## 7 条规则

### 1. 自包含上下文

每条笔记必须能解释自己。未来的 Claude 可能只通过 `/obsidian-find` 或 vault scan 拉到这一条笔记，没有任何周边上下文。不能只依赖反向链接来理解含义。笔记内部必须说明是什么、为什么、什么时候。

### 2. `For future Claude` 前言

每条笔记都要在 frontmatter 后面立即用 `## For future Claude` 开头，并写 2 到 3 句英文摘要。未来的 Claude 会先读它，并在 10 秒内判断这条笔记是否相关。摘要要说明笔记里有什么、为什么保存、是否有时间/陈旧性限制。

```markdown
## For future Claude
This note is a [type] about [topic] saved on [date]. It [main purpose].
[Optional caveat about staleness, confidence, or scope.]
```

### 3. 丰富且一致的 frontmatter

frontmatter 是可过滤的机器元数据。不同笔记类型有不同 schema，但所有笔记都要有机器可读 frontmatter。

通用字段:

```yaml
---
date: YYYY-MM-DD
type: <note-type>
tags: [...]
ai-first: true
---
```

说明:

- `date`: 创建或更新日期。
- `type`: 笔记类型，必须能被机器判断。
- `tags`: 必须包含类型本身。
- `ai-first: true`: 明确标记这条笔记符合 AI-first 标准。

### 4. 每条外部主张都带时效标记

陈述外部事实时，要在行内附上日期:

```markdown
- Mem0 raised \$24M Series A (as of 2026-04, mem0.ai/blog/series-a)
- Anthropic released native memory tool (as of 2026-02, anthropic.com/news/memory)
```

这样未来的 Claude 才知道在信任单个事实前需要验证什么。

### 5. 原样保留来源

每个外部主张都必须在行内保留来源 URL。不要把引用改写成抽象描述。保留实际 URL，方便多年后重新验证或刷新。

### 6. 强制交叉链接

每个被引用的人、项目、想法、决策或概念都要使用 `[[wikilinks]]`，让图谱可被未来 Claude 遍历。

```markdown
Sarah at [[People/Sarah Chen]] decided to ship the [[Projects/Dashboard Refactor]] by Friday.
```

如果被链接的笔记不存在，按 `references/write-rules.md` 的 Stub Notes 规则创建 stub。

### 7. 置信度等级

适用时标注置信度:

- `stated`: 由来源直接引用或直接声称。
- `high`: 多个来源一致。
- `medium`: 单一来源且合理。
- `speculation`: 推断。

可以在 frontmatter 中写 `confidence: high`，也可以行内写 `(confidence: speculation)`。

## 反幻觉和搜索完备性

上面 7 条规定怎么写笔记。下面这些规则规定 Claude 写入前怎么读、怎么推理。它们不可协商，因为这些失败模式会静默破坏 vault 的记忆价值。

### 错误地宣称不存在

不要在没有穷尽搜索前断言某条笔记、人物、项目或文件不存在。说“没有这条笔记”但其实有，是最常见的失败模式，甚至比编造更常见。

必须通过列出和 grep vault 来验证存在或不存在，不能依赖记忆或单次幸运查询。结论缺失前，要按所有可能名称、别名和文件夹搜索。拿不准时，宁可多包含并标注不确定，也不要少报。

### 搜索完备性

命令读取或扫描 vault 时，必须穷举，不要采样。要列出每条匹配笔记，不是列几个代表。把部分扫描报告成完整扫描，会制造自信但错误的答案，这比诚实说明“我只检查了 X”更糟。

### 不编造

不要发明未被实际陈述的事实、实体、比例、日期或关系。未知就标记为 `TBD`。每个外部主张都带时效标记和来源 URL；推断要带置信度。不要为了让章节看起来完整而伪造内容。没有决策时，一个空的 `## Decisions` 章节是正确的。

## 类型 Schema

所有类型都要保留通用字段，类型特定字段只能增加，不能删掉通用字段。

### `type: daily`

```yaml
date: YYYY-MM-DD
type: daily
tags: [daily]
mood: ""
energy: ""
ai-first: true
```

### `type: project`

```yaml
date: YYYY-MM-DD
updated: YYYY-MM-DD
type: project
status: active
tags: [project, ...]
related-people: ["[[People/...]]", ...]
related-projects: ["[[Projects/...]]", ...]
ai-first: true
```

`status` 可用值: `active | planning | completed | archived | on-hold`。

### `type: person`

```yaml
date: YYYY-MM-DD
updated: YYYY-MM-DD
type: person
tags: [person, ...]
role: ""
company: "[[Companies/...]]"
relationship: weak | medium | strong
last-interaction: YYYY-MM-DD
related-projects: ["[[Projects/...]]", ...]
ai-first: true
```

### `type: idea`

```yaml
date: YYYY-MM-DD
type: idea
tags: [idea, ...]
status: captured
related-projects: ["[[Projects/...]]", ...]
ai-first: true
```

`status` 可用值: `captured | exploring | graduated | shelved`。

### `type: task`

```yaml
date: YYYY-MM-DD
type: task
status: in-progress
priority: 🔴 | 🟡 | 🟢
due: YYYY-MM-DD
tags: [task, ...]
related-projects: ["[[Projects/...]]", ...]
related-people: ["[[People/...]]", ...]
ai-first: true
```

`status` 可用值: `in-progress | done | waiting | cancelled`。

### `type: decision`

决策通常写在项目笔记的 Key Decisions 章节内。确实需要独立决策笔记时:

```yaml
date: YYYY-MM-DD
type: decision
tags: [decision, ...]
related-projects: ["[[Projects/...]]", ...]
confidence: stated | high | medium | speculation
sources: [...]
ai-first: true
```

### `type: devlog` / `type: log`

```yaml
date: YYYY-MM-DD
type: devlog
tags: [devlog, ...]
project: "[[Projects/...]]"
related-people: ["[[People/...]]", ...]
ai-first: true
```

### `type: review`

```yaml
date: YYYY-MM-DD
period-start: YYYY-MM-DD
period-end: YYYY-MM-DD
type: review
tags: [review, ...]
ai-first: true
```

### `type: research` / `type: research-deep` / `type: x-read` / `type: x-pulse` / `type: youtube` / `type: podcast`

研究类笔记的完整 schema 分散在 `commands/research*.md`、`commands/x-*.md`、`commands/youtube.md`、`commands/podcast.md`。共同要求是设置 `ai-first: true` 并遵循通用规则。

### `type: podcast`

```yaml
date: YYYY-MM-DD
time: HH:MM
type: podcast
show: ""
host: ""
episode-title: ""
episode-url: ""
feed-url: ""
source-url: ""
guid: ""
published: ""
duration: ""
transcript-source: rss-transcript-tag | whisper-api | show-notes
tags: [research, podcast, ...]
cost-usd: 0.0
ai-first: true
```

### `type: adr`

```yaml
date: YYYY-MM-DD
type: adr
tags: [adr, decision]
decision: ""
status: proposed | accepted | superseded
related-projects: ["[[Projects/...]]", ...]
supersedes: "[[Knowledge/ADR-...]]"
ai-first: true
```

### `type: synthesis` / `type: emerge` / `type: connect` / `type: challenge`

思考工具的输出，保存到 `Knowledge/` 或 `Ideas/`:

```yaml
date: YYYY-MM-DD
type: <thinking-tool-type>
tags: [research, thinking, ...]
sources: [...]
related-people: [...]
related-projects: [...]
ai-first: true
```

### `type: distillation`

由 `/obsidian-distill` 写入。它是对单个来源的压缩视图，每个主张都带 `(src: Bn)` 指针，指向底部列出的编号来源块。这样 distillation 可以对照原文审计。`source` 是被蒸馏内容的原样路径或 URL，是时效/验证锚点。来源未陈述的推断必须放在独立标注章节，不能混进蒸馏主张。

```yaml
date: YYYY-MM-DD
type: distillation
source: "<verbatim path or URL of the distilled source>"
source-blocks: 0
tags: [distillation, thinking]
related-people: [...]
related-projects: [...]
ai-first: true
```

### `type: agenda-snapshot`

由日历命令写入，是日历的可重建时间点快照。Google Calendar 是真相源，不是这条笔记。`fetched-at` 是时效锚点。

```yaml
date: YYYY-MM-DD
type: agenda-snapshot
range: "YYYY-MM-DD..YYYY-MM-DD"
range-label: today
calendar-source: google-calendar
calendars: [primary]
fetched-at: "YYYY-MM-DDTHH:MM:SS+HH:MM"
event-count: 0
conflict-count: 0
tags: [agenda, calendar]
ai-first: true
```

### `type: meeting`

由日历事件生成。Notes / Decisions / Action items 章节必须从空开始，不能编造会议内容。

```yaml
date: YYYY-MM-DD
type: meeting
event-id: ""
event-url: ""
conference-url: ""
start: "YYYY-MM-DDTHH:MM:SS+HH:MM"
end: "YYYY-MM-DDTHH:MM:SS+HH:MM"
duration-min: 0
organizer: ""
attendees: ["[[People/...]]", ...]
tags: [meeting]
ai-first: true
```

### `type: recurring-task`

由 `/obsidian-recurring` 写入，用于追踪重复义务。它有周期和计算出的 `next-due`，每次完成后推进。History 章节记录每次发生。

```yaml
date: YYYY-MM-DD
type: recurring-task
cadence: ""
owner: ""
blocker: "[[People/...]]"
next-due: YYYY-MM-DD
amount: ""
tags: [recurring-task]
ai-first: true
```

### `type: architecture-overview`

由 `/obsidian-architect` 写入。它是代码库的顶层地图: 技术栈、模块、图、persona。位于 `Projects/<name>/Architecture/`。

```yaml
date: YYYY-MM-DD
type: architecture-overview
project: "[[Projects/...]]"
stack: []
scanned-commit: ""
tags: [architecture]
ai-first: true
```

### `type: architecture-module`

由 `/obsidian-architect` 写入，一个核心模块一条: 做什么、依赖什么、在整体中的角色。

```yaml
date: YYYY-MM-DD
type: architecture-module
project: "[[Projects/...]]"
module: ""
path: ""
scanned-commit: ""
tags: [architecture]
ai-first: true
```

## 常见反模式

| 反模式 | 为什么糟糕 |
|---|---|
| `date: today` | 以后读到时 today 已经无意义，要写实际 `YYYY-MM-DD` |
| 没有日期的裸事实 | “Mem0 是领导者”没有时点，无法验证 |
| 缺少外部 URL | “某研究说”无法追溯具体研究 |
| 人名用纯文本而不是 `[[wikilinks]]` | 图谱断裂，未来 Claude 无法遍历 |
| “见上文”“如前所述” | 单独拉出这条笔记时上下文丢失 |
| 依赖模型自己推断 | 类型、规则、来源都要显式写出 |
| 大段散文叙事 | 面向检索时，结构化项目符号通常更好 |
| 忘记 `ai-first: true` | 未来 Claude 不知道这条笔记是否符合标准 |
| 误用不可见替换字符 | 例如 em dash、弯引号、Unicode 数学符号会被 hook 检查 |

## 审计清单

- [ ] frontmatter 后有 `## For future Claude` 前言。
- [ ] frontmatter 中有 `ai-first: true`。
- [ ] `type:` 字段正确。
- [ ] `date:` 是 `YYYY-MM-DD`。
- [ ] tags 包含类型。
- [ ] 所有人、项目、概念都使用 `[[wikilinks]]`。
- [ ] 外部主张同时有时效标记和来源 URL。
- [ ] 多来源内容标注置信度。
- [ ] 没有 “see above” 这类上下文依赖引用。
- [ ] 自包含，零上下文可读。
- [ ] 没有编造事实、实体或日期，未知项标为 `TBD`。
- [ ] 任何“没有笔记 / 没找到”的结论都经过穷尽搜索，而不是凭记忆。

## 迁移说明

原规则在 2026-04-25 建立，并随 obsidian-second-brain v0.5.0 研究工具包发布。5 个研究命令从第一天就遵守它。26 个既有 `/obsidian-*` 命令在 v0.6.0 中显式引用此文档。更早写入的笔记可能不符合标准，可由 `/obsidian-health` 标记。
