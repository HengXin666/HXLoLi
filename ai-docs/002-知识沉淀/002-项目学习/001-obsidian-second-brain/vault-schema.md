---
title: Vault 结构规范
---

# Vault 结构规范

对应原文: `ref/obsidian-second-brain/references/vault-schema.md`

## 默认结构: Wiki-style / LLM-first

这个结构适用于 Claude 承担大部分或全部写作的 vault。主要读者是 LLM，不是人类。Obsidian 是存储引擎，Claude 是交互界面。

```text
Your Vault/
├── _CLAUDE.md                  <- Claude 的操作手册
├── index.md                    <- 全页面目录，Claude 优先读取
├── log.md                      <- 每次 vault 操作的时间顺序日志
├── SOUL.md                     <- 身份、价值观、沟通风格
├── CRITICAL_FACTS.md           <- 约 120 tokens，总是加载: 时区、经理、位置、公司
│
├── raw/                        <- 不可变。Claude 读取但不修改。
│   ├── articles/               <- 剪藏文章、网页
│   ├── transcripts/            <- 会议记录、播客转录
│   ├── pdfs/                   <- 文档、报告
│   └── videos/                 <- YouTube 元数据和转录
│
├── wiki/                       <- Claude 工作区。Claude 维护这里的一切。
│   ├── entities/               <- 人、公司、工具，扁平结构
│   ├── concepts/               <- 想法、框架、方法论
│   ├── projects/               <- 项目笔记
│   ├── daily/                  <- 每日笔记
│   ├── logs/                   <- dev log、work log
│   ├── reviews/                <- 周/月回顾
│   ├── tasks/                  <- 独立任务笔记
│   └── decisions/              <- ADR 架构决策记录
│
├── boards/                     <- Kanban boards
├── templates/                  <- 笔记模板
└── _trash/                     <- 软删除笔记
```

关键原则:

- `raw/` 不可变: 原始来源放这里。Claude 只读不改。wiki 页面损坏时，可从 raw 重新推导。
- `wiki/` 是 Claude 工作区: Claude 是唯一写入者。每个人、概念、项目都在这里。
- `index.md` 是入口: Claude 先读它来导航，比搜索更便宜、更快。
- 扁平文件夹优先于嵌套: `wiki/entities/` 是扁平列表，不适合人类浏览，但适合 Claude grep 和索引。

## 备选结构: Obsidian-style / Human-first

适用于每天在 Obsidian 中浏览 vault 的用户。文件夹按人类空间记忆组织。

```text
Your Vault/
├── _CLAUDE.md
├── index.md
├── log.md
├── Home.md                     <- 带 dataview 查询的 dashboard
│
├── Daily/                      <- 每日笔记
├── Dev Logs/                   <- 技术工作日志
├── Tasks/                      <- 独立任务笔记
├── Projects/                   <- 项目笔记
├── People/                     <- 每人一条笔记
├── Boards/                     <- Kanban boards
│
├── Knowledge/                  <- 参考材料和学到的东西
├── Learning/                   <- 书、课程、消费过的内容
├── Ideas/                      <- 想法捕获
├── Content/                    <- 内容日历和草稿
│
├── Goals/                      <- 年度和人生目标
├── Health/                     <- 健康追踪
├── Finances/                   <- 每月财务笔记
├── Jobs/                       <- 雇佣或合同角色
├── Businesses/                 <- 自己拥有的公司
├── Mentions/                   <- 认可/提及日志
├── Reviews/                    <- 周/月回顾
│
├── Templates/                  <- 笔记模板
└── _trash/                     <- 软删除笔记
```

## 文件夹映射

| Wiki-style | Obsidian-style | 内容 |
|---|---|---|
| `raw/articles/` | `Knowledge/` | 原始来源材料 |
| `wiki/entities/` | `People/` + `Jobs/` + `Businesses/` | 人、公司、工具 |
| `wiki/concepts/` | `Ideas/` + `Learning/` | 想法、框架、方法论 |
| `wiki/projects/` | `Projects/` | 活跃和归档项目 |
| `wiki/daily/` | `Daily/` | 每日笔记 |
| `wiki/logs/` | `Dev Logs/` | 工作会话日志 |
| `wiki/reviews/` | `Reviews/` | 周/月回顾 |
| `wiki/tasks/` | `Tasks/` | 独立任务笔记 |
| `wiki/decisions/` | 项目笔记内部 | ADR |
| `boards/` | `Boards/` | Kanban boards |

## Frontmatter schema

### Daily Note

```yaml
---
date: 2026-03-24
tags:
  - daily
mood: 4
energy: 3
---
```

### Project Note

```yaml
---
date: 2026-03-24
tags:
  - project
status: active
job: "[[Acme Corp]]"
timeline:
  - fact: "status: planning"
    from: 2026-03-01
    until: 2026-03-15
    learned: 2026-03-01
  - fact: "status: active"
    from: 2026-03-15
    until: present
    learned: 2026-03-15
---
```

### Task Note

```yaml
---
date: 2026-03-24
tags:
  - task
status: in-progress
project: "[[Project Name]]"
job: "[[Company]]"
requested_by: "[[Person Name]]"
due: 2026-03-28
---
```

### Entity Note

用于人、公司、工具。

```yaml
---
date: 2026-03-24
tags:
  - entity
  - person
role: "Senior Engineer"
company: "[[Acme Corp]]"
last_interaction: 2026-03-24
timeline:
  - fact: "CTO at Acme Corp"
    from: 2024-01-01
    until: 2026-04-07
    learned: 2026-02-23
  - fact: "Architect at Acme Corp"
    from: 2026-04-07
    until: present
    learned: 2026-04-07
    source: "[[2026-04-07]]"
---
```

双时间事实规则: 不要覆盖角色、公司、状态或位置。新增 `timeline:` 条目:

- `from` / `until`: event time，即事实在现实中为真的时间。
- `learned`: transaction time，即 vault 首次记录该事实的时间。
- `source`: 可选，vault 从哪里得知该事实。

顶层 `role:` 和 `company:` 永远反映当前状态。`timeline:` 保留完整历史。

它支持:

- 历史查询: “一月谁是 CTO?”
- 反思: “周二你相信 X，周三摄入 Y 后理解转成 Z。”
- 智能调和: 不同时间的不同角色不是矛盾。
- 审计轨迹: vault 何时从哪里学到每个事实。

### Source Note

```yaml
---
date: 2026-03-24
tags:
  - source
source_type: article
source_url: "https://..."
content_hash: ""
---
```

### Concept Note

```yaml
---
date: 2026-03-24
tags:
  - concept
status: active
related_projects: []
---
```

### Dev Log

```yaml
---
date: 2026-03-24
tags:
  - devlog
project: "[[Project Name]]"
job: "[[Company]]"
---
```

### Decision Record / ADR

```yaml
---
date: 2026-03-24
tags:
  - decision-record
status: accepted
---
```

### Kanban Board

```yaml
---
kanban-plugin: board
---
```

### Goal

```yaml
---
date: 2026-01-01
tags:
  - goal
category: "career"
status: active
progress: 35
target_date: 2026-12-31
---
```

## 命名约定

| 类型 | 模式 | 示例 |
|---|---|---|
| Daily note | `YYYY-MM-DD.md` | `2026-03-24.md` |
| Dev log | `YYYY-MM-DD — Description.md` | `2026-03-24 — API Gateway Debug.md` |
| Entity | 全名，扁平 | `Jane Smith.md`, `Acme Corp.md` |
| Concept | 描述性标题 | `LLM-Wiki Pattern.md` |
| Project | 专有名称 | `My Project Name.md` |
| Source | `YYYY-MM-DD — Source Title.md` | `2026-04-06 — Karpathy LLM Wiki.md` |
| Decision | `ADR-YYYY-MM-DD — Title.md` | `ADR-2026-04-06 — Wiki Style Default.md` |
| Archive prefix | `_archived_` | `_archived_Old Project.md` |

## Dataview 查询模式

活跃项目:

```dataview
TABLE status, job FROM "wiki/projects"
WHERE contains(tags, "project") AND status = "active"
SORT file.name ASC
```

最近 daily notes:

```dataview
TABLE date, mood, energy FROM "wiki/daily"
SORT date DESC
LIMIT 7
```

所有实体:

```dataview
TABLE role, company, last_interaction FROM "wiki/entities"
WHERE contains(tags, "entity")
SORT last_interaction DESC
```

最近摄入的来源:

```dataview
TABLE source_type, source_url FROM "raw"
SORT date DESC
LIMIT 10
```
