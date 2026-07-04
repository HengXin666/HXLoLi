---
title: _CLAUDE.md 模板
---

# `_CLAUDE.md` 模板

对应原文: `ref/obsidian-second-brain/references/claude-md-template.md`

## 概念

`_CLAUDE.md` 是一个放在 vault 根目录的文件。它是 Claude 在 vault 中工作前第一个要读的东西。它给所有 Claude surface 提供同一份操作上下文，包括 Desktop、Code、VS Code、terminal，不依赖模型记忆。

## 如何生成

当用户要求 Claude 创建 `_CLAUDE.md` 时，Claude 应该:

1. 调用 `list_files_in_vault()` 映射 vault 结构。
2. 如果 `Home.md` 或同等 dashboard 存在，读取它。
3. 从 `Templates/` 读取 2 到 3 个模板。
4. 读取当前 kanban boards。
5. 用发现到的真实值填充模板。
6. 调用 `append_content("_CLAUDE.md", content)` 写入 vault 根目录。

## 模板结构译注

### 标题

```markdown
# Claude Operating Manual — [Your Name]'s Vault

> Read this file before doing anything in this vault.
> This is the single source of truth for how Claude operates here.
```

含义: 明确它是操作手册，是单一真相源。

### Section 0: AI-First Vault Rule

这一节必须最先读，适用于每条笔记。

核心内容:

- vault 是给 future-Claude 读取和推理的，不是优先给人类 review。
- owner 很少直接读笔记，而是调用 Claude 来检索、综合、连接多年积累的知识。
- 每条 Claude 写入的笔记必须自包含，有 `For future Claude` 前言，有机器可读 frontmatter，有时效标记，有原样来源，有 `[[wikilinks]]`，有置信度。
- 规则适用于所有 `/obsidian-*`、`/research*` 命令、定时 agents、直接 vault 写入。

### Section 0.5: 行动前验证实时状态

这节不是 vault 特定规则，而是通用操作原则。行动前要读实际代码、schema、部署分支、环境变量或实时数据，不要从陈旧上下文猜。

例子:

- 宣称 bug 前先读 schema/types。
- `git fetch origin` 并读部署分支，不只读本地 `main`。
- 做 anchor-based patch 前 grep 实际文件。
- 实时日期、费率、时间要获取，不从训练数据推断。
- 责怪代码前检查运行进程里的 env vars。
- mock tests 可能漏 schema drift，宣称完成前读一个真实 payload。

对 `HXLoLi` 来说，这条可以直接写进仓库级 AI 操作规范。

### Vault Identity

包含:

- owner。
- primary purpose。
- last updated。

### Folder Map

列出每个文件夹及用途，例如:

| Folder | Purpose |
|---|---|
| `Daily/` | 每天一条笔记 |
| `Projects/` | 活跃和归档项目 |
| `Tasks/` | 独立任务笔记 |
| `Boards/` | Kanban boards |
| `People/` | 每人一条笔记 |
| `Dev Logs/` | 技术工作日志 |
| `Knowledge/` | 参考材料和永久笔记 |
| `Templates/` | 笔记模板 |

### Key Files

明确关键入口:

- Dashboard。
- Work Board。
- Personal Board。
- Mentions Log。
- People Index。

### Active Context

重大项目或重点周期开始时更新:

- 当前最高优先级。
- 当前工作。
- manager。
- 关键同事。

### Auto-Save Rules

Claude 可以不问就自动保存:

- 对话中形成的决策 -> 项目笔记 + daily note。
- 新提到的人 -> People，必要时创建 stub。
- 分配或承诺的任务 -> kanban board + Tasks note。
- 完成的开发工作 -> Dev Logs + 项目笔记 + daily note。
- 同事的提及/认可 -> Mentions Log + 人物笔记。
- 完成任务 -> 移到 kanban Done。

Claude 应先询问:

- 财务或个人金融数据。
- 私密文件夹。
- 删除或归档已有笔记。

### Naming Conventions

典型规则:

- Daily notes: `YYYY-MM-DD.md`
- Dev logs: `YYYY-MM-DD — Description.md`
- Deals: `Client Name - Description Month Year.md`
- Tasks: 描述性标题，不带日期前缀。
- People: 全名。
- Archive prefix: `_archived_`

### Frontmatter Requirements

最低要求:

```yaml
---
date: YYYY-MM-DD
tags:
  - [note-type]
---
```

note types 示例:

```text
daily | project | task | person | devlog | deal | goal | mention | content
```

### Kanban Convention

列:

```text
📥 Backlog · 📋 This Week · 🔨 In Progress · ⏳ Waiting On · ✅ Done
```

优先级:

```text
🔴 critical · 🟡 important · 🟢 low
```

事项格式:

```markdown
- [ ] 🔴 **Title** · @{YYYY-MM-DD}
	Description. [[Related Project]] [[Person]]
```

完成格式:

```markdown
- [x] ~~🔴 **Title**~~ ✅ Date
```

### Propagation Rules

| Event | Also update |
|---|---|
| New project | Board Backlog + today's daily note |
| Task done | Board Done + project note + daily note |
| Dev session | Dev Logs + project note + daily note |
| Person interaction | daily note + People note |
| Decision made | project Key Decisions + daily note |
| Mention/recognition | Mentions Log + person note + daily note |
| Deal update | deal file + Side Biz board + daily note |

### People to Know

列出最相关的人，减少 Claude 每次重新发现:

```markdown
| Person | Role | Notes |
|---|---|---|
| [Name] | [Role] | [One-line context] |
```

### Active Projects

保持当前活跃项目列表:

```markdown
- `[[Projects/Project Name]]` — [one-line status]
```

### Do Not Touch

明确不能动的范围:

- `Templates/`: 正常 vault 操作中不改模板。
- 私密文件夹: 只有明确要求才读。
- 其他私密或敏感目录。

## 保持 `_CLAUDE.md` 新鲜

以下情况应该更新或重新生成:

- 新重大项目开始。
- 团队变化，例如新 manager、新同事。
- 文件夹重构。
- 活跃优先级显著变化。

触发语:

```text
Update my _CLAUDE.md
Regenerate my vault manifest
```

## 对 HXLoLi 的改写建议

`HXLoLi` 不需要 `_CLAUDE.md` 这个名字，但需要等价文件。建议:

```text
HXLoLi/AI_RULES.md
HXLoLi/ai-docs/000-规范/index.md
```

内容应包括:

- `docs/`、`blog/`、`ai-docs/` 的边界。
- 哪些内容可自动生成，哪些必须人工 review。
- 私有文章和加密内容规则。
- AI 生成内容进入 `ai-docs/`，review 后才能毕业到 `docs/` 或 `blog/`。
- 写入前必须读现有同目录文档，匹配风格。
- 外部事实需要日期和 URL。
- 生成区使用 sentinel，不能覆盖人工区。
