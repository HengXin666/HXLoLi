---
title: 写入规则
---

# 写入规则

对应原文: `ref/obsidian-second-brain/references/write-rules.md`

## 前置规则

写任何 Obsidian vault 笔记前，都要先读 `references/ai-first-rules.md`。每条 Claude 写入的笔记都必须遵守 AI-first 规则: 前言、丰富 frontmatter、时效标记、强制 wikilink、原样来源、置信度。

本文件是在 AI-first 规则之上的操作细则，规定 Claude 如何写、链接、格式化和更新笔记。

## 传播规则

不要孤立创建笔记。每次写入都有涟漪效应。

写入或更新某项内容时，要继续追问: 还有哪些笔记需要知道这件事?

```text
新项目创建
  -> 加到 kanban board 的 Backlog
  -> 从今天的 daily note 链接过去
  -> 如果涉及某人，也从这个人的笔记链接

任务完成
  -> kanban 卡片移动到 Done，并加删除线
  -> 更新项目笔记的 Recent Activity 或 Delivered
  -> 写入今天的 daily note

人物笔记更新
  -> 如果今天发生互动，写入 daily note
  -> 如果有提及或 shoutout，加入 Mentions Log

Dev log 创建
  -> 从项目笔记 Recent Activity 链接
  -> 从今天的 daily note Work / Work Log 链接

决策产生
  -> 写入项目笔记 Key Decisions
  -> 写入今天的 daily note

交易推进
  -> 更新 deal 文件的状态、概率、说明
  -> 更新 Side Biz kanban board
  -> 反映到 daily note
```

## 内部链接

使用 `[[Note Name]]` 语法。必须链接:

- 笔记里提到的人: `[[Jane Smith]]`
- 引用的项目: `[[My Project Name]]`
- 公司/工作: `[[Acme Corp]]`
- 相关任务: `[[Task Name]]`

除非必要，不要硬编码路径。Obsidian 会按文件名解析 `[[Name]]`。

如果被链接的笔记还不存在，要创建它。stub 也可以，但至少要有 frontmatter、标题和一行上下文。

## 日期格式

| 场景 | 格式 | 示例 |
|---|---|---|
| frontmatter `date` | `YYYY-MM-DD` | `2026-03-24` |
| frontmatter `due` | `YYYY-MM-DD` | `2026-03-28` |
| Kanban 到期标签 | `@{YYYY-MM-DD}` | `@{2026-03-28}` |
| 正文日期 | 人类可读格式 | `March 24` 或 `Mar 24` |
| 带日期文件名 | `YYYY-MM-DD` | `2026-03-24.md` |

## Kanban board 格式

board 使用 `kanban-plugin: board` YAML frontmatter。列是 H2 标题。事项是任务 checkbox，可带缩进说明。

活动项:

```markdown
- [ ] 🔴 **Task Title** · @{2026-03-28}
	One-line description. [[Related Project]] [[Person]]
```

等待项:

```markdown
- [ ] 🟡 **Task Title** · @{2026-04-07}
	Context for why it's blocked. [[Person responsible]]
```

完成项移动到 `## ✅ Done` 列:

```markdown
- [x] ~~🔴 **Task Title**~~ ✅ Mar 24
	Brief note on outcome.
```

优先级约定:

- 🔴 Critical / blocking
- 🟡 Important / this week
- 🟢 Nice to have / low urgency

不要删除完成项。完成项移动到 Done 列并加删除线。Done 列就是 changelog。

## 状态值

项目:

```text
active | planning | completed | archived | on-hold
```

任务:

```text
in-progress | done | waiting | cancelled
```

交易:

```text
prospect | negotiating | confirmed | completed | lost
```

目标:

```text
active | completed | paused | abandoned
```

内容:

```text
draft | scheduled | published
```

## 写作风格校准

第一次往某个文件夹写新笔记前:

1. 读该文件夹中 1 到 2 篇已有笔记。
2. 匹配标题结构、frontmatter 字段、语气、emoji 使用、列表风格、章节名。
3. 不引入全新惯例，只扩展已有惯例。

## 归档

优先使用软归档: 给文件名加 `_archived_` 前缀。

```text
Old Project.md -> _archived_Old Project.md
```

同时更新 frontmatter: `status: archived`。

不要删除 vault 笔记，要归档。vault 是永久记录。

## 模板使用

从模板创建笔记时，要删除所有 Templater 语法，例如 `<% ... %>`，并替换成实际值。保存的笔记里不能留下模板占位符。

## Stub 笔记

当链接目标还不存在时，创建最小 stub:

```yaml
---
date: 2026-03-24
tags:
  - person
---

# Person Name

<!-- Note created as stub. Expand when more info is available. -->
```

## 章节注入

更新已有笔记时，不要粗暴追加或覆盖。使用目标章节注入:

1. 读取完整文件。
2. 找到目标章节标题。
3. 把内容追加到该章节最后一个条目之后，下一个 `---` 或下一个 `##` 之前。
4. 写回完整文件。

kanban board 的处理: 找到正确列标题，把新项插到该列最后一个事项之前，或空列顶部。

## Sentinel 安全再生成

对命令生成且人类可能手改的笔记，例如架构文档、dashboard、可重复刷新的页面，要使用 sentinel 标记，确保刷新不会破坏人工编辑。

```markdown
<!-- @generated:start -->
...machine-generated content - safe to overwrite on the next run...
<!-- @generated:end -->

<!-- @user:start -->
...human notes - NEVER overwritten by a refresh...
<!-- @user:end -->
```

刷新规则:

1. 读取已有笔记。
2. 只替换 `@generated:start` 和 `@generated:end` 之间的内容。
3. 永远不碰 `@user` block，也不碰标记外任何内容，全部视为人工拥有。
4. 第一次运行且没有 marker 时，要给生成内容包上 `@generated` marker，方便下次安全刷新。

这让命令具备幂等、可重复运行的能力。用户不用担心刷新会擦掉自己的补充。原项目已在 `/obsidian-architect` 中使用。

## 写入前搜索

创建任何笔记前:

```text
search(query="keyword from title")
```

如果找到匹配:

- 同一概念: 更新已有笔记，不新建。
- 名字相似但概念不同: 可以创建，但要选一个更清楚的名字。

重复检测对人物、项目、交易特别重要，因为同一对象常有不同名字格式或工作标题。

不要凭记忆宣称不存在。写 “没有笔记” 或因为相信没有而创建前，必须穷尽搜索: 所有可能名称、别名、文件夹，列目录和 grep，而不是只跑一个查询。错误地宣称缺失是最常见失败模式。不确定时，宁可过度包含并标注不确定。
