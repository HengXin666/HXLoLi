# Agent Note: obsidian-second-brain 从 11 篇译注浓缩为 1 篇

Status: implemented

- 影响: `ai-docs/002-AI/004-记忆/003-obsidian-second-brain/` + 跨文章链接 + 侧边栏 + 演示页注册表

## Problem

`003-obsidian-second-brain` 下原是按上游文件一对一切分的 11 篇译注:

```text
ai-first-rules.md  write-rules.md  vault-schema.md  commands-index.md
readme-public.md   skill-manual.md  fork-insights.md  ecosystem.md
claude-md-template.md  hx-loli-borrow-list.md  hx-loli-adoption-plan.md
```

这是**上游的目录结构**, 不是读者需要的结构. 后果:

1. 同一机制被拆到 3~4 篇里重复讲 (AI-first 规则同时出现在 ai-first-rules / write-rules / skill-manual / readme-public).
2. `readme-public.md` 605 行、`skill-manual.md` 522 行, 是产品文档体量, 但读者只想知道"这套东西怎么运转、哪几条能抄".
3. 「可借鉴点」与「采纳方案」两篇是面向 HXLoLi 的迁移建议, 与译注混排, 读者分不清哪些是事实、哪些是作者判断.
4. 11 篇里没有一张图、没有一个演示页, 全是表格与代码块.

## Decision

**按"读者要理解一条机制"重排为单篇**, 而不是按上游的文件边界切分.

| 旧 | 新 |
|---|---|
| 11 篇分片 | `index.md` 单篇 (12 章: 0x00~0x0B) |
| 纯文字 + 表格 | 2 张 archify 机制图 (`#ppt` 侧车) + 1 个总览演示页 (`##PPT##` .tsx) |

保留 `hxid: hx-a953a087` 不变, 因此两处外部引用 (`004-记忆/001`、`003-工具链/002`) 不需要改 hxid, 只重算了路径与链接文字.

## Alternatives considered

下面每条都是 Problem 里已记录的理由所否掉的选项, 不是事后补的:

- **保持上游的一对一分片 (11 篇)** —— 就是本次改动的起点. 后果见 Problem 四条: 同一机制被拆到 3~4 篇
  重复讲、单篇达到产品文档体量、事实与作者判断混排、全库无一张图.
- **保留 11 篇分片, 另加一篇总览做索引** —— 不解决问题本身: 读者仍要在 3~4 篇之间自己拼一条机制,
  重复叙述与 605 行产品文档体量都原样留下, 只是多了一个入口.
- **只删重复段落, 保持文件边界不动** —— 会让每篇都变成半截内容: 一条机制的部分事实留在原文件、
  部分被抽走, 读者失去"读到一段就是完整一条机制"的保证.
- **什么都不做** —— 这 11 篇是本仓库唯一一处按上游目录结构组织、而非按读者理解组织的知识区,
  不重排就会一直作为反例存在, 后续每次整理都要重新判断一次.

## Consequences

- 旧 11 个 .md 全部删除; `index.md` 重写为 17KB 单篇.
- 新增 `vault-architecture.html` (架构图: 一个命令源 -> 六个 CLI -> 一个 vault)、`ingest-dataflow.html` (数据流: 一次摄入的传播)、`obsidian-second-brain-deck.tsx` (16 屏总览演示页).
- `src/hxdeck/decks.generated.ts` 重新生成 (3 个演示页).
- 侧边栏重建、`hxid check` / `links` 全绿、tag 注册表 `generate` 重建.

## 顺带修掉的一个存量缺陷

`DocTagDocListPage` 的「同字母兄弟标签」会链接到 `count === 0` 的**纯结构 tag** (`编程语言` / `工程与工具` / `生活杂谈`). 这类 tag 只在注册表里当 parent 用、没有笔记挂在自己名下, Docusaurus 不为它生成路由 —— 构建报 6 条 broken link. 已在 `siblingTags` 里加 `entry.count > 0` 过滤.

## 未做

- 没有新建 `ai-docs/000-规范`, 也没有落地 `AI_RULES.md` 与 health check 脚本: 那是原「采纳方案」的内容, 属于项目自身的工程决策, 需要单独拍板, 不在本次浓缩范围内.
