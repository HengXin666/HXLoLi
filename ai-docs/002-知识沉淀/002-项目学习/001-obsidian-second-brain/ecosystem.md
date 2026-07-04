---
title: 生态边界
---

# 生态边界

对应原文: `ref/obsidian-second-brain/ECOSYSTEM.md`

## 核心观点

`obsidian-second-brain` 是 core。特定领域 fork 扩展这个模式。

这个页面列出已知 fork: 它们继承 vault-rewrite architecture，包括 AI-first rule、wikilink graph、Karpathy LLM Wiki 的 rewrite-vs-append 原则，然后应用到具体领域，例如学术研究、法律实践、金融、医学等。

每个 fork 保持独立。它们和 upstream 的关系是契约式的，不是托管式的:

- upstream 拥有核心 primitives。
- fork 拥有所有领域特定能力。

## 为什么要这样分工

vault skill 的价值取决于它携带的领域知识。PubMed routing 应该在 academic fork，不在 core。案例法检索应该在 legal fork，不在 core。

如果把每个领域都吸进 upstream repo，结果会是一个无人能维护的庞大 skill，而且会破坏 AI-first rule。因为上游维护者不可能真正理解自己不用的受控词表。

所以 upstream 提供 primitives:

- vault management。
- AI-first rule enforcement。
- rewrite engine。
- 多平台 adapter layer: Claude Code / Codex CLI / Gemini CLI / OpenCode。
- 通用 research toolkit 形状: Phase 1 vault scan、Phase 2/3 external research、Phase 4 synthesis。
- 可插拔 Backend protocol，用于 Phase 3，让任何领域添加自己的 research backends，而不用 fork 整个 engine。

fork 可以把领域工作中产生的通用 primitives 贡献回 upstream。判断标准: 非领域用户是否也会受益。

## 已知 fork

| Fork | 领域 | 维护者 | 状态 |
|---|---|---|---|
| `SHzzzAyys/scholarbrain` | 学术研究，PubMed、arXiv、Semantic Scholar | `@SHzzzAyys` | 活跃。Paper-card 受控词表、DeepSeek backend、语言感知 gap prompts。生态模式的第一个证明案例。 |

## 如何加入列表

如果你构建了一个领域特定 fork，并且真正扩展了这个模式，可以在 upstream Discussions 中发起讨论，并提供:

1. fork 仓库。
2. 它服务的领域简介。
3. 计划贡献回 upstream 的内容。

不以 stars、downloads、受众规模为门槛。门槛是这个 fork 是否真的以一种可供他人学习的方式使用了这套架构。

## 不属于此列表的内容

- 只改名或换主题颜色的个人 fork。
- 除 README 外没有实际领域功能的 fork。
- 在自己的 vault 写入中违反 AI-first rule 的 fork。
- 只是把 upstream 功能换名转售，没有真实领域工作的项目。

列表是 curated。如果 fork 停止维护或偏离模式，会被移除。

## 对 HXLoLi 的启发

这份生态边界对个人博客很有价值: 不要把所有东西都塞进一个通用命令系统。

建议:

- `HXLoLi` core 只保留博客/文档/AI 知识库的通用 primitives。
- C++、游戏引擎、番剧、音乐播放器等领域沉淀可以有独立 workflow。
- 可贡献/复用的通用部分沉淀到脚本和规则；强个人偏好的内容保留在个人知识区。
