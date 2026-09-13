---
hxid: "hx-a953a087"
title: "obsidian-second-brain 全解: 一个会自我重写的 AI 知识库"
created_at: "2026-09-13"
model: "deepseek-v4-flash"
skill: ["hx-to-ai-docs", "hx-docs-organize", "hx-docs-layout", "hx-docs-ppt", "hx-archify"]
authors: "Heng_Xin"
tags: ["记忆系统", "知识库", "Skill", "AI Agent"]
---

# obsidian-second-brain 全解: 一个会自我重写的 AI 知识库

> [!NOTE]
> 你的笔记库在半年后会比今天更聪明吗?
> 大多数"第二大脑"的答案是不会 —— 它们只是一个更整齐的文件柜: 放进去的东西原样躺着, 三个月前的决策和昨天的决策互相矛盾, 而没有任何人知道.
> `obsidian-second-brain` 想换掉这个前提: **知识库不是往里面加东西, 而是围绕新信息重写自己**.
> 那么, 一个不靠增长、只靠进化的知识库, 究竟由哪些机制拼出来?

## 0x00 它要解决什么问题

两个各自都很强、却完全断开的工具:

| | Claude Code | Obsidian vault |
|---|---|---|
| 强在哪 | 推理、写作、联网抓取 | 长期保存、可检索、可 diff |
| 弱在哪 | 每次会话从零开始, 关掉就忘 | 只是文件堆, 没人连接线索 |

结果是重复决策、想法烂在 daily note 里、没有人反驳你. 这个项目从 2026 年 4 月起就在做一件事: 让 Claude 直接以 vault 为工作区读写, 并把"vault 该怎么被写"变成一套**可机械检查的规则**, 而不是一段靠自觉遵守的约定.

它把自己定位为 Karpathy 的 LLM Wiki 模式的演进版:

| 维度 | Karpathy 的 LLM Wiki | obsidian-second-brain |
|---|---|---|
| 新来源 | 追加新页面并交叉引用 | **重写已有页面**, 替换陈旧主张 |
| 矛盾 | 标出来, 人工解决 | `/obsidian-reconcile` 自动调和 |
| 模式发现 | 用户问才浮现 | `/obsidian-synthesize` 自己找 |
| 运行时机 | 按需 | 4 个定时 agent 在夜里维护 |
| 笔记格式 | 人类可读的 wiki 页 | AI-first: 为未来 AI 检索而写 |

一句话: 如果说 Karpathy 的 wiki 是**你用 LLM 维护的知识库**, 那这个项目想做的是**自己维护自己的知识库**.

### 它不是 Obsidian 插件

一个常见误会值得先澄清, 因为它决定了能力边界:

| | Obsidian 插件 | obsidian-second-brain |
|---|---|---|
| 运行位置 | Obsidian 进程内部 | Claude Code 等 CLI 内部 |
| 能力上限 | 受 Obsidian API 限制 | 受"shell 能做什么"限制 |
| 典型能力 | UI、编辑器扩展、面板 | 联网研究、定时任务、跨年综合 |
| 对 vault 的认知 | 通过 Vault API | 就是一堆普通 Markdown 文件 |

vault 对它而言只是普通 Markdown, 这正是它能用同一套规则同时跑在 Claude Code、Codex CLI、Gemini CLI、OpenCode、Hermes、Pi 六个平台上的原因.

## 0x01 七条 AI-first 笔记规则

这是整套东西的地基, 也是唯一被每个命令、每个 hook、每个定时 agent 共同引用的文档. 它的前提是反直觉的:

> **vault 是写给未来的 AI 读的, 不是给人逐页读的.**

你很少直接打开笔记; 你调用 Claude 在多年积累里检索、综合、连接. 所以笔记格式要优先服务"被检索", 而不是"被阅读".

| # | 规则 | 落地形式 |
|---|---|---|
| 1 | 自包含上下文 | 单条笔记被单独拉出来也能读懂, 不依赖反向链接 |
| 2 | `## For future Claude` 前言 | frontmatter 之后立即写 2~3 句摘要 |
| 3 | 丰富且一致的 frontmatter | `date` / `type` / `tags` / `ai-first: true` |
| 4 | 外部主张带时效标记 | `(as of 2026-04, mem0.ai/blog/series-a)` |
| 5 | 原样保留来源 URL | 不改写成抽象描述, 方便多年后重验 |
| 6 | 强制交叉链接 | 人 / 项目 / 概念一律写成 `[[wikilinks]]` |
| 7 | 标注置信度 | `stated` / `high` / `medium` / `speculation` |

每条笔记的固定开头长这样:

```markdown
## For future Claude
This note is a [type] about [topic] saved on [date]. It [main purpose].
[Optional caveat about staleness, confidence, or scope.]
```

### 三条反幻觉铁律

写规则之外还有读规则, 因为下面三种失败会**静默**毁掉 vault 的记忆价值 —— 页面照样打开, 只是内容开始骗人:

- **错误地宣称不存在**: 没有穷尽搜索就说"没有这条笔记". 原文明确指出, 这是**比编造更常见**的失败模式. 结论必须是列目录 + grep + 遍历所有别名之后才得出的.
- **搜索完备性**: 扫描必须穷举, 不能采样. 把部分扫描汇报成完整扫描, 比诚实说"我只检查了 X"更糟.
- **不编造**: 未知就写 `TBD`; 没有决策时, 一个空的 `## Decisions` 章节不是缺陷, 而是正确答案.

### 类型 schema 是可增长的

`type:` 决定这条笔记带哪些 frontmatter 字段. 原文定义了十余种: `daily` / `project` / `person` / `idea` / `task` / `decision` / `devlog` / `review` / `research` / `podcast` / `adr` / `synthesis` / `distillation` / `meeting` / `recurring-task` / `architecture-overview`. 唯一的共同约束是: **通用字段只能增加, 不能删除**.

## 0x02 写入不是保存, 是传播

这是整个项目里最有迁移价值的一条: **不要孤立地创建笔记**.

每次写入都要追问"这件事还属于哪里", 然后把变更铺开.

| 事件 | 还要更新 |
|---|---|
| 新项目创建 | kanban board 的 Backlog + 今天的 daily note |
| 任务完成 | board 移到 Done + 项目笔记 + daily note |
| 人物互动 | daily note + 人物笔记 (不存在就建 stub) |
| 决策产生 | 项目笔记的 Key Decisions + daily note |
| dev log 创建 | 项目笔记 Recent Activity + daily note |
| 任意 vault 写入 | 操作日志 + `index.md` 目录 |

### 写入前先搜索

创建任何笔记之前必须先搜一遍. 命中同一概念就**更新**, 不新建; 名字相似但概念不同才允许新建, 而且要换一个更清楚的名字. 重复笔记被原文称为 "vault rot".

### 章节注入与 sentinel

更新已有笔记时不要粗暴追加或覆盖, 而是走固定动作: 读取完整文件 -> 定位目标章节 -> 插到该章节最后一项之后.

对**会被重复生成**的页面 (架构文档、dashboard、状态页), 则用 sentinel 把机器区和人工区隔开:

```markdown
<!-- @generated:start -->
...下次刷新时可以安全覆盖...
<!-- @generated:end -->

<!-- @user:start -->
...人工补充, 任何刷新永不触碰...
<!-- @user:end -->
```

刷新时只替换 `@generated` 区间; 标记之外的一切都视为人工所有. 这条让"可重复运行的生成器"第一次变得安全, 也是 [Agent Memory 选型指南](../005-Agent-Memory框架选型/index.md "hxid:hx-3ac6bdbe") 里反复强调的"可重建 / 不可重建"分界线在写入侧的落地.

### 双时间事实

当事实变化时 —— 角色、公司、状态、位置 —— **不要删掉旧值**:

```yaml
timeline:
  - fact: "CTO at Acme Corp"
    from: 2024-01-01
    until: 2026-04-07
    learned: 2026-02-23
    source: "[[2026-02-23]]"
  - fact: "Architect at Acme Corp"
    from: 2026-04-07
    until: present
    learned: 2026-04-07
```

`from` / `until` 是事实在现实中为真的时间 (event time), `learned` 是 vault 得知它的时间 (transaction time). 顶层字段永远反映当前状态, `timeline:` 保留完整历史 —— 于是"一月时谁是 CTO"和"你周三之后看法为什么会变"都变得可查.

## 0x03 一次摄入的完整数据流

一个 URL 进来, vault 里发生了什么:

```text
用户运行 /obsidian-ingest <来源>
        |
        v
命令正文 (由 commands/ 编译到 dist/<platform>/)
        |
        |-- 引用 references/ai-first-rules.md
        |-- 把确定性工作交给 scripts/* (解析 / 抓取 / 扫描)
        v
写出 AI-first Markdown 到 vault
        |
        |-- Claude Code 下由 validate-ai-first.sh 校验
        |-- 传播到 index.md / 操作日志 / 相关笔记 / daily note
        v
后续 /obsidian-world 与 load_vault_context.py 再读回状态
```

它的评价标准值得原样抄下来:

> **摄入后 vault 应该变得不同, 而不只是变大.**

旧页面如果没有变得更聪明、更互联、更当前, 这次摄入就不够深.

[一次摄入的传播路径: 从原文到综合页 #ppt ##w100%##](ingest-dataflow.html)

## 0x04 vault 的结构与两种布局

| 布局 | 主要读者 | 适用场景 |
|---|---|---|
| Wiki-style / LLM-first | LLM | Claude 承担几乎全部写作 |
| Obsidian-style / Human-first | 人 | 每天自己在 Obsidian 里浏览 |

LLM-first 的骨架:

```text
vault/
├── _CLAUDE.md          # 操作手册, 所有规则里优先级最高
├── index.md            # 全页面目录, Claude 优先读它而不是搜
├── log.md              # 追加式操作日志
├── SOUL.md             # 身份与价值观
├── CRITICAL_FACTS.md   # 约 120 tokens, 每次会话必加载
├── raw/                # 不可变原始来源, 只读不改
├── wiki/               # Claude 工作区
│   ├── entities/  concepts/  projects/  daily/
│   └── logs/      reviews/   tasks/     decisions/
├── boards/             # kanban
├── templates/          # 模板
└── _trash/             # 软删除
```

三条关键原则:

- **`raw/` 不可变**: wiki 页面损坏时, 可以从 raw 重新推导 —— 这是"派生内容可全量重建"的前提.
- **`index.md` 是入口**: Claude 先读索引来导航, 比搜索更便宜也更快.
- **扁平优先于嵌套**: `wiki/entities/` 是扁平列表, 不适合人浏览, 但适合 grep 与索引.

### `_CLAUDE.md`: 整个项目最重要的一个概念

它放在 vault 根目录, 是 Claude 进入这个 vault 前第一个要读的文件. 没有它, 每次对话都要重新学一遍 vault 的惯例; 有了它, Desktop、Code、VS Code、terminal 所有 surface 共享同一份操作上下文.

优先级规则很硬: **`_CLAUDE.md` 覆盖 skill 默认规则**, skill 默认值只在它沉默时才生效. 它记录的东西包括: 文件夹地图、每种笔记类型的 frontmatter schema、命名约定、哪些内容可以自动保存、哪些必须先问 (财务数据、私密文件夹、删除或归档).

## 0x05 44 个命令与四层能力

命令按 frontmatter 的 `category:` 分四类; 43 个跨平台, `/obsidian-calendar` 依赖 Google Calendar MCP, 只在 Claude Code 和 Pi 上完整可用.

| 分类 | 数量 | 干什么 |
|---|---:|---|
| `vault` | 16 | 保存、捕获、查找、任务板、项目状态 |
| `thinking` | 13 | 反驳、浮现模式、综合、决策、回顾 |
| `research` | 8 | X / Web / YouTube / podcast 研究并落库 |
| `meta` | 7 | 初始化、健康检查、导出、可视化、架构文档 |

它们按四层能力组织, 外加一个常驻层:

```text
LAYER 1  Operations       Claude 记住一切
LAYER 2  Thinking Tools   Claude 和你一起想
LAYER 3  Context Engine   Claude 知道你是谁
LAYER 4  Research         Claude 把外部知识拉进来
ALWAYS   background + scheduled agents
```

挑几个最能说明设计意图的:

| 命令 | 一句话 |
|---|---|
| `/obsidian-save` | 从整段对话里抽取决策 / 人物 / 任务 / 想法, 保存到**正确**的笔记, 不问你该放哪 |
| `/obsidian-ingest` | 让 vault 围绕新知识重写自己, 一个来源通常触达 5~15 个页面 |
| `/obsidian-challenge` | 用你自己的历史反驳你: 找过往失败与反转过的决策 |
| `/obsidian-world` | 按 L0~L3 分级加载身份与当前状态, 控制 token 预算 |
| `/obsidian-architect` | 扫代码库写架构笔记, 重跑只刷新 generated 区 |
| `/obsidian-retrieval-eval` | 用问题集衡量搜索质量: recall@k、MRR、失败案例 |

`/obsidian-challenge` 的实际形态最能说明问题. 你说"我想用 Rust 重写 API", 它翻出 2025 年 Rust rewrite 失败的 post-mortem, 再翻出一份"未来两年继续用 TypeScript"的决策记录, 然后问:

> **你的笔记说这失败过. 还要继续吗?**

## 0x06 适配器模式: 一个源, 六个平台

核心思想: `commands/` 是唯一真相源, 构建系统把它编译到各平台, **而不是维护多套命令**.

- `commands/<name>.md` 沿用 Claude Code slash command 的形状, 并声明 `description:` / `category:` / `triggers_en:` / 可选 `exclude:`.
- `scripts/build.sh` 编排 `adapters/`: 全量构建, 或 `--platform <name>` 只构建一个平台.
- Claude Code 适配器基本是原样复制; 其他平台生成 dispatcher (`AGENTS.md` / `GEMINI.md`) 与自动路由表.
- 面向非 Claude CLI 时, 会把 Claude 特有说法中和成平台无关表达, 例如 `Read tool` 改成 `read files`.
- 产物落在 `dist/<platform>/` 并被 gitignore —— **任何时候都应该重新生成, 而不是手写修改**.

对贡献者的结论只有一句: 增加或修改命令时只改 `commands/<name>.md`, 下一次构建由适配器自动拾取.

[一个命令源如何编译到六个 CLI #ppt ##w100%##](vault-architecture.html)

## 0x07 自动化与它给自己设的边界

### 定时 agent

| Agent | 时间 | 工作 |
|---|---|---|
| `morning` | 08:00 | 建当天 daily note, 拉入今天到期或逾期的任务 |
| `nightly` | 22:00 | 关闭当天 / 调和矛盾 / 综合模式 / 修复孤立笔记 / 重建索引 |
| `weekly` | 周五 18:00 | 生成周回顾 |
| `health` | 周日 21:00 | vault 健康检查, **只报告不修复** |

再加一个 PostCompact hook: 上下文压缩之后启动 headless `claude -p`, 把这次会话里值得留的东西传播进 vault.

### 安全被写进了默认值

这部分比功能列表更值得学, 因为它承认了"自动写入是危险的":

- 后台 agent 默认**关闭**, 需要双开关 (`OBSIDIAN_VAULT_PATH` **且** `OBSIDIAN_BG_AGENT_ENABLED=1`) 才启用.
- 无人值守运行**只增不删**: 不删除、不归档、不合并.
- 健康检查里破坏性的修复动作 (归档、合并、解决矛盾) 必须显式确认.
- 写入时还有一道非阻塞 validator: 检查 frontmatter delimiter、是否混入 tab、四个必备字段、有没有 `## For future Claude` 前言; 失败只在 stderr 报警告, **不回滚写入**.

## 0x08 生态边界: upstream 给原语, fork 给领域

`ECOSYSTEM.md` 定义的契约很清楚:

- **upstream 拥有核心原语**: vault 管理、AI-first 规则、rewrite engine、多平台适配层、通用研究工具形状 (Phase 1 vault scan / Phase 2-3 external research / Phase 4 synthesis), 以及一个可插拔的 Phase 3 backend 协议.
- **fork 拥有全部领域知识**: PubMed routing 属于学术 fork, 案例法检索属于法律 fork.

理由是关于维护成本的: 把每个领域都吸进 upstream, 结果会是一个无人能维护的庞大 skill, 而且上游维护者不可能真正理解自己不用的受控词表. 回流标准也只有一个: **非领域用户是否也会受益**.

已知的第一个证明案例是学术方向的 `scholarbrain`. 不收的同样明确: 只换主题色的 fork、除 README 外没有领域功能的 fork、以及在自己的 vault 写入中违反 AI-first 规则的 fork.

## 0x09 这份复盘里最值得抄的四件事

上游对 166 个 fork 做过一次很诚实的分析, 结论比功能列表更有参考价值:

- 其中 **156 个是从未改动的镜像** (0 commits ahead), 11 个有自己的提交, 而真正做了实质工作的只有 3 个.
- 归纳出的强信号是: 付费 API 是采用门槛; 日历集成被多个 fork 独立要求; 上游**没有测试**; Codex / Windows 支持被反复验证; 缺少反幻觉 guard.

于是它们把研究工具全部补上免费无 key 模式, 把反幻觉与搜索完备性提到 P0, 并补了 smoke test 与 CI. 留下的可迁移条目是这四条:

1. **把"想学的机制"和"想抄的代码"分开** —— 抄设计, 不抄代码.
2. **写入规则必须机器可检查** —— 否则 AI 生成的内容很快会退化成不可审计的散文.
3. **可重复生成的页面必须带 sentinel** —— 否则第一次刷新就会擦掉人工补充.
4. **没有穷尽搜索, 就没有资格说"不存在"** —— 这一条同时是反幻觉规则与检索质量的前提.

顺着看这套东西与已有沉淀的关系, 可以对照 [对话记忆与知识库增量沉淀](../001-对话记忆与知识库增量沉淀/index.md "hxid:hx-462ef6c0") 里"追加 / 覆盖 / 快照"的三种策略分野, 以及 [自维护可插拔记忆层设计](../002-自维护可插拔记忆层设计/index.md "hxid:hx-37f17262") 里"文件即记忆 + 索引前门"的取舍.

## 0x0A 全局速览

[obsidian-second-brain 全解 ##PPT 1 sakura##](obsidian-second-brain-deck.tsx)

## 0x0B 参考来源

- [obsidian-second-brain (GitHub / eugeniughelbur)](https://github.com/eugeniughelbur/obsidian-second-brain) —— 本文全部分析的对象, 结论对应 commit `bf63932`.
- [Karpathy's LLM Wiki (gist)](https://gist.github.com/karpathy/442a6bf555914893e9891c11519de94f) —— upstream 自述所基于的原始模式: 丢入来源、LLM 生成 wiki 页面、再向 wiki 提问.
- ["I rebuilt Karpathy's LLM Wiki. Here's what's missing from the original." (The AI Operator)](https://theaioperator.io/p/i-rebuilt-karpathys-llm-wiki-heres) —— 作者本人对该模式的定位与补充说明.
