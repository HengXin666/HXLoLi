---
title: "自维护可插拔记忆层设计: 一次对比市面方案后的收敛"
created_at: "2026-09-06"
model: "DeepSeek-V4-Flash"
skill: ["hx-make-ai-docs"]
authors: "Heng_Xin"
tags: ["AI Agent", "记忆系统", "知识库", "可插拔", "Port-Adapter", "泛化"]
---

# 自维护可插拔记忆层设计: 一次对比市面方案后的收敛

## 0x00 背景

要给 HXLoLi 的 AI 生态 (DSH 为主, 未来可能还有 Codex/其他 CLI) 配一套长期记忆层. 两个硬需求:

1. **所有 AI 都能读我的知识库** (从任何工作区, 只要我允许);
2. **知识是关联/渐进/可推广的** —— 不只是 top-k 相似度, 而是能从一个具体事故 (例如"后端队列并发控制踩坑") 抽象成跨项目可复用的经验 ("我所有容器/异步系统都有并发控制问题需要被注意到"), 并且能稳定地既覆盖业务目标又覆盖编码坑.

调研了市面方案 (VCP / OpenViking / agentmemory / StrataGate / ReMe / Basic Memory / Mem0 / Graphiti / cognee / obsidian-second-brain / 腾讯 Hy-Memory), 得到的判断是:

- 单个开源项目都能覆盖一部分需求, 但没有一个满足"我要自己维护 + 可插拔 + 带我要的那种跨项目推广能力"这三件事;
- **跨项目推广**: 经代码核查, OpenViking/agentmemory/ReMe 确有"具体→摘要抽象"机制 (见 0x02 五.1), 但它们都缺两环: (a) 抽象结果不自动跨项目生效 (agentmemory 按项目隔离, ReMe/OpenViking 的 digest/experience 绑单工作区); (b) 无用户级"经验门" (promote 成跨域规则的确认环节). 这正是自建层要补的核心差异点;
- 结论: 与其选型, 不如设计一层薄的、自维护的、端口-适配器 (Port-Adapter) 式的记忆层. 市面方案不是照搬对象, 而是**可插拔的 adapter 和存储后端, 以及设计思想来源**.

## 0x01 核心结论

> 记忆层 = 一个核心内核 (Port/抽象 API) + 多个可插拔 adapter (接入层: DSH / Codex / CLI...) + 多个可插拔 storage (存储层: 文件 / SQLite / 向量 / 图) + 一个**推广引擎** (把具体 → 一般).

1. **内核只定义抽象, 不实现任何接入和存储**. 接入层是 1 套 API 的多个实现 (DSH 用一个实现, Codex 用另一个); 存储层同理 (可能是很多库、很多表). 内核与实现之间靠接口解耦, 这就是"安全感"的来源: 想迭代哪个实现就迭代哪个, 不污染核心.
2. **市面方案的正确用法 = 抄设计不抄代码**. VCP 抄"Tag 河流/写时召回"思想; OpenViking 抄"L0/L1/L2 分层 + intent 检索"; agentmemory 抄"hooks 自动捕获 + lesson"; StrataGate 抄"L0-L5 + 证据门"; ReMe 抄"文件即记忆 + daily→digest 自演进"; Hy-Memory 抄"supersedes 演化链"; obsidian-second-brain 抄"typed relations + index 前门". 每个都只抄思想, 具体代码全部自维护.
3. **真正的差异化是"推广引擎" (Generalizer)**. 市面已有"具体到摘要抽象"的雏形 (OpenViking experiences / agentmemory reflect / ReMe dream digest, 见 0x02 五.1), 但缺三步: **跨项目生效 + 用户级经验门 + 规则与实例双向链接**. 自建层的推广引擎补的就是这三步: 把某项目踩坑, 经 LLM 抽象 + 人工确认, 升级为对"我所有容器/异步系统"都成立的跨项目不变量, 并让命中任一实例时自动带出整条规则. 触发方式已定: 后台自动聚类提议 + 用户 review 队列统一确认 (见 0x02 二).
4. **分层兼容 (R2) 与推广 (R1) 是两件事, 都要显式设计**. R2 靠 Port-Adapter 架构; R1 靠一个专门的"推广工作流" (不是检索时顺便做的).

## 0x02 关键细节

### 一、内核接口设计 (Port)

抽象出一个 `MemoryKernel` 接口, 只依赖领域概念, 不依赖任何 harness 或存储:

```ts
// 记忆对象: 统一形态, 但可以携带不同来源的原始载荷
interface MemoryEntry {
  id: string
  kind: "fact" | "preference" | "event" | "decision" | "lesson" | "rule" | "pattern" | "context"
  content: string        // 人类可读正文
  source: string         // 来源引用 (会话 id / 文件路径 / URL)
  scope: string          // 命名空间 (project / agent / global)
  ts: {                  // 双时态 (呼应 004 笔记)
    validAt: string      // 生效时间
    assertedAt: string   // 写入时间
  }
  relations?: Relation[] // 关系: supersedes / relates / generalizes / appliesTo...
}

// 存储端口: 内核只依赖这个接口
interface MemoryStore {
  add(entry: MemoryEntry): Promise<void>
  get(id: string): Promise<MemoryEntry | null>
  query(q: Query): Promise<MemoryEntry[]>   // 语义/关键词/关系 混合
  traverse(from: string, relation: string): Promise<MemoryEntry[]>
  update(id: string, patch: Partial<MemoryEntry>): Promise<void>  // 用于演化链
}

// 接入端口: 每个 harness 一个 adapter 实现它
interface HarnessAdapter {
  readonly name: "dsh" | "codex" | "cli" | ...
  onSessionStart(ctx): Promise<Injection>     // 会话开始注入什么
  onTurnEnd(turn): Promise<Capture[]>         // 轮次结束捕获什么
  onPreStep(step): Promise<Recall | null>     // 每步前是否召回 (可选, 最灵活)
  registerTools(registry): void               // 向 harness 注册记忆工具
  onConsolidationScheduled(schedule): void    // 让后台整理可被调度
}

// 推广引擎端口: 把具体 → 一般
interface Generalizer {
  generalize(entries: MemoryEntry[]): Promise<GeneralizationProposal>
  // 返回: 一条候选一般性规则 + 覆盖的具体实例 + 置信度 + 需要人工确认
}
```

### 二、推广引擎 (R1) —— 本设计最独特的机制

市面方案普遍只做到"检索得好", 没有"想得深". 推广引擎解决的是用户的真实诉求:

> "我在后端遇到了队列并发控制问题 → 通过记忆直接推广: 注意到我所有容器其实都有并发策略问题. "

工作流 (每次事故/决策后由 consolidation 触发, 非每轮):

```text
1. [具体实例] 某项目某次"队列并发踩坑"被捕获 (含上下文/代码/结论)  → 存为 kind: lesson, scope: project
2. [后台聚类] 后台 consolidate 发现 ≥N 条相关 lesson (同 tag/同 cause)  → 触发 generalizer (不打断会话)
3. [后台抽象] LLM 提取共同不变量: "消息/任务队列需显式设计并发上限与幂等" → 生成候选 rule (scope: global) + 覆盖实例清单
4. [review 队列] 候选 rule 写入 ai-docs 记忆层的待审队列 (review-queue/ 目录, 每条一个 md)  → 用户空闲时统一处理
5. [用户确认] 用户批量 review: 确认 → 升为 kind: rule; 驳回/改写 → 打回或附注; 全程不打断任何会话
6. [双向链接] rule ⇄ 每条来源 lesson 建立 generalizes/appliesTo 关系   → 之后任意项目命中 lesson 都能带出 rule
7. [生效] 规则进入"常驻轻量提示" (跨项目提醒), 或作为检索时的高权重证据
```

关键设计: **抽象后台做, 闸门用户开** (已确认方案 a). 机器可以批量聚类+提议, 但"把经验上升为对我所有项目都成立的规则"必须经用户统一 review 队列确认 — 呼应 HXLoLi 一贯的"AI 辅助沉淀需 review"纪律, 防幻觉式过度推广. review 队列本身也是一层记忆: 每条候选带置信度、覆盖实例数、建议动作 (确认/改写/驳回), 让用户的确认成本尽量低.

为什么它能"稳定阐述业务目标 + 编码坑": rule 有两种来源, 但共享同一 schema 与召回通道 —— 业务规则 (客户项目规范) 和工程规则 (并发/幂等/可观测性) 都作为 scope:global 的 rule 存在, 检索时按当前会话语境 (项目/角色) 加权.

### 三、接入层 Adapter: 具体到 DSH 的实现方式

DSH 的插件体系 (cordis) 给了天然的 adapter 挂载点. 参考 ReMe 的 cordis.patch.yml 与 StrataGate 的写法:

```yaml
# cordis.patch.yml (inject into a DSH profile)
- insert:
    - id: hx-memory
      name: "@deepseek-ai/cordis-plugin-group"
      group: true
      isolate: { hxMemory: true }
      config:
        - id: hx-memory-runtime
          name: "@hx/hx-memory-dsh"        # DSH adapter (one impl of HarnessAdapter)
        - id: hx-memory-client
          name: "@hx/hx-memory-client"     # Web UI (status / review queue)
```

DSH adapter 内部:
- 用 `ctx.on("agent/session-start")` 注入"记忆使用指引" (参考 ReMe guidance: 只注入指引, 不注入历史);
- 用 `ctx.on("turn/end")` 捕获完成对话轮 → 批量入记忆 (参考 ReMe autoMemory / agentmemory hooks);
- 用 `ctx.on("agent/pre-step")` 做可选的轻量召回注入 (参考 StrataGate 的 pre-step 注入 ≤ 900 tokens; DSH 的 pre-step 可以返回完整消息, 是比 session-start 更细的注入点);
- 用 `defineTool` 注册只读工具 `memory_search` + 主动工具 `memory_save` / `memory_rule_propose` (参考 ReMe reme_search);
- 后台 consolidation / generalization 用 DSH 的定时调度 (类似 ReMe dreamCron).

未来加 Codex: 写第二个 HarnessAdapter 实现 (Claude Code 用 hooks + SKILL.md; Codex 用 AGENTS.md + pre-exec hook; 纯 CLI 用 MCP). 内核零改动.

### 四、存储层: 文件优先 + SQLite 索引 + 可选向量

用户已有的 HXLoLi = Markdown 事实源 (blog/docs/ai-docs). 设计原则: **真相在文件里, 索引在库里** (可重建, 绝不反向覆盖).

| 存储 | 存什么 | 为什么 |
|---|---|---|
| Markdown 文件 (git) | 全部记忆正文 (daily/ digest/ rules/) | 人可读可改, diff/审计/版本控制, 与 HXLoLi 双链体系兼容 |
| SQLite | 索引: id/路径/tags/relations/双时态 | 精确查询/关系遍历/反向链接快 |
| 可选向量 (如 sqlite-vec / chroma) | embedding | 语义召回加速, 可后加可移除, 不是必需 |
| 可选图 (如 relation 表) | relations 遍历 | 双链在文件里已表达, 图只是加速器 |

存储 adapter 也做成 pluggable: 只要实现 MemoryStore 接口, 可以换 (文件+SQLite / 纯 SQLite / 未来服务化), 内核和接入层都不用动.

### 五、市面方案对比快表 (谁该抄, 谁该躲)

| 方案 | 抄什么 (设计思想) | 躲什么 (不照搬原因) |
|---|---|---|
| VCP | 写时召回 "先看见过去再续写"; Tag 河流; 占位符 DSL | 无测试, 单作者营销化, 封闭运行时, 算法过度复杂 |
| OpenViking | L0/L1/L2 分层; intent 检索; session commit | Rust+Python 重, AGPL, 引入第二套系统 |
| agentmemory | hooks 自动捕获; lesson 置信度×recency | 项目级 lesson (不跨项目), iii-engine 黑盒 |
| StrataGate | L0-L5 分层; 证据门; DSH 原生插件形态 | 34★ 早期, 记忆在 SQLite 与 MD 双份 |
| ReMe | 文件即记忆; daily→digest 自演进; BM25 免向量; DSH 插件 | 仍需独立 Python 服务; digest 单工作区 (不跨项目) |
| Basic Memory | typed Observations+Relations+wikilink | AGPL; 无自动捕获 |
| 腾讯 Hy-Memory | supersedes 演化链 (版本化记忆); 6 层框架 | 见下节玩具点 |
| Graphiti | 时序 KG + episode 溯源 | 重, 自建图库 |
| obsidian-second-brain | typed edges; index 前门; 44 命令编译到多 CLI | 以 Obsidian vault 为锚, 与你 ai-docs 结构不完全一致 |

### 五.1 代码核查: 市面"具体→一般"机制的真实分布

对本地克隆做了 grep + 源码核查, "把具体实例抽象成可复用原则"的机制并不为零, 但分布如下:

| 系统 | 机制 | 核查结论 |
|---|---|---|
| OpenViking | experiences 记忆类型 + experiences.yaml 明确**抽象强制**: "剥离实体/ID/人名/原始文本, 用泛化抽象描述使规则普遍适用"; Situation/Approach/Reflect 三段式; supersedes 字段 (替换旧经验并继承轨迹史); experience_lineage 链 | **同类中最强 R1** — 但绑单工作区, AGPL |
| agentmemory | 四层 consolidation (working→episodic→semantic→procedural) + mem::reflect ("综合跨越 2+ 条记忆的横切洞见") + 置信度 lesson + supersession 版本链 | **真 R1**, 但 lesson 按项目隔离, 不跨项目 |
| ReMe | dream 流水线: auto_memory→compressor→dream_extract ("one unit = one abstraction", digest=抽象记忆层)→dream_integrate (CREATE/CORROBORATE/REFINE/CORRECT→procedure/personal/wiki 桶) | **最接近"具体日记→可复用摘要"的自动提升**, 但 digest 绑单工作区, 且无用户级推广门 |
| VCP | AgentDream (DreamInsight 建新日记) — 核心 TagMemo/RiverMemo 引擎**零 generalize 逻辑** (grep 只有 hash 噪声) | R1 弱, 且需 admin 逐个审批 |
| StrataGate | 核心无 generalize 机制 (0 hits); 只记录"发生了什么/什么是真" | R1 缺 |
| Hy-Memory | supersedes 演化链 (同一认知对象版本演化) | 是版本史不是跨域推广 |

**自建层的差异化落点**: 前人有"摘要抽象", 无人做"**跨项目 + 用户确认门 + 双向链接(规则⇄实例) + 跨域生效**"——四条都补上才是用户要的"队列并发踩坑 → 我所有容器都有并发控制问题"的推广.

### 六、腾讯 Hy-Memory 的玩具点 (实测源码核查结论)

用户想了解 Hy-Memory, 并希望看到"它哪里 toy". 本调研对 pypi/npm 制品做了**解包读码**核查 (hy-memory 1.2.21 / hermes 0.2.8 / openclaw 1.2.4 / opencode 0.1.14):

**值得抄进自建层的真创新 (源码证实)**:
1. **supersedes 演化链是真实现**: 写入时 LLM reconciler 产出 ADD/SUPERSEDE/UPDATE op, SUPERSEDE 写双向指针 (新→supersedes=[旧], 旧→superseded_by=[新]+status=SUPERSEDED), 召回命中任意节点双向展开整条链, 删链有修复逻辑. 语义上"覆盖不丢史, 并列不碎片".
2. **召回单元 = 提炼后的高层记忆**, 原文 (L1_RAW) 只作影子存档 — "记忆减 70%, 密度增 45%"的根因.
3. **profile/normal/proactive 三通道分流** + over-fetch 再展开去重; BM25+向量 hybrid+RRF 兜底; 意图带 valid_until 到期惰性降级.
4. **sidecar 生命周期工程**: 插件自建 venv / ensureServer 复用 / 杀僵尸端口进程 / /health 真探活 / 写侧熔断 — 自写 harness 插件的好模板.
5. 中文 extractor/intention prompt 的分词规则、owner 归属、时间锚点纪律质量在线.

**Toy-like / 不成熟证据 (逐条有出处)**:
1. **发货制品全部挂在社区个人名下, 非腾讯官方**: pypi/npm 三包作者 = alvinfei (个人, 无 Tencent 关联), openclaw 版甚至是另一个个人 seventhsummer (README 自称"腾讯混元团队研发"); Tencent-Hunyuan org 80 仓库与 TencentCloud org 均无 hy-memory 代码 — 官方只做品牌背书, 不托管/不审计. 腾讯真正 org 化的是 TencentDB Agent Memory (另一产品, 官网还匿名拿它垫背当"某云平台记忆框架").
2. **层框架"名义层化"**: 代码里是 L0-L7 枚举, 但 **L5_KNOWLEDGE 文档自述"暂未实现"**; README 说 7 层, 官网说 6 层; 实际完整写路径只有 L1-L4+意图. 三处层号对不上 = PPT 化.
3. **System2 在发货版从不执行**: 机制真实但只在显式调用 digest() 时跑; server 与三个插件全代码 grep **无人调用 digest** — 营销讲的"后台夜班深度沉淀"在 shipped 形态下从不出现在线上.
4. **lite 模式 = 数据黑洞**: lite 只写 L1_RAW, 而三条召回通道都过滤 L1_RAW (代码证实) — 写得进永远召不回, 官方 FAQ 自己承认.
5. **benchmark 纯自报**: 85.20/76.91 无 eval 脚本/无评测条件/无复现材料; 独立使用者 (阅微堂 zhiqiang.org) 按官网装了两个月弃用 (记忆散乱、上下文被撑大).
6. **稳定性翻车实锤**: openclaw issue #102126 — server 跑约 1h 静默挂死, 每轮 prefetch 卡满 120s; issue #92743 autoRecall 把整个消息信封当搜索 query.
7. **工程文档/供应链欠账**: README 让装 hy-memory-internal 但该包 404; 62 天 34 个 release (5 月 29 个) 版本 churn 剧烈; hermes 排障提示 pip install one-memory (抄别家产品残留); --dangerously-force-unsafe-install; npm 插件运行时 SIGKILL 19527 端口别的进程; 依赖不锁版本.
8. **数据单向门**: 全量落 ~/.hy-memory/ (chroma+kuzu+sqlite 私有格式), 全包无 export/backup/migrate 工具.
9. **写路径成本失控**: pro/ultra 每批 = extractor LLM + reconcile LLM (>=2 次/批), 无任何成本上限开关.

**结论**: Hy-Memory 作为**设计思想来源**极有价值 (演化链/高层召回单元/通道分流/sidecar 生命周期都要抄), 作为**直接选型**风险高 (供应链不明、层与 System2 名不副实、无导出、无 DSH、稳定性问题), 与本调研"自维护 + 可插拔 + 带推广"的目标天然冲突 — 这反而印证了自建的必要性.

### 七、存储层具体表结构 (多库多表, 可插拔)

用户要求"存储可能分很多库很多表", 设计上把存储也抽象成多个可选 backend, 每个 backend 是 MemoryStore 的一个实现, 内核不关心它是文件还是库:

```text
MemoryStore (接口)
├─ FileBackend (默认): Markdown 真相 + 派生索引
│    ├─ <root>/daily/YYYY-MM-DD.md      # 每日捕获 (原始, 人可读)
│    ├─ <root>/digest/<topic>.md        # 整理后知识 (wikilink 双链)
│    ├─ <root>/rules/<rule>.md          # 推广后的跨项目规则 (generalizes 反向链接)
│    └─ <root>/index.sqlite             # 派生索引 (可重建, 不覆盖真相)
├─ SqliteBackend: 全部记忆进表 (适合不想管文件时)
│    ├─ memories(id, kind, scope, content, source, valid_at, asserted_at)
│    ├─ relations(from_id, rel, to_id, weight, asserted_at)
│    └─ tags(memory_id, tag)
├─ VectorBackend (可选加速): embeddings 表, 接在任一 backend 之上
└─ (未来) ServiceBackend: 远程/团队共享, 同样只是 MemoryStore 实现
```

关键不变量:
- **真相在文件 (若用 FileBackend), 索引可重建**: 删 index.sqlite 不影响记忆; 反向操作 (删文件) 才影响真相, 应被禁止/提示.
- **relations 有向带权**: 双链 [[wikilink]] 只是 relations 的一种 (relates_to); 推广引擎写 generalizes / applies_to.
- **双时态在每条记录**: valid_at (生效) + asserted_at (写入), 回答"以前是什么"用时间切片 (呼应 004 笔记).

### 八、DSH Adapter 完整事件接线 (当前版本即可实现)

已核实安装的 DSH 0.1.1-rc.2 的 dsh-agent-loop 就暴露 agent/session-start 与 turn/end 事件 (源码 lib/index.js 中均存在), 所以下列接线在当前版本即可实现, 无需升级:

```ts
export function apply(ctx: Context, cfg: Config) {
  // 1. 会话开始: 注入记忆使用指引 (不注入历史, 参考 ReMe guidance)
  ctx.on("agent/session-start", ({ agent }) => {
    if (cfg.rootAgentsOnly && agent.session.header?.origin === "subagent") return;
    ctx.inject(createUserMessage(memoryGuidance(cfg.language))); // plugin context
  });
  // 2. 轮次结束: 捕获完成对话 → 批量入记忆
  ctx.on("turn/end", ({ agent, data }) => {
    if (!cfg.autoCapture) return;
    captureTurn(agent, data); // queue → auto_memory (后台)
  });
  // 3. (可选) pre-step 轻量召回注入 (比 session-start 更细的注入点)
  ctx.on("agent/pre-step", async ({ agent, step }) => {
    if (!cfg.autoRecall) return null;
    return recallForStep(agent, step); // ≤ N tokens 相关记忆, 或 null
  });
  // 4. 注册工具
  ctx.effect(() => registerMemoryTools(ctx, client, cfg), "hx-memory.tools()");
  // 5. 后台整理 + 推广 (定时)
  ctx.on("schedule", () => scheduleConsolidationAndGeneralize(ctx, cfg));
}
```

这一节的价值: 把"可插拔"落到实处 — 内核 HarnessAdapter 接口对应 DSH 的 ctx.on 接线; 未来加 Codex/CLI 就是写第二个 adapter, 内核与存储零改动. 接入方式参考 StrataGate (inject: tools/systemPrompt/llm) 与 ReMe (inject: agents/sessions/tools + cordis patch isolate).

## 0x03 参考来源

- 腾讯 Hy-Memory 官网: https://memory.hunyuan.tencent.com/ (六层框架/演化链/评测声明)
- Hy-Memory OpenClaw/Hermes/Opencode 接入页: https://memory.hunyuan.tencent.com/openclaw 等 (安装/配置/模式细节)
- 极客公园: https://www.geekpark.net/news/365698 (Hy-Memory 机制与行业分析)
- chooseai: https://www.chooseai.net/news/4102/ (三层架构对标 mem0/Graphiti)
- GitHub: agentmemory (rohitg00/agentmemory), StrataGate (diqierjia/StrataGate-AgentMemory), ReMe (agentscope-ai/ReMe), VCP (lioensky/VCPToolBox), OpenViking (volcengine/OpenViking), Basic Memory (basicmachines-co/basic-memory)
- 本笔记姊妹篇: [004-对话记忆与知识库增量沉淀](../004-对话记忆与知识库增量沉淀/index.md) (ledger/views/policy + 双时态), [005-DSH Runtime设计思想](../005-DeepSeek%20Harness%20Runtime设计思想插件树与事件日志/index.md) (插件树/事件日志)
