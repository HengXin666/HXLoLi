# Agent Note: 外部源码项目作为一等素材类型, 且探索一律隔离到临时目录

Status: implemented

Decision-ID: repo-source-kind-and-scratch-isolation

- **引用落点**: 无源码引用 (约束的是 skill 文档的写法与 agent 的行为边界, 没有对应的代码声明可挂)

## Code

- `.agents/skills/hx-note/scripts/cli/flow/hx_flow.py`

## Problem

一次真实会话暴露了两个缺口。当时的请求是:
「`/hx-note` https://github.com/openvetta/open-vetta 学习一下这个项目的前端 / 客户端 / 整体 UI 设计…
你用视觉看一看, 最好把项目捆到本地…我希望能得到 1 个可复现的解决方案, 你甚至可以编写对应的 HTML」。

缺口的两个表现:

1. **没有"素材是源码项目"这条路。** `steps/2-collect/impl/` 原有六个类型 (article / video /
   local / research / custom / providers), 全是**文本类**素材。前端项目、UI 实现、单文件 HTML、
   组件库 playground 都不在内  而 `.html` / `.tsx` 在这个 skill 里**只作为产物形态出现**
   (演示页侧车, 见 `steps/6-derive/impl/tsx-deck.md`)。结果: 采集阶段的契约里没有一处告诉 agent
   "可以读源码、可以跑起来看、可以把关键实现抄成最小复现"。
2. **没有改动边界。** 全 skill 只有 `entries/organize/impl/` 里写了"只读体检", 那是**整理自己笔记**的
   约束。采集外部项目时没有任何规则说"不许改它、不许留东西在仓库里"。实测的表现是: 为了学一个前端
   项目的 UI, 最后在笔记仓库里留下了一棵演示工程的目录树  它既不是笔记也没人维护, 只在
   `git status` 里当噪声。

## Decision

**加一条素材路径 `repo`, 并在 SKILL.md 顶层写死产出边界。**

1. `--kind repo` + `steps/2-collect/impl/repo.md`。固定路线是
   **先看结构 -> 跑起来看 -> 再回去读实现 -> 只采可迁移那一层 -> 用最小 HTML/TSX 复现**。
   前端特有的采集点列了表 (设计令牌 / 动效 / 布局骨架 / 交互反馈 / 色彩),
   并明确"抄做法不抄数值"。
2. **SKILL.md 顶层加一条铁律**: 全程只往两处写  暂存区与最终那一篇 `index.md`;
   克隆/试跑/装依赖一律进 `/tmp/hx-<用途>-<slug>/`; **被学的项目一个字节都不改**,
   要试就在副本上试。

第 2 条写在 SKILL.md 而不是只写在 `impl/repo.md`, 是刻意的: 铁律必须在**动手之前**就在上下文里,
而 `impl/` 是走到采集那一步才读的。

## Alternatives considered

- **什么都不做, 复用 `article` 或 `custom`** — 最强理由: 源码项目最终也是"读一个 URL 然后整理要点",
  与 article 的形式相同; 加新类型等于给状态机加一个分支, 有维护成本。否决理由: 两者的**采集动作
  根本不同**  article 是抓文本, repo 是**克隆 + 跑起来 + 读代码 + 写最小复现**。
  把它塞进 article, 结果就是 agent 只抓 README 然后开始总结项目简介, 而用户要的是"这个效果怎么做出来的"。
- **只加 `impl/repo.md`, 不动 SKILL.md 的铁律** — 最强理由: 实现文档里已经写了"不改被学的项目、
  不改仓库", 一条规则写一处比写两处好维护。否决理由: 这条约束的失效代价不对称 
  漏读它会在用户的仓库里留下垃圾目录树, 而修复要靠人工清理。实测就是这样发生的。
  放在顶层, 它在**每一次会话的上下文里**都在。
- **给"临时目录"再抽一层共享约定** — 最强理由: 现在 `/tmp` 的用法散落在 `video.md` (抽帧)、
  `transcribe.md` (中间 transcript) 与本 note 三处, 抽成共享文档能统一。否决理由: 三处的**生命周期
  与清理时机各不相同** (抽帧要收尾清一遍、transcript 只在对话中保留、repo 副本可留到验证完),
  强行统一反而丢掉差异。等第四个用例出现再抽。

## Consequences

- 学习外部项目成为一等路径, 且与其它素材类型走同一套九步流程  步骤 3-9 完全不用改。
- 产出边界变成**可检查的**: 笔记仓库里除了 `index.md` 与同目录配图/侧车, 不该出现新目录。
  `entries/organize/impl/batch-audit.md` 的第 1、5 两项本来就在查"散装文件", 现在有了明确的判定依据。
- 代价: **这条约束没有门禁**。`/tmp` 与仓库之间的边界靠 agent 自觉, lint 抓不到"你克隆进了仓库"。
  真正的兜底是 `git status` 的 review 与上面那两项体检。
- description 里补了"外部源码项目"与"学习这个项目的前端或 UI 设计"两个触发词, 否则这类请求
  不会命中这个 skill。

## Verification

- `uv run .agents/skills/hx-note/scripts/cli/flow/hx_flow.py init --help` → `--kind` 含 `repo`;
- `uv run .agents/skills/hx-make-skill/scripts/validate_skill.py .agents/skills/hx-note` → PASS (0 errors, 0 warnings);
- `uv run .agents/skills/hx-note/scripts/cli/flow/hx_flow.py doctor` → 路径全部可达;
- `npm run verify-notes` → 五道门禁全绿。
