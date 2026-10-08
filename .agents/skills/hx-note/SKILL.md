---
name: hx-note
description: "HXLoLi 知识沉淀流水线: 采集素材 (视频/文章/外部源码项目/代码/调研) -> 写出面向知识库的原子知识点 .hx-info.md -> 人类审核 -> 定路径 -> 派生出面向人类阅读的 index.md -> AI 味盲审 -> 保真盲审 -> 落地注册, 由一个九步状态机强制不许跳步、产物缺失就拒绝推进; 另有两条独立入口: 整理存量笔记 (重分类/hxid/tag 倒排树) 与把视频音频转成 transcript。素材是一套可照做的做法时另出一份可复用规格 (触发场景 / 契约+机械判据 / 已定死的选择 / 写法偏好 / 可直接拷走的件). 配图走 drawio.svg, 去 AI 味有可机械检测的判据与可增长的表达语料库. Use when 用户说 沉淀到 ai-docs、写一篇 ai-docs、参考这个视频/文章/开源项目沉淀一下、学习这个项目的前端或 UI 设计、知识点、原子笔记、派生文档、写成文章、把这套规范沉淀下来、照着搭一个、可复用规格、怎么把这套做法写下来、去AI味、AI味、盲审、画个架构图、配图、演示页、整理一下目录、重新分类、tag 太碎、链接断了、总结这个视频、转写这个音频, 或要把链接/视频/代码变更/调研变成知识稿与阅读稿时."
license: MIT
disable-model-invocation: true
metadata:
  author: Heng_Xin
  version: "4.0"
---

# hx-note

沉淀一篇 ai-docs 笔记. **知识产物有两份; 可运行件是侧车, 不算第三份. **

```
素材 ──> .hx-info.md   面向知识库: 纯知识点, 自包含, 无铺垫
              │
              └─(派生)──> index.md   面向人类: 引入 / 图文正文 / 展望
```

`.hx-info.md` 是**唯一事实源**. `index.md` 只能换一种说法讲它已有的东西, 不新增技术论断; 需要新事实先回写那边. 这条由步骤 8 的保真盲审兜住

## 三条入口

| 入口 | 什么时候走 | 读这个契约 |
|---|---|---|
| **新增一篇** | 要把素材沉淀成新笔记 | `steps/1-intake/index.md` 起, 逐步往下 |
| **整理存量** | 重新分类 / 改名 / 搬家 / 整理 tag / 修断链 | `entries/organize/index.md` |
| **转写媒体** | 单纯要视频音频的 transcript 与总结, 不落盘 | `entries/transcribe/index.md` |

后两条**不重叠于九步**, 有各自的任务顺序, 不要把它们塞进流水线

## 九步

每步一个文件夹. `index.md` 是**契约** (只做这一件事 / 产物 / 过关条件); 具体做法在它下面的 `impl/`, 按素材类型或场景分文件

| # | 步骤 | 只关心 | 契约 |
|---|---|---|---|
| 1 | intake | 用户到底要什么 (落成可判定的验收条款) | `steps/1-intake/index.md` |
| 2 | collect | 素材里有什么 | `steps/2-collect/index.md` |
| 3 | atom | 先发散造出素材没有的结论, 再落成知识点 | `steps/3-atom/index.md` |
| 4 | align | 逼问对齐: 一次一题, 锐化模糊词 | `steps/4-align/index.md` |
| 5 | place | 放哪里 | `steps/5-place/index.md` |
| 6 | derive | 怎么讲给人听 | `steps/6-derive/index.md` |
| 7 | review | 说不说人话 | `steps/7-review/index.md` |
| 8 | fidelity | 有没有讲偏 | `steps/8-fidelity/index.md` |
| 9 | land | 站点认不认 | `steps/9-land/index.md` |

```bash
F="uv run .agents/skills/hx-note/scripts/cli/flow/hx_flow.py"
$F init --slug "<短标识>" --title "<标题>" --source "<URL 或路径>" --kind <类型>
$F status --slug "<slug>"        # 只打印当前一步该做什么, 读它的契约
$F done <stage> --slug "<slug>" --note "<结论>"   # 产物没落盘会被拒
$F place  --slug "<slug>" --to "ai-docs/<分类>/<NNN-标题>" --title "<标题>" --tag "<tag>"
$F brief  --slug "<slug>"        # 步骤 4: 把待审的题与目标契约端给人类 (贴进对话, 再停下等答)
$F cover  --slug "<slug>"        # 需求覆盖: 哪条验收条款零支撑 (= 从没被实现过)
$F doctor --slug "<slug>"        # 步骤 9: 全部交付闸门
```

**`status` 每次只打印当前一步. 照它做, 不要往前看, 不要顺手做下一步**  「一个步骤做了多件事」正是这套流程要消灭的问题

## 跨步骤的能力

不属于任何单步、被多步共用的东西放这里

| 能力 | 契约 | 被谁用 |
|---|---|---|
| **voice** 去 AI 味与盲审 | `shared/voice/index.md` | 步骤 3 / 6 / 7 / 8 |
| 平台 markdown 语法 (含演示页两种形态) | `shared/hxloli-md.md` | 步骤 6 写正文前读一次, 决定用哪种内嵌 |
| 产出物清单与本地联动 | `shared/pipeline.md` | 步骤 9 |
| 该用哪个模型 | `shared/model-selection.md` | 任何一步要传 `--model` 时 |

## 红线 (任何步骤都不得违反)

- **责任在人类.** 方向由人类把握, 你只辅助. 禁止自作主张开始沉淀, 禁止替人类拍板
- **先出阶段性成果再反问.** 有半成品再问, 不要问空问题. 一次只问当前 frontier, 每题附推荐答案
- **用户意图要先定位再动手.** 用户的话指代的是对话里前面的内容, 他在和人说话而不是填表. 收到「这个不行」时先把「这个」解析成具体对象并复述确认, 再改
- **读者视角铁律.** `index.md` 只写面向读者的内容. 生成过程、工具链、本地路径、验证步骤、provenance、审核提示一律不进正文; 审核要点只写进 `.hx-mitemite.md`
- **需求必须有可判定的验收条款.** 阶段 1 的 `source/goal.md` 里, 每条 `G` 都要写清"怎么算满足"  出现可执行动作 (查/跑/测/数/看, 或具体文件名与命令). 写不出检查动作的是愿望, 进"背景"一节. **需求只活在对话里 = 上下文一压缩就没了. ** `done intake` 会拦; `cover` 会在后面暴露"哪条需求从没被实现过"
- **一次只问一题.** 阶段 4 的对齐是**对话**, 不是问卷. 把 8 题带长理由一次性抛出去, 人类只会扫读或照抄推荐 (实测原话: 「他就让我在那里乱说」). 做法见 `steps/4-align/impl/grill.md`
- **先发散再落盘.** 素材是一套做法/规范/实现时, 步骤 3 必须**先**跑四个发散算子(`steps/3-atom/impl/divergence.md`) 产出 `source/divergence.md`, 再写 `.hx-info.md`. **不跑就会退化成复述**  模型读素材的默认行为就是换一种说法讲一遍. 闸门是硬的: `done atom` 在发散记录少于 4 条带判据产出时直接拒绝推进
- **审核必须真的发生.** 出完题要跑 `hx_flow.py brief --slug <slug>` 并把那一屏**贴进对话**  答题卡在点开头目录里, 人类看不见它. **推荐答案不是答案**: 它写在问题里让人能否决, 不是人类的答复. 未答的题会让 `done align` 直接拒绝推进
- **不臆造.** 查不到就说查不到; 无法确认的标 `[不确定]`; 不确定当前模型时 frontmatter 写`Unknown`. **画像里找不到真实关联时如实说找不到**  编造的共鸣比没有共鸣更伤
- **不跳盲审.** 步骤 7 与 8 各用一个**新鲜的** sub-agent, 且**顺序不可换** (先味道、后保真): 改文风会动句子, 动句子就可能带偏事实. 协议见 `shared/voice/impl/blind-audit.md`
- **知识产物只有两份; 但必须带能运行的件.** 事实源 `.hx-info.md` 与阅读稿 `index.md`, 只此两份. **这不等于"只能有两个文件"**  与 `index.md` **同目录**的侧车(`*-deck.tsx` 内联演示页 / `*.html` / `*.drawio.svg` / 复现件) 是笔记的一部分, 照常产出. 实测后果: 把这条读成"产物上限 2"时, 笔记会退化成纯散文  读者读完懂了原理, 却拿不到任何能照做的东西. 见 `steps/6-derive/impl/reusable-spec.md`
- **一份写不出可运行件的笔记, 参考价值是零.** 判据是机械的: `index.md` 里至少有一个带分组名的代码块, 或一个同目录侧车, 且它被一句话引进了论点. `doctor` 会拦 (见 `steps/9-land/impl/gates.md`)
- **仓库干净靠"位置", 不靠"数量".** 探索、克隆、试跑、装依赖一律在临时目录(`/tmp/hx-<用途>-<slug>/`), 不在 `ai-docs/` 下留"成品项目目录". 学外部项目时**一个字节都不改它**  要试就在副本上试. 见 `steps/2-collect/impl/repo.md`

## 脚本

| 脚本 | 用途 | 用在哪 |
|---|---|---|
| `scripts/cli/flow/hx_flow.py` | 九步状态机 (init/status/brief/cover/done/place/doctor/list) | 全程 |
| `scripts/cli/voice/hx_voice.py` | AI 味检测 (lint) / 合并折行 (fix) / 语料库 (learn/samples) | 步骤 3 / 6 / 7 |
| `scripts/lib/textpaths.py` | 上面两个文风脚本共用的路径展开 (收目录/通配符); 内部件, 不直接调用 | — |
| `scripts/cli/persona/hx_persona.py` | 构建用户画像 | 步骤 6 |
| `scripts/cli/illustration/hx_drawio.py` | JSON spec -> 可再编辑的 . drawio.svg | 步骤 6 |
| `scripts/cli/mitemite/hx_mitemite_add.py` `scripts/cli/mitemite/hx_mitemite_res.py` | 答题卡读写 | 步骤 4 |
| `scripts/cli/authoring/makeDoc.py` | 初始化 `index.md` 模板 (由 `place` 调用) | 步骤 5 |
| `scripts/cli/taxonomy/hxloli_tags.py` | tag 注册表 | 步骤 5 / 9, 存量整理 |
| `scripts/cli/textfmt/format_cn_punct.py` | 标点归一化 | 步骤 9, 存量整理 |
| `scripts/cli/identity/hx_docs_id.py` | hxid 分配与校验 | 存量整理 |
| `scripts/cli/transcribe/hx_look_video_prepare.py` | 解析输入、准备 transcript | 转写媒体 |
| `scripts/cli/voice/review-paragraphs.py` | 逐段测「格言化」(jev), 挑出该看哪几段 | 步骤 6 / 7 |

`scripts/cli/` 下按域分目录 (flow / voice / textfmt / taxonomy / identity / authoring / illustration / persona / mitemite / transcribe): 每个域的入口脚本可直接 `uv run`, 兄弟模块只被 import; `scripts/lib/` 是跨域共享库. 一个目录的文件数上限与预算声明机制见 `hx-make-skill/references/code-quality.md`: 目录确需超过 6 个文件时, 在该目录内放一份预算声明. 本 skill 有两处按这种方式声明

```text
assets/layout.json
steps/2-collect/impl/layout.json
```

## 模板与素材

`templates/interest-outlook.md` (默认文章结构) / `templates/problem-solution.md` (踩坑复盘) / `templates/reusable-spec.md` (照做型规格: 骨架 / 库 / 分层 / 写法偏好 / 可拷走的件) / `templates/_registry.md` (模板索引; 加一个文件即新增一种结构) / `assets/info-template.md` (info 骨架) / `assets/divergence-template.md` (发散记录格式; 算子见 `steps/3-atom/impl/divergence.md`) / `assets/goal-template.md` (目标与验收契约; 需求唯一事实源, 阶段 1 起由 `grill.md` 锐化术语) / `assets/voice-audit-prompt.md` 与 `assets/fidelity-audit-prompt.md` (两个盲审的 prompt) / `assets/deck-template.html` (演示页起点) / `assets/pre-commit-note.sh` (提交前自动跑标点归一化与语料检查, 再执行一次 v2 暂存区扫描并区分错误与待审核项; 提示不阻挡人工提交)

## 怎么扩展 (都不要改这个文件)

| 要加什么 | 加在哪 |
|---|---|
| 新素材类型 | `steps/2-collect/impl/<类型>.md` + `hx_flow.py` 的 `--kind` 加一个取值 |
| 新文章结构 | `templates/<结构>.md` + 在 `templates/_registry.md` 登记 |
| 新的一类"可照做"素材 | 在 `steps/6-derive/impl/` 加一个 `reusable-spec-example-<领域>.md` (照 `reusable-spec.md` 的五层填) |
| 新画像数据源 | `steps/2-collect/impl/providers.md` |
| 新 AI 味规则 | `ai-docs/.hx-voice.toml` 加一条 (`hx_voice.py learn --bad --pattern`) |
| 新步骤 | `hx_flow.py` 的 `STAGES` 表加一行 + 新建 `steps/<N>-<key>/index.md` |
| 新入口 | 新建 `entries/<名>/index.md` |
| **把一篇笔记沉淀成 skill** | 用 `hx-make-skill`; 笔记里的可复用规格节就是它的初稿  先用 `templates/reusable-spec.md` 把做法写成契约+约定+偏好, 再按 skill 的规范落成目录 |

步骤之间只靠文件传递, 所以任何一步都能在一个新鲜 agent 里从磁盘恢复
