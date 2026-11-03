# hx-note 改造计划 (待认)

状态: 2026-11-01 按"先试"的口径落地 P1 到 P4, 未当定稿; 实跑对照见仓库 `.trial/hx-note-trial/REPORT.md`

## 现状基线

- 三道门禁 `validate_skill.py --strict` / `prose_rules.py --check` / `check_layout.py` 现在全绿, 改完必须仍然全绿
- `scripts/cli/flow/` 与 `scripts/cli/voice/` 各 6 个文件, 已满额; `flow_core.py` 288 行, 逼近 300 行上限
- 所以新检查只能并进现有文件里还有余量的那几个 (`flow_steps.py` 128 行, `hx_voice.py` 90 行), 或新开一个域目录

## 诊断

### 1. AI 味重: 只有"禁", 没有"仿"

- 去 AI 味全靠 `voice_rules.py` 的黑名单加 lint. 黑名单只能堵已知指纹, 模型会换一种同样机器的说法 (删了破折号就上冒号排比)
- 正向样本只有 `.hx-voice.toml` 里零散的单句, 没有整段的作者真文让模型"照着这个语气写"
- 写作顺序反了: 先产出结论句式的 `.hx-info.md`, 再"换一种说法"派生 `index.md`, 天然就是翻译腔. 作者自己写是 遭遇 到 画面 到 事实
- `templates/interest-outlook.md` 14KB, 规则、反例、口播对照、理由全混在一起. 模型的注意力被流程和规则吃掉, 写出来是"合规的僵硬"

### 2. 换个 AI 产出就变

- 采集步骤写死了 `argo_fetch` / `wide_research` / `argo_search` 这类特定工具名 (`steps/2-collect/impl/article.md` 与 `steps/2-collect/impl/research.md`). 别的 agent 没有这些工具, 就各自即兴, 采集质量从源头开始分叉
- 版式靠形容词约束 ("生动" "不要大段"), 没有可数的槽位与预算. 不同模型对形容词的理解差别很大, 对数字的理解几乎一致
- 九步状态机管住了"顺序", 没管住"形状": 每段几张图、几行字、几个可验证物, 只有 `0xNN 段至少一个可验证物` 这一条是硬的

### 3. 图文并茂难做到: 先写字, 后配图

- 步骤 6 先写正文, 再按 `steps/6-derive/impl/media.md` 找地方配图. 图成了事后补丁, 文字已经把意思讲完, 图只能重复
- 生图后端三条全不可用时降级成代码块或分点, 结果又回到纯文字
- article 默认禁表格 (`voice_rules.py` 的 `table` 规则是 E 级). 一张 3 到 5 行的对比表本来能省掉一大段散文, 现在只能改写成文字或代码块

### 4. 资源广度与权威性: 来源不分级

- `steps/2-collect/impl/research.md` 有"证据分级", 但分的是结论的确定度, 不是来源的权威度. 官方规范、维护者博客、社区帖子在 `provenance.md` 里是同一种写法
- 没有广度门槛: 一个核心论断只有一个来源也能过 `done collect`
- 没有主动找一手来源的步骤: 读到一篇二手文章就停, 不会回溯到它引用的规范、论文、源码

## 方案 (四步, 按顺序做)

### P1 给"仿"的东西, 并把写作顺序摆正

- 新增作者真文样本: 从 `blog/` 挑 3 到 5 段作者亲手写的整段文字, 放进 `shared/voice/` 下的样本文件, 每段附一句"这段好在哪"
- 步骤 6 动笔前必读样本, 先仿写引入段给人看, 语气对了再往下写
- `index.md` 的写法改为 遭遇 到 画面 到 事实: 先从 `intake.md` 的遭遇和分镜 (见 P3) 起稿, 事实再从 `.hx-info.md` 取. 保真盲审 (步骤 8) 不变, 照样兜事实
- 把 `templates/interest-outlook.md` 拆成两份: 骨架 (只留槽位与硬判据, 目标 3KB 以内) 加理由与反例 (移进 `steps/6-derive/impl/`, 只在盲审不过时才读)
- 黑名单与 lint 保留, 当地板用, 不再当唯一手段

### P2 把"换个 AI 就变"的地方换成能力描述加数字

- 采集文档里的工具名改成能力描述加降级链: "抓正文" 优先用会话里已有的抓取工具, 没有就 curl, 再不行就转 `steps/2-collect/impl/custom.md`. 写法照 `steps/6-derive/impl/media.md` 的生图后端探测表
- 版式约束全部改成可数的预算: 每个 `0xNN` 段 1 个主视觉, 连续纯文字不超过 4 行, 全文正文字数给一个上限 (具体数字 P3 定)
- 能机械判的都进 lint 或 `doctor`, 不留给模型自己理解

### P3 图先行, 文字压到最少

- 步骤 6 先出分镜 `source/storyboard.md`: 每个 `0xNN` 段一行, 写清 这一段的主视觉是什么 (drawio / 截图 / 生图 / 小表格 / 代码块) 加一句话它要证明什么. 分镜过了人类再写字
- 正文只写连接图与图的那几句: 引入、每张图后面那一句引用、展望
- 新增 lint 规则 `text-ratio`: 一个 `0xNN` 段里纯文字行数与可视物数的比超过阈值就报 (W 级起步, 跑几篇稳定后升 E)
- 表格放开到 5 行以内: `table` 规则改成"超过 5 行才报 E", `--allow-table` 留着给更大的表. 同步改 `templates/interest-outlook.md` 的"禁止表格"一条 **(这一条等你拍板)**
- 生图后端全不可用时, 降级顺序改为 drawio 到 小表格 到 代码块, 不再直接退成分点文字

### P4 来源分级加广度门槛

- `provenance.md` 每个来源加一列权威等级
  - T1 一手: 规范 / RFC / 官方文档 / 源码 / 论文 / 标准组织
  - T2 权威二手: 维护者或核心作者的文章、知名机构的评测
  - T3 社区: 论坛、个人博客、问答
- 采集时加"回溯一手"动作: 读到二手来源就顺着它的引用去找 T1, 找不到在 `provenance.md` 的已知缺口里写明
- 广度门槛 (先只对 `--kind research` 与 `article` 生效)
  - 每个核心论断至少 1 个 T1, 或 2 个互相独立的 T2
  - 全稿至少 3 个独立来源
  - 至少记 1 条反例或反对意见
- `done collect` 加一道检查: 数 `provenance.md` 的等级列与 `material.md` 的定位是否对得上. 代码放跨域库 `scripts/lib/sources_core.py`, 入口并进 `hx_flow.py sources` 子命令, 不往已满的 `flow/` 里加文件 (远端写接口不能建目录, 新域目录放弃)
- 正文里按等级写法区分: T1 直接陈述, T2 写"据 某某", T3 只当线索不当依据

## 改动面与门禁

- 每一步的改动只落在对应的 `steps/*/impl/`、`templates/`、`shared/voice/` 与新域目录, 尽量不改 `SKILL.md`. 必须改时只改索引表的一行
- 每一步结束跑一遍

```bash
uv run hx-make-skill/scripts/validate_skill.py hx-note --strict
uv run hx-make-skill/scripts/prose_rules.py --check hx-note
uv run hx-make-skill/scripts/check_layout.py hx-note
```

- 全部做完后按 `hx-make-skill` 第六步做一次 forward-test: 用一个新鲜 agent 拿一篇真实素材走完九步, 看产出的图文比与 AI 味

## 等你拍板

- 方案整体认不认?
- 表格放开到 5 行以内, 可以吗?
