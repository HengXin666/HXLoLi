# Agent Note: 写作规则写前端出, 闸门只核对同一份清单

Status: implemented

Decision-ID: rules-feed-forward-gates-confirm

- **引入于**: <short-sha>
- **引用落点**: `.agents/skills/hx-note/scripts/cli/voice/voice_rules.py` 的 `writing_rules` / `CONTENT_RULES`; `.agents/skills/hx-note/scripts/cli/flow/flow_read.py` 的 `print_status`
- 修订: [2026-09-28-human-review-must-actually-happen.md](2026-09-28-human-review-must-actually-happen.md)、[2026-09-28-divergence-operators-gate.md](2026-09-28-divergence-operators-gate.md)、[2026-09-28-requirements-as-acceptance-clauses.md](2026-09-28-requirements-as-acceptance-clauses.md) 里「写在文档里的要求没人执行, 所以改用脚本闸门」这条推理的适用范围. 三篇的闸门本身不撤

## Code

- `.agents/skills/hx-note/scripts/cli/voice/voice_rules.py`
- `.agents/skills/hx-note/scripts/cli/flow/flow_read.py`
- `.agents/skills/hx-note/scripts/cli/flow/flow_core.py`

## Problem

使用者原话 (2026-11-08): 「别一直靠红线了, 我发现一开始写清楚规则, 比一开始不告诉他, 后续又拦截 要好1万倍」.

这和 09-28 三篇的推理表面相反. 那三篇的证据是: 要求写在 step 文档里, 照样被跳过 (16 份答题卡里 11 份 `**A**:` 没人答). 结论是「缺的不是句子而是闸门」.

把两边的证据放在一起看, 冲突点在于失败的是哪一环:

- 09-28 失败的是**规则不在执行者眼前**. 「停下等人类」写在 `steps/4-align/` 里, 那是按需加载的文档, 执行者到那一步时不一定读. 闸门修好了它, 原因是闸门的报错把规则**送到了眼前**, 并不在于「拦」本身
- 11-08 失败的是**规则只在事后出现**. `section-visual` (每段必须有图/表/代码)、`prose-budget` (纯文字 30 行)、doctor 的「必须有可复用物」只在 lint / doctor 时才露面. 执行者先按自己的理解写完, 撞上闸门后为了过关去补东西. `media.md` 甚至写着生图不可用时「按 drawio -> 表格 -> 代码块的顺序换一种视觉, 不要退成分点文字」. 补出来的就是使用者说的「傻逼配图」和「虚假的 code」
- 同日对照实验: 带 50 字引入 / 30 行预算的 A 版, 在 3 个主题、6 次成对盲审里全部输给无预算的 B 版, 输在收获 (5-6 对 8-9). 这份数据落在 `ai-docs/.hx-staging/hx-note-trial-20261108/`

所以两条证据说的是同一件事: **规则要在动笔那一刻出现在执行者面前**. 09-28 用事后闸门间接做到了这一点; 对内容规则来说, 事后才出现的代价是返工和为过关而写的填充物.

## Decision

把规则分两类, 分别处理.

### 一、过程不变量: 保留闸门 (09-28 的结论不变)

这类规则检查的是「某个动作有没有发生」: 验收条款有没有写、发散有没有跑够 4 条、人类有没有回答、迁移有没有落到画像上. 模型的动机是跳过这些动作, 事前告诉它也照样会跳. 闸门留在 `done` 上.

### 二、写作规则: 写前端出, 事后只核对

这类规则管的是「怎么写」: 表格、段长、句式、套话、代码出处、配图种类、保真.

- **单一来源.** `voice_rules.py` 里 lint 的 E 级规则加上 `CONTENT_RULES` (lint 管不到、由 doctor 或盲审判的规则), 由 `writing_rules(profile)` 汇总成一份清单
- **写前端出.** `hx_flow.py status` 在阶段 3 (atom) 和阶段 6 (derive) 的「做什么」下面, 逐条打印这份清单. status 本来就是每一步必须调用的入口 (同 09-26 语料强制注入的理由), 所以它仍然由脚本保证到场, 不依赖模型自觉
- **事后只核对同一份清单.** lint / doctor 的报错文字与写前清单逐字同源. 规则表外不许出现「只在事后才露面」的写作规则: 新增一条写作规则, 必须加进 `RULES` 或 `CONTENT_RULES`, 它才会同时出现在写前清单里
- **只能靠「加内容」才能通过的写作闸门一律撤掉.** 判据是: 把这条闸门的最省事过法写出来, 如果过法是「补一张图 / 补一段代码 / 补一张表」, 它就会制造填充. 已停用 `section-visual`、`text-ratio`、`prose-budget`; doctor 的「可复用物」降为只报告

## Alternatives considered

- **全部回到文档约束, 撤掉所有闸门.** 最强理由: 使用者说事前写清楚好 1 万倍. 否决: 09-28 的空答题卡是真实证据. 「有没有问人类」这种动作, 写得再清楚也会被跳过. 使用者反对的是「先不告诉, 后拦截」, 并没有说「不许检查」
- **保持现状, 只把闸门报错写得更清楚.** 理由: 改动最小. 否决: 报错写得再清楚也是事后的, 照样要返工. 而且只要过关的办法是补内容, 就会持续产出填充物
- **把规则写进 `SKILL.md` 红线.** 理由: SKILL.md 每次都会加载. 否决作为唯一手段: SKILL.md 已经很长, 而且它和 lint 是两份文本, 迟早会对不上. 红线仍然保留一行摘要, 但完整清单以 `writing_rules` 为准

## Consequences

- 执行者在阶段 3 / 6 动笔前, 能看到 lint 和 doctor 会查的每一条
- 代价: status 在这两步多打 12 到 14 行. 09-28 那篇曾以「status 输出量」为由不自动打印待答题; 这里接受这个代价, 因为写作规则只在两步出现, 且是动笔前必读的
- 已同步: 规则表里每条 `why` 改成写给作者的指引 (先说怎么写, 再附一句依据), 同一段文字既是写前清单也是 lint 报错. skill 文档里所有「用表格」「每段一个主视觉」「30 行预算」的写法已清掉; 分镜闸门 `storyboard_gate` 不再要求每段填主视觉, 只要求每段写出要证明的那句话
- 未做: 盲审 (阶段 7 / 8) 的模板没有接入写前清单. 盲审者故意不给规则 (防污染), 这一点不变

## Testing

```
python3 -m py_compile flow_core.py flow_read.py voice_rules.py   -> OK
hx_flow.py status --slug ai-visual-verification (当前在阶段 6)   -> 「写前规则」下打印 14 条, 含禁表格、段长上限 150、代码出处、配图只画结构、保真
writing_rules("atom")                                            -> 12 条 (不含只管 article 的禁表格与段长)
```

尚未验证: 写前清单对产出质量的实际影响. 第一份对照数据会来自 2026-11-08 的 45 篇红线改写. 那次是先把规则全文交给改写者再动手, 回收后看 lint 的 table / long-para-hard 是否一次归零
