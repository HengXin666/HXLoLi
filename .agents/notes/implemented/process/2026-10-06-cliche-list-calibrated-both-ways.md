# Agent Note: 套话词表按人写/AI 的双向频次校准, 并区分「只约束派生文章」的词

Status: implemented

Decision-ID: cliche-list-calibrated-both-ways

- **引入于**: 本次改动 (待提交后回填该次提交的 sha)
- **引用落点**: `.agents/skills/hx-note/scripts/cli/voice/voice_rules.py` 的 `CLICHE_COUNTS`、`CLICHE_ARTICLE_ONLY` 与 `cliche-article` 规则

## Code

- `.agents/skills/hx-note/scripts/cli/voice/voice_rules.py`

## Problem

套话词表原先只有一句「新增词前先跑一遍对比统计」的口头约定, 没有记下**已完成**的统计。后果是每个后来者都要重新试一遍同样的候选词, 而且**很容易试错**  本次就差点把两个正常的词当 AI 词加进去:

| 候选词 | 人写 (blog + docs) | AI (ai-docs) | 判断 |
|---|---|---|---|
| `其实` | 124 次 / 82 篇 | 98 次 / 52 篇 | **错**。人写更多, 它是普通口语词 |
| `进行` | 1845 次 | 62 次 | **错**。人写 30 倍, 是正常动词 |
| `沉淀` | 2 次 / 1 篇 | 74 次 / 28 篇 | 对。37 倍差距 |

「人写不出现、AI 常出现」这个判据本身没错, 错在**只凭印象估**。`其实` 若凭印象加进去, 会误伤作者 82 篇手写笔记。

另一件事: 频次校准通过 ≠ 任何 profile 都能用。`沉淀` 在作者那 1 篇里是活用 (「想着一定要沉淀下来!」「可以沉淀到 Github Page 上」), 是具体动作而非空话。

## Decision

两条, 都落在 `voice_rules.py` 的数据层里:

1. **词表旁记双向频次与阈值, 不只记一句话的约定。** 新增 `CLICHE_COUNTS`, 每个词记 `(人写次, 人写篇, AI 次, AI 篇)`; 明确阈值: **人写篇数占比须低于 2%** (作者 977 篇人写语料, 即少于 20 篇) 才算合格。同时把三条被否决的候选词连数字一起写进注释  下一个 agent 看到 `其实` 的计数就不会再试它
2. **区分「全 profile」与「只约束派生文章」两类词。** 新增 `CLICHE_ARTICLE_ONLY` 与该词表承载的 `cliche-article` 规则 (E 级, 只挂 `article`)。判据: 频次校准通过, 但作者在随笔里会把该词当具体动词使用  这类词写进派生文章是空话, 写进作者自述是正常表达

## Alternatives considered

- **什么都不做, 维持口头约定** — 最强理由: 词表是人工维护的短名单, 每次加词时现跑一次统计并不麻烦, 把数字存进源码注释会让数据层变重。否决: 本次实测显示"现跑一次"并不足以防错  `其实` 与 `进行` 都在第一轮判断里被当成 AI 词, 是靠跑了统计才拦住的; 而跑过之后不记下来, 下一个人会重走一遍同样的错路
- **加 `--calibrate` 子命令自动跑统计** — 最强理由: 把"先统计再加词"从约定变成工具能力, 比记在注释里更强制。否决: 这需要 hx_voice 增加一个会遍历全仓语料的新入口, 而调用它的场合是**人加词**这种低频动作; 付出的接口面积与回归面不划算。注释里给数字已达到阻止重复试错的目的
- **把 `其实` `进行` 也加进 HARD 表, 靠豁免指令放行手写博客** — 最强理由: 它们在 AI 文本里的密度确实偏高, 是可观察的写作习惯。否决: 被豁免的对象会是**作者 977 篇手写语料里的大部分**。一个词表若要求对主体语料开豁免, 那它量的就不是套话, 而是"作者自己的用词习惯"; 那属于规范不同, 不该混进 AI 味判据
- **只加词, 不引入 profile 区分** — 最强理由: 少一个机制就少一处复杂度, `沉淀` 只误伤 1 篇, 可以接受。否决: 误伤的是**作者本人最自然的那句话**, 而这条规则的目的是筛 AI 腔而非筛作者; 一个把作者原话判违规的门禁会训练出「无视它」的习惯, 代价远大于多一个词表

## Consequences

- `ai-docs/` 的 article profile 全量由 1125 errors 升到 **1174 errors** (+49, 全部是 `沉淀`); warnings 1946 不变  差值**只来自新增词**, 无连带误伤
- 作者手写语料零误伤: `blog/` 全量跑 blog profile, `cliche-article` 命中 **0**
- 代价: 词表分裂成两份 (`CLICHE_HARD` 与 `CLICHE_ARTICLE_ONLY`), 加词时多一个"它该挂哪个 profile"的判断。判据已写进注释: 看作者在随笔里会不会把它当具体动词用
- 代价: `CLICHE_COUNTS` 与 `CLICHE_HARD` 是两份名单, 存在不同步的可能。当前只有 `沉淀` 一条记数, 未做一致性断言

## Testing

```
# 正向: article profile 该报
printf '把这次的经验沉淀成一篇笔记, 沉淀出可复用的做法.\n' > /tmp/a.md
uv run .agents/skills/hx-note/scripts/cli/voice/hx_voice.py lint --profile article /tmp/a.md
# -> E [cliche-article] 2 处, FAIL

# 反向: 作者手写 blog 不该报 (该篇原话含「一定要沉淀下来!」)
uv run .agents/skills/hx-note/scripts/cli/voice/hx_voice.py lint --profile blog \
  blog/2026/05/07/01_最近的项目.md
# -> PASS 0 errors, 0 warnings

# 全量回归: 作者语料零命中
uv run .agents/skills/hx-note/scripts/cli/voice/hx_voice.py lint --profile blog blog/ | grep -c cliche-article
# -> 0

# A/B 差值: 把 CLICHE_ARTICLE_ONLY 置空再跑, errors 应由 1174 回落到 1125
uv run .agents/skills/hx-note/scripts/cli/voice/hx_voice.py lint --profile article ai-docs/
```
